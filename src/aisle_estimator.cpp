/*********************************************************************
 * AC-TEB: online aisle (corridor) estimator. See aisle_estimator.h
 *********************************************************************/
#include "ac_teb_local_planner/aisle_estimator.h"

#include <algorithm>
#include <cmath>

namespace ac_teb_local_planner
{

namespace
{
inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

//! Quantile (0..1) of a vector (modifies its order).
inline double quantile(std::vector<double>& v, double q)
{
  const std::size_t k = static_cast<std::size_t>(
      std::floor(std::min(1.0, std::max(0.0, q)) * static_cast<double>(v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}
}  // namespace

// ---------------------------------------------------------------------------
AisleMeasurement AisleEstimator::measure(const std::vector<Eigen::Vector3d>& obstacles,
                                         const std::vector<Eigen::Vector3d>& ground,
                                         double prior_psi, double width_prior,
                                         double width_prior_std) const
{
  AisleMeasurement out;
  out.prior_psi = prior_psi;
  if (p_.slice_length <= 0.0 || p_.s_max <= p_.s_min)
    return out;

  const int nb = static_cast<int>(std::ceil((p_.s_max - p_.s_min) / p_.slice_length));
  const double ca = std::cos(prior_psi), sa = std::sin(prior_psi);

  std::vector<std::vector<double>> left(nb), right(nb);   // |l| of wall points
  std::vector<std::vector<double>> gleft(nb), gright(nb); // |l| of ground points

  auto bin = [&](const Eigen::Vector3d& p, double& l) -> int {
    const double s = ca * p.x() + sa * p.y();
    l = -sa * p.x() + ca * p.y();
    if (s < p_.s_min || s >= p_.s_max || std::abs(l) > p_.lateral_max || std::abs(l) < 1e-3)
      return -1;
    return std::min(nb - 1, static_cast<int>((s - p_.s_min) / p_.slice_length));
  };

  double l;
  for (const auto& p : obstacles)
  {
    const int k = bin(p, l);
    if (k < 0) continue;
    (l > 0 ? left[k] : right[k]).push_back(std::abs(l));
  }
  for (const auto& p : ground)
  {
    const int k = bin(p, l);
    if (k < 0) continue;
    (l > 0 ? gleft[k] : gright[k]).push_back(std::abs(l));
  }

  // edge samples: s, signed l, weight, side (0 left, 1 right).
  // Wall edge = low quantile of the obstacle points. Where a slice has no
  // wall, the edge is the lateral end of the ground (ground/obstacle junction),
  // taken as a high quantile so isolated far ground points do not count.
  // The ground edge is accepted only if some obstacle points lie just
  // beyond it (a real junction). Otherwise the ground ends because the camera
  // stops seeing it (image border, close to the robot) or because there is no
  // row at all, and that end must not be taken as a wall.
  struct Sample { double s, l, w; int side; };
  std::vector<Sample> walls[2], gedges[2];
  const bool use_ground = p_.ground_edge_weight > 0.0;
  const double ground_quantile = 0.95;
  const double junction_in = 0.03;    // obstacle may start slightly inside the ground edge [m]
  const double junction_out = p_.junction_max_gap;
  const int junction_min_pts = p_.junction_min_points;
  auto isJunction = [&](const std::vector<double>& obst_abs_l, double g) -> bool {
    if (junction_min_pts <= 0)
      return true;
    int n = 0;
    for (double o : obst_abs_l)
      if (o >= g - junction_in && o <= g + junction_out && ++n >= junction_min_pts)
        return true;
    return false;
  };
  for (int k = 0; k < nb; ++k)
  {
    const double s = p_.s_min + (k + 0.5) * p_.slice_length;
    const std::size_t min_pts = static_cast<std::size_t>(std::max(1, p_.min_points_per_slice));
    const std::size_t min_ground = 10;
    if (left[k].size() >= min_pts)
      walls[0].push_back({s, quantile(left[k], p_.edge_quantile), 1.0, 0});
    else if (use_ground && gleft[k].size() >= min_ground)
    {
      const double g = quantile(gleft[k], ground_quantile);
      if (isJunction(left[k], g))
        gedges[0].push_back({s, g, p_.ground_edge_weight, 0});
    }
    if (right[k].size() >= min_pts)
      walls[1].push_back({s, -quantile(right[k], p_.edge_quantile), 1.0, 1});
    else if (use_ground && gright[k].size() >= min_ground)
    {
      const double g = quantile(gright[k], ground_quantile);
      if (isJunction(right[k], g))
        gedges[1].push_back({s, -g, p_.ground_edge_weight, 1});
    }
  }
  // slices with an edge per side (wall or ground)
  const int nL = static_cast<int>(walls[0].size() + gedges[0].size());
  const int nR = static_cast<int>(walls[1].size() + gedges[1].size());
  out.n_left = nL;
  out.n_right = nR;
  out.n_wall_left = static_cast<int>(walls[0].size());
  out.n_wall_right = static_cast<int>(walls[1].size());
  for (int side = 0; side < 2; ++side)
    for (const auto& w : walls[side])
      out.s_wall_max = std::max(out.s_wall_max, w.s);
  for (int k = 0; k < nb; ++k)
    if (gleft[k].size() + gright[k].size() >= 20)
      out.s_ground_max = std::max(out.s_ground_max, p_.s_min + (k + 0.5) * p_.slice_length);

  // Decide the measurement mode
  std::vector<Sample> samples;
  int mode = -1;   // 0 two sides, 1 left only, 2 right only
  if (nL >= p_.min_slices && nR >= p_.min_slices)
  {
    mode = 0;
    for (int side = 0; side < 2; ++side)
    {
      samples.insert(samples.end(), walls[side].begin(), walls[side].end());
      samples.insert(samples.end(), gedges[side].begin(), gedges[side].end());
    }
  }
  else if (width_prior > 0.0)
  {
    // single side: enough edges, and at least some of them real wall slices
    // (ground junctions alone are too weak to carry the whole measurement)
    const int min_walls = std::max(2, p_.min_slices / 2);
    const bool okL = nL >= p_.min_slices && out.n_wall_left >= min_walls;
    const bool okR = nR >= p_.min_slices && out.n_wall_right >= min_walls;
    if (okL || okR)
      mode = (okL && (!okR || nL >= nR)) ? 1 : 2;
  }
  if (mode == 1 || mode == 2)
  {
    samples = walls[mode - 1];
    samples.insert(samples.end(), gedges[mode - 1].begin(), gedges[mode - 1].end());
  }
  if (mode < 0)
    return out;

  // Huber-IRLS fit. Two walls: theta = [m, b_L, b_R]; one wall: [m, b]
  const int np = mode == 0 ? 3 : 2;
  const int N = static_cast<int>(samples.size());
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, np);
  Eigen::VectorXd y(N), w0(N), w(N);
  for (int i = 0; i < N; ++i)
  {
    A(i, 0) = samples[i].s;
    A(i, mode == 0 ? 1 + samples[i].side : 1) = 1.0;
    y(i) = samples[i].l;
    w0(i) = samples[i].w;
  }
  w = w0;
  Eigen::VectorXd theta = Eigen::VectorXd::Zero(np);
  Eigen::MatrixXd AtWA;
  for (int it = 0; it < 6; ++it)
  {
    AtWA = A.transpose() * w.asDiagonal() * A;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(AtWA);
    if (ldlt.info() != Eigen::Success)
      return out;
    theta = ldlt.solve(A.transpose() * w.asDiagonal() * y);
    const Eigen::VectorXd r = y - A * theta;
    for (int i = 0; i < N; ++i)
    {
      const double ar = std::abs(r(i));
      w(i) = w0(i) * (ar <= p_.huber_delta ? 1.0 : p_.huber_delta / ar);
    }
  }
  AtWA = A.transpose() * w.asDiagonal() * A;
  const Eigen::VectorXd r = y - A * theta;
  double sigma2 = (w.array() * r.array().square()).sum() / std::max(w.sum() - np, 1.0);
  sigma2 = std::max(sigma2, 1e-4);   // >= (1 cm)^2
  const Eigen::MatrixXd cov_fit = sigma2 * AtWA.inverse();

  // Express both cases as [m, b_L, b_R] with covariance
  const double m = theta(0);
  const double sq = std::sqrt(1.0 + m * m);
  double bL, bR;
  Eigen::Matrix3d cov_theta = Eigen::Matrix3d::Zero();
  if (mode == 0)
  {
    bL = theta(1);
    bR = theta(2);
    cov_theta = cov_fit;
  }
  else
  {
    // the other wall at W_prior from the visible one
    const double off = width_prior * sq;
    bL = mode == 1 ? theta(1) : theta(1) + off;
    bR = mode == 1 ? theta(1) - off : theta(1);
    Eigen::Matrix<double, 3, 2> J;   // d[m, bL, bR]/d[m, b]
    J << 1, 0, 1, 1, 1, 1;
    cov_theta = J * cov_fit * J.transpose();
    const double v_off = (width_prior_std * sq) * (width_prior_std * sq);
    (mode == 1 ? cov_theta(2, 2) : cov_theta(1, 1)) += v_off;
  }

  const double c = 0.5 * (bL + bR);
  const double width = (bL - bR) / sq;
  if (!(width > 0.0) || !std::isfinite(width))
    return out;

  out.z << c / sq, wrapAngle(prior_psi + std::atan(m)), width;
  // Jacobian d[e, psi, W] / d[m, bL, bR]
  Eigen::Matrix3d G;
  const double sq3 = sq * sq * sq;
  G << -c * m / sq3, 0.5 / sq, 0.5 / sq,
       1.0 / (1.0 + m * m), 0.0, 0.0,
       -(bL - bR) * m / sq3, 1.0 / sq, -1.0 / sq;
  out.R = G * cov_theta * G.transpose();
  out.R(0, 0) += p_.meas_floor_e * p_.meas_floor_e;
  out.R(1, 1) += p_.meas_floor_psi * p_.meas_floor_psi;
  out.R(2, 2) += p_.meas_floor_w * p_.meas_floor_w;
  if (mode != 0)
  {
    // the width is not observed: decouple it so that W is not updated
    out.R.row(2).setZero();
    out.R.col(2).setZero();
    out.R(2, 2) = 1e6;
  }
  out.single_side = mode;
  out.m = m;
  out.b_left = bL;
  out.b_right = bR;
  out.ok = true;
  return out;
}

// ---------------------------------------------------------------------------
Eigen::Vector3d AisleEstimator::predictState(const Eigen::Vector3d& pose) const
{
  // the aisle is an undirected line: use the direction closest to the heading
  double phi = phi_;
  if (std::cos(phi - pose.z()) < 0.0)
    phi += M_PI;
  const Eigen::Vector2d n(-std::sin(phi), std::cos(phi));
  const Eigen::Vector2d pr(pose.x(), pose.y());
  return Eigen::Vector3d(n.dot(q_ - pr), wrapAngle(phi - pose.z()), width_);
}

void AisleEstimator::storeLine(const Eigen::Vector3d& x, const Eigen::Vector3d& pose)
{
  phi_ = wrapAngle(pose.z() + x(1));
  const Eigen::Vector2d n(-std::sin(phi_), std::cos(phi_));
  q_ = Eigen::Vector2d(pose.x(), pose.y()) + x(0) * n;
  width_ = x(2);
}

bool AisleEstimator::update(const std::vector<Eigen::Vector3d>& obstacles,
                            const std::vector<Eigen::Vector3d>& ground,
                            const Eigen::Vector3d& pose, double t)
{
  Eigen::Vector3d x_pred = Eigen::Vector3d::Zero();
  double prior_psi = 0.0;
  // A line that has not been confirmed for a while is dropped instead of being
  // extrapolated: a small heading error would otherwise drift laterally with
  // the distance travelled (~tan(psi) per metre) and be drawn off the walls.
  if (initialized_ &&
      ((Eigen::Vector2d(pose.x(), pose.y()) - r_accept_).norm() > p_.hold_distance ||
       (t - t_accept_) > p_.timeout))
  {
    reset();
  }
  if (initialized_)
  {
    // flipping the line direction changes the sign of e_y (and of its
    // correlation with psi)
    if (std::cos(phi_ - pose.z()) < 0.0)
    {
      const Eigen::Matrix3d T = Eigen::Vector3d(-1.0, 1.0, 1.0).asDiagonal();
      P_ = T * P_ * T;
      phi_ = wrapAngle(phi_ + M_PI);
    }
    x_pred = predictState(pose);
    const double dt = std::max(0.0, t - t_last_);
    P_(0, 0) += p_.proc_noise_e * p_.proc_noise_e * dt;
    P_(1, 1) += p_.proc_noise_psi * p_.proc_noise_psi * dt;
    P_(2, 2) += p_.proc_noise_w * p_.proc_noise_w * dt;
    prior_psi = x_pred(1);
  }
  else if (has_hint_)
  {
    // not initialised yet: slice along the global plan direction
    prior_psi = wrapAngle(hint_phi_ - pose.z());
  }
  t_last_ = t;

  last_meas_ = initialized_
                   ? measure(obstacles, ground, prior_psi, width_, std::sqrt(std::max(0.0, P_(2, 2))))
                   : measure(obstacles, ground, prior_psi);
  if (initialized_)
    updateRowEnd(pose);
  if (!last_meas_.ok)
    return false;
  const Eigen::Vector3d& z = last_meas_.z;
  const Eigen::Matrix3d& R = last_meas_.R;
  // A new aisle is only recognised from BOTH rows and a narrow width: in the
  // central corridor (no rows, one row, or wide) nothing is initialised and
  // the controller is the plain TEB.
  const bool can_init = last_meas_.single_side == 0 && z(2) >= p_.min_width && z(2) < p_.width_ref;

  if (!initialized_)
  {
    if (!can_init)
      return false;
    P_ = R;
    has_end_ = false;
    accept(z, pose, t);
    initialized_ = true;
    rejects_ = 0;
    return true;
  }

  Eigen::Vector3d nu = z - x_pred;
  nu(1) = wrapAngle(nu(1));
  const Eigen::Matrix3d S = P_ + R;
  const Eigen::Matrix3d S_inv = S.inverse();
  const double d2 = nu.dot(S_inv * nu);
  if (!(d2 <= p_.gate_chi2))
  {
    if (++rejects_ >= p_.reinit_after && can_init)
    {
      // persistent disagreement: the robot entered another aisle
      P_ = R;
      has_end_ = false;
      accept(z, pose, t);
      rejects_ = 0;
      return true;
    }
    return false;
  }

  const Eigen::Matrix3d K = P_ * S_inv;
  Eigen::Vector3d x = x_pred + K * nu;
  x(1) = wrapAngle(x(1));
  P_ = (Eigen::Matrix3d::Identity() - K) * P_;
  P_ = 0.5 * (P_ + P_.transpose());
  accept(x, pose, t);
  rejects_ = 0;
  return true;
}

void AisleEstimator::accept(const Eigen::Vector3d& x, const Eigen::Vector3d& pose, double t)
{
  storeLine(x, pose);
  t_accept_ = t;
  r_accept_ = Eigen::Vector2d(pose.x(), pose.y());
}

void AisleEstimator::updateRowEnd(const Eigen::Vector3d& pose)
{
  // End of the row ahead: ground is seen at least end_ground_gap beyond the
  // farthest wall slice (on both sides). Walls seen farther again clear it.
  const AisleMeasurement& m = last_meas_;
  double phi = phi_;
  if (std::cos(phi - pose.z()) < 0.0)
    phi += M_PI;
  const Eigen::Vector2d d(std::cos(phi), std::sin(phi));
  const Eigen::Vector2d r(pose.x(), pose.y());
  if (m.s_wall_max > 0.0 && m.s_ground_max - m.s_wall_max >= p_.end_ground_gap)
  {
    const Eigen::Vector2d e = r + (m.s_wall_max + 0.5 * p_.slice_length) * d;
    // keep the farthest end seen (an occluded slice must not pull it back)
    if (!has_end_ || d.dot(e - end_) > 0.0)
      end_ = e;
    has_end_ = true;
  }
  else if (has_end_ && m.s_wall_max > 0.0 && d.dot(r + m.s_wall_max * d - end_) > 0.3)
  {
    has_end_ = false;   // walls continue beyond the presumed end (it was a gap)
  }
}

// ---------------------------------------------------------------------------
AisleReference AisleEstimator::query(const Eigen::Vector3d& pose, double t) const
{
  AisleReference ref;
  if (!initialized_)
    return ref;

  const Eigen::Vector3d x = predictState(pose);
  const double phi = wrapAngle(pose.z() + x(1));
  const Eigen::Vector2d n(-std::sin(phi), std::cos(phi));
  const Eigen::Vector2d qc = Eigen::Vector2d(pose.x(), pose.y()) + x(0) * n;

  ref.nx = n.x();
  ref.ny = n.y();
  ref.rho = n.dot(qc);
  ref.dx = std::cos(phi);
  ref.dy = std::sin(phi);
  ref.p0x = qc.x();
  ref.p0y = qc.y();
  ref.width = x(2);
  ref.e_y = x(0);
  ref.psi = x(1);
  ref.e_std = std::sqrt(std::max(0.0, P_(0, 0)));
  ref.horizon = p_.horizon;

  // Confidence: product of smooth factors, so that the centring fades in and
  // out instead of switching between AC-TEB and plain TEB (which made the
  // robot weave). The estimate is held (dead reckoning on the static line)
  // while the walls are not seen, up to hold_distance.
  auto ramp = [](double v, double v0, double v1) {   // 1 below v0, 0 above v1
    return v <= v0 ? 1.0 : (v >= v1 ? 0.0 : (v1 - v) / (v1 - v0));
  };
  const Eigen::Vector2d r(pose.x(), pose.y());
  ref.blind = (r - r_accept_).norm();
  const double c_blind = ramp(ref.blind, 0.5 * p_.hold_distance, p_.hold_distance);
  const double c_std = ramp(ref.e_std, p_.max_e_std, 2.0 * p_.max_e_std);
  const double c_head = ramp(std::abs(x(1)), p_.max_heading_error, p_.max_heading_error + 0.3);
  // robot outside the band between both rows: not in this aisle any more
  const double c_lat = std::abs(x(0)) <= 0.5 * x(2) ? 1.0 : 0.0;
  double c_end = 1.0;
  if (has_end_)
  {
    ref.to_end = Eigen::Vector2d(ref.dx, ref.dy).dot(end_ - r);
    c_end = 1.0 - ramp(ref.to_end, 0.0, p_.end_fade);   // 1 far from the end, 0 at the end
  }
  const bool width_ok = x(2) >= p_.min_width && x(2) <= p_.max_width;
  ref.conf = width_ok ? c_blind * c_std * c_head * c_lat * c_end : 0.0;
  ref.valid = ref.conf > 0.0;
  ref.weight = ref.conf * p_.weight / (1.0 + std::exp(p_.width_steepness * (x(2) - p_.width_ref)));
  return ref;
}

}  // namespace ac_teb_local_planner
