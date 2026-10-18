/*********************************************************************
 * AC-TEB: online aisle (corridor) estimator.
 *
 * ROS-independent core (Eigen + STL only).
 *
 *  1. Wall points (obstacle label) and ground points from the ground
 *     segmentation, expressed in the robot base frame.
 *  2. Points are rotated into the predicted aisle frame (s along the aisle,
 *     l lateral, robot at the origin) and binned in slices along s.
 *     Per slice, the inner edge of the left/right wall is a low quantile of
 *     |l| of the obstacle points on each side.
 *  3. Both edges are fitted jointly with a common slope (parallel lines),
 *     l = m s + b_L,  l = m s + b_R, by Huber-IRLS.
 *  4. Measurement z = [e_y, psi, W]:
 *        e_y  lateral offset of the centreline w.r.t. the robot (left > 0)
 *        psi  aisle direction w.r.t. the robot heading
 *        W    free width between the walls
 *     with covariance propagated from the fit. If only one wall is visible
 *     and the width is already known, the centreline is placed W/2 from the
 *     visible wall (W is then not updated). In slices without enough wall
 *     points, the lateral end of the ground replaces the wall edge (lower
 *     weight), but only where some obstacle points confirm a ground/wall
 *     junction (not where the ground ends at the image border).
 *  5. Kalman filter in robot coordinates; between updates the state is
 *     propagated exactly with the robot motion (the aisle is static in the
 *     odometry frame). Chi-square gating + re-initialisation (new aisle).
 *********************************************************************/
#ifndef AC_TEB_AISLE_ESTIMATOR_H_
#define AC_TEB_AISLE_ESTIMATOR_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace ac_teb_local_planner
{

struct AisleEstimatorParams
{
  // --- slicing (robot frame) ---
  double s_min = 0.3;              //!< start of the observed aisle section [m]
  double s_max = 2.0;              //!< end of the observed aisle section [m]
  double slice_length = 0.1;       //!< slice size along the aisle [m]
  double lateral_max = 1.5;        //!< max lateral distance of wall points [m]
  double edge_quantile = 0.1;      //!< quantile of |l| used as inner wall edge
  int min_slices = 4;              //!< min slices with an edge on each side
  int min_points_per_slice = 5;    //!< min wall points in a slice to define an edge (rejects sparse ghosts)
  double ground_edge_weight = 0.3; //!< weight of ground/wall-junction edges (0 disables them)
  double junction_max_gap = 0.5;   //!< obstacle points up to this distance beyond the ground end confirm a junction [m]
  int junction_min_points = 1;     //!< obstacle points needed to confirm a junction (0: accept every ground end)
  double huber_delta = 0.05;       //!< Huber threshold [m]
  // --- filter ---
  double proc_noise_e = 0.05;      //!< process noise std of e_y [m/sqrt(s)]
  double proc_noise_psi = 0.05;    //!< process noise std of psi [rad/sqrt(s)]
  double proc_noise_w = 0.02;      //!< process noise std of W [m/sqrt(s)]
  double meas_floor_e = 0.02;      //!< min measurement std of e_y [m]
  double meas_floor_psi = 0.02;    //!< min measurement std of psi [rad]
  double meas_floor_w = 0.03;      //!< min measurement std of W [m]
  double gate_chi2 = 16.0;         //!< innovation gate (3 dof)
  int reinit_after = 5;            //!< consecutive gated measurements -> reset
  // --- validity ---
  double timeout = 1.0;            //!< max age of the last accepted update [s]
  double max_e_std = 0.15;         //!< max std of e_y for a valid estimate [m]
  double min_width = 0.4;          //!< [m]
  double max_width = 3.0;          //!< [m]
  double max_heading_error = 0.6;  //!< |psi| above which the centering starts to fade out [rad]
  // --- persistence (the aisle is static in the planning frame) ---
  double hold_distance = 3.0;      //!< distance travelled without accepted measurements before the estimate is dropped [m]
  double end_fade = 1.0;           //!< fade-out length before the detected end of the row [m]
  double end_ground_gap = 1.0;     //!< ground seen this far beyond the last wall slice -> end of the row ahead [m]
  // --- centering weight gamma_c(W) ---
  double weight = 50.0;            //!< gamma_c^max
  double width_ref = 1.5;          //!< W_bar [m]
  double width_steepness = 10.0;   //!< kappa_W [1/m]
  double horizon = 2.0;            //!< L_c: centering applied up to this distance ahead [m]
};

struct AisleMeasurement
{
  bool ok = false;
  Eigen::Vector3d z = Eigen::Vector3d::Zero();     //!< [e_y, psi, W]
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  int n_left = 0, n_right = 0;                      //!< slices with an edge (wall or ground junction)
  int n_wall_left = 0, n_wall_right = 0;            //!< ... of which real wall slices
  int single_side = 0;                              //!< 0 both walls, 1 left only, 2 right only
  double m = 0, b_left = 0, b_right = 0;           //!< fit in the prior aisle frame
  double prior_psi = 0;                             //!< aisle-frame rotation used
  double s_wall_max = -1;                           //!< farthest slice with a wall edge (either side) [m]
  double s_ground_max = -1;                         //!< farthest slice with ground points [m]
};

//! Aisle reference handed to the optimizer (planning frame)
struct AisleReference
{
  bool valid = false;
  double nx = 0, ny = 1, rho = 0;   //!< centreline: nx*x + ny*y = rho
  double dx = 1, dy = 0;            //!< aisle direction
  double p0x = 0, p0y = 0;          //!< projection of the robot on the centreline
  double width = 0;                 //!< W [m]
  double weight = 0;                //!< gamma_c(W) (0 if invalid)
  double horizon = 0;               //!< L_c [m]
  double e_y = 0, psi = 0;          //!< robot-frame state (for logging)
  double e_std = 0;
  double conf = 0;                  //!< confidence in [0,1] that scales gamma_c (fades, no on/off switching)
  double blind = 0;                 //!< distance travelled since the last accepted measurement [m]
  double to_end = 1e9;              //!< distance to the detected end of the row [m]
};

class AisleEstimator
{
public:
  explicit AisleEstimator(const AisleEstimatorParams& p = AisleEstimatorParams()) : p_(p) {}

  void setParams(const AisleEstimatorParams& p) { p_ = p; }
  const AisleEstimatorParams& params() const { return p_; }
  void reset() { initialized_ = false; rejects_ = 0; has_end_ = false; }
  bool initialized() const { return initialized_; }

  //! Direction of the global plan in the planning frame [rad]. Used only to
  //! orient the slicing before the first estimate (instead of the robot heading).
  void setHeadingHint(double phi) { hint_phi_ = phi; has_hint_ = true; }
  void clearHeadingHint() { has_hint_ = false; }

  /**
   * @brief Measurement + filter update.
   * @param obstacles wall points in the robot base frame (x fwd, y left, z up)
   * @param ground    ground points in the robot base frame (may be empty)
   * @param robot_pose robot pose (x, y, yaw) in the planning frame at the cloud stamp
   * @param t time stamp [s]
   * @return true if the measurement was accepted
   */
  bool update(const std::vector<Eigen::Vector3d>& obstacles,
              const std::vector<Eigen::Vector3d>& ground,
              const Eigen::Vector3d& robot_pose, double t);

  //! Aisle reference for the current robot pose (planning frame) at time t.
  AisleReference query(const Eigen::Vector3d& robot_pose, double t) const;

  //! Last raw measurement (debug / visualisation).
  const AisleMeasurement& lastMeasurement() const { return last_meas_; }

  //! Measurement only (no filtering), public for testing.
  AisleMeasurement measure(const std::vector<Eigen::Vector3d>& obstacles,
                           const std::vector<Eigen::Vector3d>& ground,
                           double prior_psi, double width_prior = -1.0,
                           double width_prior_std = 0.0) const;

private:
  //! State [e_y, psi, W] seen from robot_pose, from the stored planning-frame line.
  Eigen::Vector3d predictState(const Eigen::Vector3d& robot_pose) const;
  void storeLine(const Eigen::Vector3d& x, const Eigen::Vector3d& robot_pose);

  AisleEstimatorParams p_;
  bool initialized_ = false;
  int rejects_ = 0;
  // centreline stored in the planning frame: point q, direction phi, width W
  Eigen::Vector2d q_ = Eigen::Vector2d::Zero();
  double phi_ = 0.0;
  double width_ = 0.0;
  double hint_phi_ = 0.0;
  bool has_hint_ = false;
  Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
  double t_last_ = 0.0;       //!< time of the last filter step
  double t_accept_ = -1e9;    //!< time of the last accepted measurement
  Eigen::Vector2d r_accept_ = Eigen::Vector2d::Zero();   //!< robot position at the last accepted measurement
  bool has_end_ = false;      //!< end of the row detected ahead
  Eigen::Vector2d end_ = Eigen::Vector2d::Zero();         //!< end of the row (planning frame)
  //! Accept a filtered state and remember where it was measured
  void accept(const Eigen::Vector3d& x, const Eigen::Vector3d& pose, double t);
  //! Detect the end of the row from the last measurement
  void updateRowEnd(const Eigen::Vector3d& pose);
  AisleMeasurement last_meas_;
};

}  // namespace ac_teb_local_planner

#endif  // AC_TEB_AISLE_ESTIMATOR_H_
