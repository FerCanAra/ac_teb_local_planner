/*********************************************************************
 * AC-TEB: aisle-centering edge.
 *
 * Unary edge on a pose vertex s_i = [x_i, y_i, beta_i] that penalises the
 * signed lateral distance of the pose to the estimated aisle centreline
 *
 *     C = { p in R^2 | n^T p = rho },   ||n|| = 1
 *
 *     e_c,i = n^T p_i - rho          cost = gamma_c(W) * e_c,i^2
 *
 * gamma_c(W) is set as the information matrix of the edge (see
 * TebOptimalPlanner::AddEdgesAisleCentering). Jacobian w.r.t. s_i is
 * [n_x, n_y, 0]; it is computed numerically, as for the other TEB edges.
 *********************************************************************/
#ifndef AC_TEB_EDGE_AISLE_CENTERING_H_
#define AC_TEB_EDGE_AISLE_CENTERING_H_

#include "ac_teb_local_planner/g2o_types/vertex_pose.h"
#include "ac_teb_local_planner/g2o_types/base_teb_edges.h"
#include "ac_teb_local_planner/misc.h"

#include "g2o/core/base_unary_edge.h"

namespace ac_teb_local_planner
{

/**
 * @class EdgeAisleCentering
 * @brief Pulls a pose of the band towards the aisle centreline.
 * @remarks Call setParameters() before adding the edge to the optimizer.
 */
class EdgeAisleCentering : public BaseTebUnaryEdge<1, Eigen::Vector3d, VertexPose>
{
public:
  EdgeAisleCentering()
  {
    _measurement.setZero();   // [n_x, n_y, rho]
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setParameters() on EdgeAisleCentering()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    _error[0] = _measurement[0] * bandpt->x() + _measurement[1] * bandpt->y() - _measurement[2];
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAisleCentering::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param nx, ny unit normal of the centreline (planning frame)
   * @param rho centreline offset: nx*x + ny*y = rho
   */
  void setParameters(const TebConfig& cfg, double nx, double ny, double rho)
  {
    cfg_ = &cfg;
    _measurement << nx, ny, rho;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ac_teb_local_planner

#endif  // AC_TEB_EDGE_AISLE_CENTERING_H_
