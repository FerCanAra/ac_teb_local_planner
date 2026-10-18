/*********************************************************************
 * AC-TEB: aisle-heading edge.
 *
 * Penalises the angle between the heading theta_i of a pose of the band
 * and the estimated aisle direction phi_c (oriented with the direction of
 * travel). Together with EdgeAisleCentering (lateral offset) it acts as a
 * proportional-derivative pair: without it the band reaches the centreline
 * at an angle and overshoots, which shows up as S-shaped motion.
 *********************************************************************/
#ifndef AC_TEB_EDGE_AISLE_HEADING_H_
#define AC_TEB_EDGE_AISLE_HEADING_H_

#include "ac_teb_local_planner/g2o_types/vertex_pose.h"
#include "ac_teb_local_planner/g2o_types/base_teb_edges.h"
#include "ac_teb_local_planner/misc.h"

#include "g2o/core/base_unary_edge.h"
#include "g2o/stuff/misc.h"

namespace ac_teb_local_planner
{

class EdgeAisleHeading : public BaseTebUnaryEdge<1, double, VertexPose>
{
public:
  EdgeAisleHeading()
  {
    _measurement = 0.0;   // phi_c [rad]
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setParameters() on EdgeAisleHeading()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    _error[0] = g2o::normalize_theta(bandpt->theta() - _measurement);
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAisleHeading::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param phi aisle direction in the planning frame, oriented with the travel direction [rad]
   */
  void setParameters(const TebConfig& cfg, double phi)
  {
    cfg_ = &cfg;
    _measurement = phi;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ac_teb_local_planner

#endif  // AC_TEB_EDGE_AISLE_HEADING_H_
