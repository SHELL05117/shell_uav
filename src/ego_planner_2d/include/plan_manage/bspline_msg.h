/*
 * @Function: Lightweight Bspline data struct (replaces the generated
 *            ego_planner/Bspline.msg header of the original repo)
 * @Ported from: JackJu-HIT/Ego-Planner-2D-ROS2 (MIT License)
 */
#ifndef _BSPLINE_MSG_H_
#define _BSPLINE_MSG_H_

#include <geometry_msgs/Point.h>
#include <vector>
#include <cstdint>

namespace ego_planner
{

struct Bspline
{
  Bspline()
    : order(0), traj_id(0), yaw_dt(0.0) {}

  int32_t order;
  int64_t traj_id;
  std::vector<double> knots;
  std::vector<geometry_msgs::Point> pos_pts;
  std::vector<double> yaw_pts;
  double yaw_dt;
};

} // namespace ego_planner

#endif // _BSPLINE_MSG_H_
