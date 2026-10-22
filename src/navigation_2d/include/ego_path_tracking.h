#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

namespace ego_path_tracking
{
// Selects the point on `path` that is `lookahead_distance` metres of arc length
// ahead of the vehicle's closest projection onto the path (clamped to the end of
// the path), so the returned target moves continuously with the vehicle instead
// of jumping from vertex to vertex.
//
// `remaining_distance` is the straight-line distance to path.poses.back() - the
// end of the LOCAL trajectory, which is NOT the route goal.
//
// `cross_track_distance`, when non-null, receives the perpendicular distance from
// the vehicle to the path (NaN when the inputs are rejected). It is the quantity
// that decides how much a given lookahead steers: tan(heading error) =
// cross_track / lookahead, so a 0.05 m lag is 14 deg of steer at a 0.20 m
// lookahead and 6 deg at 0.45 m. A path that is followed with an oscillating
// cross-track error and a short lookahead zig-zags; that is the mechanism behind
// the reported left-right sway.
inline bool selectLookaheadPoint(const nav_msgs::Path &path,
                                 double x,
                                 double y,
                                 double lookahead_distance,
                                 double goal_tolerance,
                                 geometry_msgs::Point &target,
                                 double &remaining_distance,
                                 double *cross_track_distance = nullptr)
{
  if (cross_track_distance != nullptr)
  {
    *cross_track_distance = std::numeric_limits<double>::quiet_NaN();
  }
  if (path.poses.empty())
  {
    return false;
  }

  const auto &goal = path.poses.back().pose.position;
  remaining_distance = std::hypot(goal.x - x, goal.y - y);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(remaining_distance) ||
      !std::isfinite(lookahead_distance) || !std::isfinite(goal_tolerance) ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      remaining_distance <= goal_tolerance)
  {
    return false;
  }

  const double lookahead = std::max(0.0, lookahead_distance);
  if (path.poses.size() == 1)
  {
    target = goal;
    if (cross_track_distance != nullptr)
    {
      *cross_track_distance = remaining_distance;
    }
    return true;
  }

  double closest_distance_sq = std::numeric_limits<double>::infinity();
  double closest_arc_length = 0.0;
  double total_length = 0.0;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i)
  {
    const auto &start = path.poses[i].pose.position;
    const auto &end = path.poses[i + 1].pose.position;
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(end.x) || !std::isfinite(end.y))
    {
      return false;
    }
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_length_sq = segment_x * segment_x + segment_y * segment_y;
    const double segment_length = std::sqrt(segment_length_sq);
    const double projection = segment_length_sq > 1e-12
                                  ? std::max(0.0, std::min(1.0,
                                        ((x - start.x) * segment_x +
                                         (y - start.y) * segment_y) /
                                            segment_length_sq))
                                  : 0.0;
    const double closest_x = start.x + projection * segment_x;
    const double closest_y = start.y + projection * segment_y;
    const double dx = x - closest_x;
    const double dy = y - closest_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < closest_distance_sq)
    {
      closest_distance_sq = distance_sq;
      closest_arc_length = total_length + projection * segment_length;
    }
    total_length += segment_length;
  }

  if (cross_track_distance != nullptr)
  {
    *cross_track_distance = std::sqrt(closest_distance_sq);
  }

  const double target_arc_length = std::min(total_length,
                                             closest_arc_length + lookahead);
  double traversed = 0.0;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i)
  {
    const auto &start = path.poses[i].pose.position;
    const auto &end = path.poses[i + 1].pose.position;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length <= 1e-12)
    {
      continue;
    }

    if (target_arc_length <= traversed + segment_length)
    {
      const double ratio = std::max(0.0, std::min(1.0,
          (target_arc_length - traversed) / segment_length));
      target.x = start.x + ratio * segment_x;
      target.y = start.y + ratio * segment_y;
      target.z = start.z + ratio * (end.z - start.z);
      return true;
    }
    traversed += segment_length;
  }

  target = goal;
  return true;
}
}  // namespace ego_path_tracking
