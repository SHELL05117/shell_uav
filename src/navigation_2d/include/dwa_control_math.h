#ifndef NAVIGATION_2D_DWA_CONTROL_MATH_H
#define NAVIGATION_2D_DWA_CONTROL_MATH_H

#include <cmath>

// ROS-independent coordinate and validity rules. Values remain in ROS ENU;
// MAVROS performs the final ENU/FLU to PX4 NED/FRD conversion.
namespace dwa_control
{
struct Vec2
{
  double x, y;
  Vec2(double x_value = 0.0, double y_value = 0.0) : x(x_value), y(y_value) {}
};

struct Pose2
{
  double x, y, yaw;
  Pose2(double x_value = 0.0, double y_value = 0.0, double yaw_value = 0.0)
      : x(x_value), y(y_value), yaw(yaw_value) {}
};

inline bool finite(const Vec2 &value)
{
  return std::isfinite(value.x) && std::isfinite(value.y);
}

inline bool finite(const Pose2 &value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
}

inline Vec2 rotate(const Vec2 &value, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return Vec2(c * value.x - s * value.y, s * value.x + c * value.y);
}

inline bool limitPlanar(const Vec2 &input, double maximum, Vec2 &output)
{
  output = Vec2();
  if (!finite(input) || !std::isfinite(maximum) || maximum < 0.0)
    return false;
  const double speed = std::hypot(input.x, input.y);
  if (!std::isfinite(speed))
    return false;
  const double scale = speed > maximum && speed > 0.0 ? maximum / speed : 1.0;
  output = Vec2(input.x * scale, input.y * scale);
  return true;
}

inline bool bodyVelocityToLocal(const Vec2 &body, double local_yaw,
                                double maximum, Vec2 &local)
{
  local = Vec2();
  Vec2 bounded;
  if (!std::isfinite(local_yaw) || !limitPlanar(body, maximum, bounded))
    return false;
  local = rotate(bounded, local_yaw);
  return true;
}

// Two observations of the same body define local <- map. This accommodates
// nonzero yaw, different origins, and map corrections; callers must first
// establish freshness/time alignment for both observations.
inline bool mapPointToLocal(const Vec2 &point, const Pose2 &map_body,
                            const Pose2 &local_body, Vec2 &local)
{
  local = Vec2();
  if (!finite(point) || !finite(map_body) || !finite(local_body))
    return false;
  const Vec2 delta = rotate(Vec2(point.x - map_body.x, point.y - map_body.y),
                            local_body.yaw - map_body.yaw);
  local = Vec2(local_body.x + delta.x, local_body.y + delta.y);
  return finite(local);
}

inline bool yawOf(double x, double y, double z, double w, double &yaw)
{
  yaw = 0.0;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w))
    return false;
  const double norm = std::sqrt(x*x + y*y + z*z + w*w);
  if (!std::isfinite(norm) || norm < 1e-8)
    return false;
  x /= norm; y /= norm; z /= norm; w /= norm;
  yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
  return true;
}

inline bool fresh(double now, double stamp, double maximum_age,
                  double future_tolerance = 0.05)
{
  return std::isfinite(now) && std::isfinite(stamp) &&
         std::isfinite(maximum_age) && std::isfinite(future_tolerance) &&
         stamp > 0.0 && maximum_age > 0.0 && future_tolerance >= 0.0 &&
         now - stamp >= -future_tolerance && now - stamp <= maximum_age;
}
}  // namespace dwa_control

#endif
