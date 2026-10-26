#ifndef LOCATION_LOCALIZATION_GUARD_H
#define LOCATION_LOCALIZATION_GUARD_H

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace location_guard
{
inline bool fresh(double stamp, double now, double timeout)
{
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0.0 &&
         timeout > 0.0 && now - stamp >= -0.02 && now - stamp <= timeout;
}

// Counts distinct measurements, never repetitions of one cached ROS message.
class RangeGate
{
public:
  double minimum = 0.05, maximum = 6.0, max_delta = 0.60, confirmation_tolerance = 0.10;
  int confirmations_required = 20;
  bool observe(double value, double stamp)
  {
    if (!std::isfinite(stamp) || stamp <= seen_stamp_) return false;
    seen_stamp_ = stamp;
    if (!std::isfinite(value) || value < minimum || value > maximum)
    { confirmations_ = 0; return false; }
    if (valid_ && std::fabs(value - value_) > max_delta)
    {
      if (confirmations_ && std::fabs(value - candidate_) <= confirmation_tolerance)
        ++confirmations_;
      else confirmations_ = 1;
      candidate_ = value;
      if (confirmations_ < std::max(1, confirmations_required)) return false;
    }
    value_ = value; stamp_ = stamp; valid_ = true; confirmations_ = 0;
    return true;
  }
  bool valid() const { return valid_; }
  double value() const { return value_; }
  double stamp() const { return stamp_; }
private:
  double seen_stamp_ = 0.0, stamp_ = 0.0, value_ = 0.0, candidate_ = 0.0;
  int confirmations_ = 0;
  bool valid_ = false;
};

class PositionGate
{
public:
  double max_delta = 0.30, confirmation_tolerance = 0.10;
  int confirmations_required = 10;
  bool observe(double x, double y, double stamp)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(stamp)) return false;
    if (stamp == stamp_ && valid_) return x == x_ && y == y_;
    if (stamp <= seen_stamp_) return false;
    seen_stamp_ = stamp;
    if (valid_ && std::hypot(x-x_, y-y_) > max_delta)
    {
      if (confirmations_ && std::hypot(x-candidate_x_, y-candidate_y_) <= confirmation_tolerance)
        ++confirmations_;
      else confirmations_ = 1;
      candidate_x_ = x; candidate_y_ = y;
      if (confirmations_ < std::max(1, confirmations_required)) return false;
    }
    x_ = x; y_ = y; stamp_ = stamp; valid_ = true; confirmations_ = 0;
    return true;
  }
private:
  double seen_stamp_ = 0.0, stamp_ = 0.0, x_ = 0.0, y_ = 0.0;
  double candidate_x_ = 0.0, candidate_y_ = 0.0;
  int confirmations_ = 0;
  bool valid_ = false;
};

struct Vector { double x, y, z; };
struct Quaternion { double x, y, z, w; };
inline double norm(const Vector &v) { return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); }
inline bool rotate(const Quaternion &input, const Vector &v, Vector &out)
{
  const double n = std::sqrt(input.x*input.x+input.y*input.y+input.z*input.z+input.w*input.w);
  if (!std::isfinite(n) || n < 0.8 || n > 1.2 || !std::isfinite(norm(v))) return false;
  const double x=input.x/n, y=input.y/n, z=input.z/n, w=input.w/n;
  const Vector t{2.0*(y*v.z-z*v.y), 2.0*(z*v.x-x*v.z), 2.0*(x*v.y-y*v.x)};
  out = Vector{v.x+w*t.x+y*t.z-z*t.y,
               v.y+w*t.y+z*t.x-x*t.z,
               v.z+w*t.z+x*t.y-y*t.x};
  return std::isfinite(norm(out));
}

// ground_clearance is the optical centre's VERTICAL clearance at the level
// takeoff reference. Offsets are body FLU; beam points towards the floor.
inline bool projectHeight(double range, const Quaternion &attitude, const Vector &beam,
                          const Vector &offset, double ground_clearance, double max_tilt_deg,
                          double minimum_z, double maximum_z, double &height)
{
  const double beam_norm = norm(beam);
  if (!std::isfinite(range) || range <= 0.0 || !std::isfinite(beam_norm) || beam_norm < 1e-6 ||
      !std::isfinite(ground_clearance) || ground_clearance < 0.0 ||
      !std::isfinite(max_tilt_deg) || max_tilt_deg <= 0.0 || max_tilt_deg >= 80.0 ||
      !std::isfinite(minimum_z) || !std::isfinite(maximum_z) || minimum_z >= maximum_z) return false;
  Vector world_beam{}, world_offset{};
  if (!rotate(attitude, Vector{beam.x/beam_norm, beam.y/beam_norm, beam.z/beam_norm}, world_beam) ||
      !rotate(attitude, offset, world_offset)) return false;
  const double down = -world_beam.z;
  if (down < std::cos(max_tilt_deg * 3.14159265358979323846 / 180.0)) return false;
  height = range*down - ground_clearance + offset.z - world_offset.z;
  return std::isfinite(height) && height >= minimum_z && height <= maximum_z;
}

inline bool validSerialFrame(const unsigned char *data, std::size_t size)
{
  if (size != 9 || data[0] != 0x59 || data[1] != 0x59) return false;
  unsigned sum = 0;
  for (std::size_t i=0; i<8; ++i) sum += data[i];
  return static_cast<unsigned char>(sum) == data[8];
}
}  // namespace location_guard
#endif
