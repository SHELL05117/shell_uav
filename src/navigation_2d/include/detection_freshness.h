#ifndef NAVIGATION_2D_DETECTION_FRESHNESS_H
#define NAVIGATION_2D_DETECTION_FRESHNESS_H
#include <cmath>

namespace navigation_yolo
{
// Keep the upstream acquisition age as well as monotonic elapsed time. Fresh
// receipt or a newly completed slow inference cannot renew an old source frame.
inline bool freshDetectionSource(double source_stamp, double ros_now,
                                 double age_at_receipt, double elapsed,
                                 double timeout)
{
  return std::isfinite(source_stamp) && std::isfinite(ros_now) &&
      std::isfinite(age_at_receipt) && std::isfinite(elapsed) &&
      std::isfinite(timeout) && source_stamp > 0.0 && ros_now >= source_stamp &&
      age_at_receipt >= 0.0 && elapsed >= 0.0 && timeout > 0.0 &&
      ros_now - source_stamp <= timeout && age_at_receipt + elapsed <= timeout;
}
}  // namespace navigation_yolo
#endif
