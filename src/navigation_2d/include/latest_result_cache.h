#ifndef NAVIGATION_2D_LATEST_RESULT_CACHE_H
#define NAVIGATION_2D_LATEST_RESULT_CACHE_H
#include <cstdint>
#include <mutex>
#include <utility>

namespace navigation_yolo
{
template <typename Result>
class LatestResultCache
{
public:
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = false;
  }
  void publish(Result result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = std::move(result);
    ++sequence_;
    valid_ = true;
  }
  bool poll(Result &result, uint64_t &sequence) const
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !valid_ || sequence == sequence_)
      return false;
    result = result_;
    sequence = sequence_;
    return true;
  }
private:
  mutable std::mutex mutex_;
  Result result_;
  uint64_t sequence_ = 0;
  bool valid_ = false;
};
}  // namespace navigation_yolo
#endif
