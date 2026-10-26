#ifndef NAVIGATION_2D_YOLO_TENSOR_LAYOUT_H
#define NAVIGATION_2D_YOLO_TENSOR_LAYOUT_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <iterator>
#include <string>
#include <vector>

// No ROS, OpenCV or RKNN dependency: the same layout conversion is exercised
// by the detector and by host-side regression tests.
namespace navigation_yolo
{
struct Head
{
  int channels = 0;
  int height = 0;
  int width = 0;
  std::vector<float> data;
};

inline std::vector<std::size_t> reversedDimensions(const uint32_t *dims, std::size_t count)
{
  if (!dims || (count != 4 && count != 5)) return {};
  return std::vector<std::size_t>(std::reverse_iterator<const uint32_t *>(dims + count),
                                  std::reverse_iterator<const uint32_t *>(dims));
}

inline bool describeHead(const std::vector<std::size_t> &shape, int channels,
                         std::size_t elements, Head &head, std::string &error)
{
  std::size_t product = 1;
  for (std::size_t dimension : shape)
  {
    if (dimension == 0 || dimension > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        product > std::numeric_limits<std::size_t>::max() / dimension)
    {
      error = "invalid/overflowing output dimension";
      return false;
    }
    product *= dimension;
  }
  if (product != elements || channels < 1 || shape.empty() || shape[0] != 1)
  {
    error = "output batch/element count does not match its declared shape";
    return false;
  }
  if (shape.size() == 4 && shape[1] == static_cast<std::size_t>(channels))
  {
    head.channels = channels;
    head.height = static_cast<int>(shape[2]);
    head.width = static_cast<int>(shape[3]);
    return true;
  }
  if (shape.size() == 5 && shape[1] == 3 && shape[4] * 3 == static_cast<std::size_t>(channels))
  {
    head.channels = channels;
    head.height = static_cast<int>(shape[2]);
    head.width = static_cast<int>(shape[3]);
    return true;
  }
  error = "expected [1,C,H,W] or [1,3,H,W,K] raw head";
  return false;
}

inline bool normalizeHead(const std::vector<std::size_t> &shape, int channels,
                          const float *source, std::size_t elements,
                          Head &head, std::string &error)
{
  if (!source || !describeHead(shape, channels, elements, head, error))
  {
    if (!source) error = "null output buffer";
    return false;
  }
  if (!std::all_of(source, source + elements, [](float value) { return std::isfinite(value); }))
  {
    error = "non-finite raw output";
    return false;
  }
  if (shape.size() == 4)
  {
    head.data.assign(source, source + elements);
    return true;
  }
  // [N,A,H,W,K] -> [N,A,K,H,W] -> [N,A*K,H,W]. Merely reshaping
  // would mix cell coordinates, objectness and class logits.
  const std::size_t plane = static_cast<std::size_t>(head.height) * head.width;
  const std::size_t fields = shape[4];
  head.data.resize(elements);
  for (std::size_t anchor = 0; anchor < 3; ++anchor)
    for (std::size_t pixel = 0; pixel < plane; ++pixel)
      for (std::size_t field = 0; field < fields; ++field)
        head.data[(anchor * fields + field) * plane + pixel] =
            source[(anchor * plane + pixel) * fields + field];
  return true;
}

struct Box
{
  float center_x, center_y, width, height;
};

inline float sigmoid(float value)
{
  if (value >= 0.0f) return 1.0f / (1.0f + std::exp(-value));
  const float exp_value = std::exp(value);
  return exp_value / (1.0f + exp_value);
}

inline Box decodeV5Box(float x, float y, float w, float h, int grid_x, int grid_y,
                       int stride, float anchor_width, float anchor_height)
{
  const float sw = 2.0f * sigmoid(w), sh = 2.0f * sigmoid(h);
  return Box{(2.0f * sigmoid(x) - 0.5f + grid_x) * stride,
             (2.0f * sigmoid(y) - 0.5f + grid_y) * stride,
             sw * sw * anchor_width, sh * sh * anchor_height};
}
}  // namespace navigation_yolo
#endif
