#ifndef USB_CAM_ROCKCHIP_MJPEG_H
#define USB_CAM_ROCKCHIP_MJPEG_H
#include <cstddef>
namespace usb_cam {
// Optional RK backend: MPP decodes MJPEG, RGA converts NV12 to tightly packed RGB.
class RockchipMjpeg {
 public:
  RockchipMjpeg(int width, int height);
  ~RockchipMjpeg();
  bool decode(const void *jpeg, std::size_t length, char *rgb);
 private:
  RockchipMjpeg(const RockchipMjpeg&);
  RockchipMjpeg& operator=(const RockchipMjpeg&);
  struct State;
  State *state_;
};
}
#endif
