#include <gtest/gtest.h>
#include <usb_cam/usb_cam.h>

TEST(UsbCamLifecycle, ShutdownWithoutStartingIsRepeatable)
{
  usb_cam::UsbCam camera;
  camera.shutdown();
  camera.shutdown();
  // The destructor calls shutdown again after this scope.
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
