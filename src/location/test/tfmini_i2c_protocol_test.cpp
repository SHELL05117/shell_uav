#include <gtest/gtest.h>
#include <tf_mini_i2c.h>
#include <vector>
#include <algorithm>

namespace {
std::vector<uint8> sent, reply;
bool fail_read = false, fail_write = false;
void frame(unsigned distance = 12, unsigned strength = 3705) {
  sent.clear(); fail_read = fail_write = false;
  reply = {0x59, 0x59, uint8(distance), uint8(distance >> 8),
           uint8(strength), uint8(strength >> 8), 0x20, 0x09, 0};
  for (unsigned i = 0; i < 8; ++i) reply[8] += reply[i];
}
}

// Replace only the transport; no Linux device is opened by this test target.
I2C::I2C(uint8, uint8) {}
I2C::~I2C() {}
void I2C::change_address(uint8) {}
void I2C::write_bytes(const uint8 *data, std::size_t length) {
  if (fail_write) throw std::system_error(EIO, std::generic_category());
  sent.assign(data, data + length);
}
void I2C::read_bytes(uint8 *data, std::size_t length) {
  if (fail_read || reply.size() != length)
    throw std::system_error(EIO, std::generic_category());
  std::copy(reply.begin(), reply.end(), data);
}

TEST(TFMiniI2C, ExactRawCommandAndMetres) {
  frame(); TFMiniPro device;
  EXPECT_FLOAT_EQ(.12f, device.ReadData());
  EXPECT_EQ((std::vector<uint8>{0x5a, 0x05, 0x00, 0x01, 0x60}), sent);
}
TEST(TFMiniI2C, BadChecksum) {
  frame(); reply[8] ^= 1; TFMiniPro device; EXPECT_LT(device.ReadData(), 0);
}
TEST(TFMiniI2C, BadHeaderWithValidChecksum) {
  frame(); reply[0] = 0x5a; reply[8] += 1;
  TFMiniPro device; EXPECT_LT(device.ReadData(), 0);
}
TEST(TFMiniI2C, ShortFrame) {
  frame(); reply.pop_back(); TFMiniPro device; EXPECT_LT(device.ReadData(), 0);
}
TEST(TFMiniI2C, ReadFailureDoesNotReuseLastHeight) {
  frame(); TFMiniPro device; EXPECT_FLOAT_EQ(.12f, device.ReadData());
  fail_read = true; EXPECT_LT(device.ReadData(), 0);
}
TEST(TFMiniI2C, WriteFailure) {
  frame(); fail_write = true; TFMiniPro device; EXPECT_LT(device.ReadData(), 0);
}
TEST(TFMiniI2C, InvalidDistance) {
  TFMiniPro device;
  for (unsigned value : {0u, 9u, 1201u, 65535u}) {
    frame(value); EXPECT_LT(device.ReadData(), 0);
  }
}
TEST(TFMiniI2C, InvalidSignalStrength) {
  TFMiniPro device;
  for (unsigned value : {0u, 99u, 65535u}) {
    frame(12, value); EXPECT_LT(device.ReadData(), 0);
  }
}
TEST(TFMiniI2C, ValidDistanceAndSignalBoundaries) {
  TFMiniPro device;
  frame(10, 100); EXPECT_FLOAT_EQ(.10f, device.ReadData());
  frame(1200, 65534); EXPECT_FLOAT_EQ(12.f, device.ReadData());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
