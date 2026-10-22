// Cross-check for tools/servo_angle_calib.py.
//
// The Python tool claims its angle -> pulse -> tick arithmetic is identical to
// ServoController::pulseUsFor / ticksFor.  A claim like that is worth exactly
// nothing until it is executed, so this file is the same arithmetic copied
// verbatim out of src/servo_control/src/servo_controller_node.cpp and
// gcc-compiled, and the two outputs are diffed:
//
//   g++ -O2 -o /tmp/servo_tick_check tools/servo_tick_check.cpp
//   /tmp/servo_tick_check 0 1 30 55 89.5 90 90.5 140 170 180 200
//   python3 tools/servo_angle_calib.py --dry-run --angles 0 1 30 55 89.5 90 90.5 140 170 180 200
//
// 200 is in the list on purpose: pulseUsFor clamps to 180 first, and the tool
// has to clamp the same way or a mistyped parameter would be measured wrong.
//
// Build-time note: this is a plain C++ file with no ROS dependency, so it also
// compiles on the Windows box (w64devkit) where the real package cannot.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace
{
// Verbatim from servo_controller_node.cpp::pulseUsFor.  The clamp lives here,
// before the pulse width is computed, so an angle of 200 becomes the 180
// position and not a 2417 us pulse - the tool has to clamp identically or a
// mistyped parameter would be measured at a pulse the node never produces.
double pulseUsFor(double angle, int min_pulse_us, int max_pulse_us)
{
  angle = std::max(0.0, std::min(180.0, angle));
  return min_pulse_us + (max_pulse_us - min_pulse_us) * angle / 180.0;
}

// Verbatim from servo_controller_node.cpp::ticksFor.
uint16_t ticksFor(double angle, int min_pulse_us, int max_pulse_us,
                  int pwm_frequency)
{
  return static_cast<uint16_t>(
      std::lround(pulseUsFor(angle, min_pulse_us, max_pulse_us) *
                  pwm_frequency * 4096.0 / 1000000.0));
}

// ticksFor without the rounding, so a near-tie between lround and Python's
// banker's rounding is visible as a decimal instead of as an off-by-one.
double rawTicksFor(double angle, int min_pulse_us, int max_pulse_us,
                   int pwm_frequency)
{
  return pulseUsFor(angle, min_pulse_us, max_pulse_us) * pwm_frequency *
      4096.0 / 1000000.0;
}

// Verbatim from servo_controller_node.cpp::initializePwm.
int prescaleFor(int pwm_frequency)
{
  const double prescale_value = 25000000.0 / (4096.0 * pwm_frequency) - 1.0;
  return static_cast<int>(std::lround(prescale_value));
}
} // namespace

int main(int argc, char **argv)
{
  const int min_pulse_us = 750;
  const int max_pulse_us = 2250;
  const int pwm_frequency = 50;

  std::printf("prescale=%d (freq=%d)\n", prescaleFor(pwm_frequency),
              pwm_frequency);
  std::printf("%8s %10s %6s %10s\n", "angle", "pulse_us", "ticks", "raw");

  if (argc < 2)
  {
    std::fprintf(stderr, "usage: %s ANGLE [ANGLE ...]\n", argv[0]);
    return 2;
  }
  for (int i = 1; i < argc; ++i)
  {
    const double angle = std::atof(argv[i]);
    std::printf("%8.1f %10.1f %6u %10.4f\n", angle,
                pulseUsFor(angle, min_pulse_us, max_pulse_us),
                ticksFor(angle, min_pulse_us, max_pulse_us, pwm_frequency),
                rawTicksFor(angle, min_pulse_us, max_pulse_us, pwm_frequency));
  }
  return 0;
}
