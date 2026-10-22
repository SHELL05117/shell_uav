#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bench tool: drive one PCA9685 servo channel to an arbitrary angle.

WHY THIS EXISTS
---------------
servo_controller_node accepts only four fixed command strings on /gpio_cmd
(CAMERA_FRONT, CAMERA_DOWN, DROP_FIRE, DROP_RESCUE) and it reads its four
angles from parameters at construction time.  That is what keeps a stray
message from moving the drop servo during a run - but it leaves the two bench
jobs with nowhere to go:

  1. find which angle actually releases the fire container and which one
     releases the rescue container, and
  2. bracket those two so they are far enough apart that the mechanism
     cannot confuse them (the node itself warns below 5 deg).

There are two ways to do those two jobs, and they are NOT interchangeable.

  * Prefer /servo_debug_cmd, driven by tools/servo_debug.py.  Under
    src/servo_control/launch/servo_bench.launch the node subscribes that topic
    itself and accepts "SET_DROP <deg>", "SET_CAMERA <deg>", "PARK", "STATUS".
    The servo keeps being driven by the same process that will fly the
    aircraft, the node's cached angles stay in step, and nothing has to be
    stopped.  On the aircraft that is the one to reach for.

    That bench launch is the ONLY place angle_debug is true.
    navigation_2d.launch deliberately keeps it false, and it does not declare it
    as an <arg> either - so `roslaunch navigation_2d navigation_2d.launch
    angle_debug:=true` silently does nothing, which looks exactly like a dead
    topic.  Stop the flight launch before starting the bench one: one PCA9685,
    one set of topics, two writers.

  * This script is the fallback for when ROS is not an option: the node will
    not start (I2C permissions, a parameter it FATALs on, a crash loop) and
    you still need to prove the wiring and find the release angles.

So this tool talks to the PCA9685 directly, using the SAME register sequence
and the SAME angle -> pulse -> tick arithmetic as servo_controller_node.cpp
(setAngle / initializePwm), so what you measure here is what the node will
produce in flight.

The mapping, copied verbatim:

    pulse_us = min_pulse_us + (max_pulse_us - min_pulse_us) * angle / 180
    ticks    = round(pulse_us * pwm_frequency * 4096 / 1_000_000)
    PWM period = 4096 ticks = 1 / pwm_frequency seconds

The tick arithmetic is checked against the C++ implementation in
tools/servo_tick_check.cpp; run --dry-run to exercise it with no hardware.

SAFETY - READ BEFORE RUNNING ANYTHING WITH --angle / --sweep / --interactive
--------------------------------------------------------------------------
* If servo_controller_node is running, use /servo_debug_cmd instead of this
  script (tools/servo_debug.py).  It drives the same PCA9685 through the node
  that already owns it, so there is nothing to stop, no cached angle to
  resynchronise, and the startup sequence is not repeated underneath a live
  flight process.
* Stop servo_controller_node first.  Two writers on one PCA9685 means the
  node's cached angle and the real angle disagree, and the node's next
  DROP_FIRE will look like a no-op.  The tool checks `rosnode list` and
  refuses unless you pass --force.
* A sweep WILL RELEASE A LOADED CONTAINER if it passes a release angle.
  Test with the mechanism unloaded, or with the container mechanically
  isolated, until you know where the two release angles are.
* initializePwm() rewrites MODE1/MODE2/prescale, which glitches every
  channel for a few hundred microseconds.  Pass --no-init to skip it when
  the node has already configured the chip and nothing has been reset.
* The camera channel is not touched unless you ask for it, and PWM outputs
  hold their last value, so the camera keeps its position while this runs.

Usage
-----
    # no hardware needed: print the angle -> pulse -> tick table
    python3 tools/servo_angle_calib.py --dry-run --angles 30 55 90 140 170

    # one shot
    python3 tools/servo_angle_calib.py --angle 90

    # bracket a release angle
    python3 tools/servo_angle_calib.py --sweep 60 150 10

    # hand-driven: type angles, `p` to park, `q` to quit
    python3 tools/servo_angle_calib.py --interactive
"""

import argparse
import math
import os
import struct
import subprocess
import sys
import time

# fcntl is Unix-only, so it is imported inside Pca9685.open() rather than here.
# That is deliberate: --dry-run must keep working on the Windows box where the
# arithmetic gets cross-checked against tools/servo_tick_check.cpp, and a
# top-level `import fcntl` would make the whole tool unimportable there.

# PCA9685 registers, same ones servo_controller_node.cpp pokes.
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

MODE1_SLEEP = 0x10
MODE1_AI = 0x20
MODE1_RESTART = 0x80
MODE2_OUTDRV = 0x04

# i2c-dev ioctl for "talk to this slave address from now on".
I2C_SLAVE = 0x0703

OSC_HZ = 25_000_000.0
TICKS_PER_PERIOD = 4096.0

# The four angles servo_controller_node resolves its commands to, plus the
# parked position, all from navigation_2d.launch.
NODE_ANGLES = {
    30: "drop_safe_angle (parked)",
    55: "down_angle (camera)",
    90: "fire_drop_angle (DROP_FIRE)",
    140: "rescue_drop_angle (DROP_RESCUE)",
    170: "front_angle (camera)",
}


def clamp_angle(angle):
    """Same clamp as the first statement of ServoController::setAngle.

    Not cosmetic: setAngle clamps before it computes the pulse width, so an
    angle of 200 goes to the 180 position and not to a 2417 us pulse that the
    servo cannot reach. Without this the bench would measure a pulse the
    flight code never produces. The C++ cross-check in tools/servo_tick_check.cpp
    caught exactly this at angle 200 (it printed 461, this tool printed 495).
    """
    return max(0.0, min(180.0, angle))


def pulse_us_for(angle, min_pulse_us, max_pulse_us):
    """Same expression as ServoController::setAngle."""
    angle = clamp_angle(angle)
    return min_pulse_us + (max_pulse_us - min_pulse_us) * angle / 180.0


def ticks_for(angle, min_pulse_us, max_pulse_us, pwm_frequency):
    """Same expression as ServoController::setAngle.

    Python's round() is banker's rounding where C++ std::lround() rounds half
    away from zero, so do it explicitly - the two differ on exactly .5 and
    that is a one-tick disagreement with the flight code.
    """
    pulse_us = pulse_us_for(angle, min_pulse_us, max_pulse_us)
    raw = pulse_us * pwm_frequency * TICKS_PER_PERIOD / 1_000_000.0
    return int(math.floor(raw + 0.5))


class Pca9685(object):
    """Minimal i2c-dev driver, register-for-register the node's behaviour."""

    def __init__(self, device, address, frequency, min_pulse_us, max_pulse_us,
                 dry_run=False):
        self.device = device
        self.address = address
        self.frequency = frequency
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.dry_run = dry_run
        self.fd = -1

    def open(self):
        if self.dry_run:
            return
        import fcntl  # Unix-only; see the note next to the top-level imports
        self.fd = os.open(self.device, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
        except OSError as exc:
            os.close(self.fd)
            self.fd = -1
            raise SystemExit(
                "无法在 %s 上选中 PCA9685 地址 0x%02x：%s\n"
                "检查：i2cdetect -y 1 是否能看到 0x%02x；用户是否在 i2c 组；"
                "modprobe i2c-dev 是否已加载。" % (self.device, self.address, exc,
                                                   self.address))

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def write_register(self, register, value):
        if self.dry_run:
            print("    [dry-run] reg 0x%02X <- 0x%02X" % (register, value))
            return
        os.write(self.fd, struct.pack("BB", register, value & 0xFF))

    def initialize_pwm(self):
        prescale = int(math.floor(
            OSC_HZ / (TICKS_PER_PERIOD * float(self.frequency)) - 1.0 + 0.5))
        self.write_register(MODE1, MODE1_SLEEP)
        self.write_register(MODE2, MODE2_OUTDRV)
        self.write_register(PRESCALE, prescale)
        self.write_register(MODE1, MODE1_AI)
        if not self.dry_run:
            time.sleep(0.0005)
        self.write_register(MODE1, MODE1_AI | MODE1_RESTART)
        print("PCA9685 init: %s addr 0x%02X, %d Hz, prescale %d, "
              "pulse %d..%d us" % (self.device, self.address, self.frequency,
                                   prescale, self.min_pulse_us,
                                   self.max_pulse_us))

    def set_angle(self, channel, angle):
        ticks = ticks_for(angle, self.min_pulse_us, self.max_pulse_us,
                          self.frequency)
        payload = struct.pack("BBBBB", LED0_ON_L + 4 * channel, 0, 0,
                              ticks & 0xFF, (ticks >> 8) & 0xFF)
        if self.dry_run:
            print("    [dry-run] ch%d <- %6.1f deg  (%7.1f us, %4d ticks)"
                  % (channel, angle,
                     pulse_us_for(angle, self.min_pulse_us, self.max_pulse_us),
                     ticks))
        else:
            os.write(self.fd, payload)
            print("  ch%d -> %6.1f deg   %7.1f us   %4d ticks"
                  % (channel, angle,
                     pulse_us_for(angle, self.min_pulse_us, self.max_pulse_us),
                     ticks))
        return ticks


def warn_if_node_running(force):
    """Two writers on one PCA9685 is the failure this prevents."""
    try:
        out = subprocess.check_output(["rosnode", "list"],
                                      stderr=subprocess.DEVNULL,
                                      timeout=5).decode("utf-8", "replace")
    except Exception:
        print("提示：无法执行 rosnode list（ROS 未 source？）——"
              "请自行确认 servo_controller_node 没有在跑。")
        return
    if "servo_controller_node" not in out:
        return
    message = (
        "servo_controller_node 正在运行，它自己也在写同一个 PCA9685，\n"
        "两边会互相覆盖角度（节点的缓存角度和真实角度将不再一致）。\n"
        "先把它停掉：rosnode kill /servo_controller_node")
    if force:
        print("警告：" + message + "\n（--force 已指定，继续。）")
    else:
        raise SystemExit("拒绝执行：" + message + "\n确实要继续请加 --force。")


def node_label(angle):
    """Name of the node parameter this angle corresponds to, if any.

    Only an exact match counts. int(90.5) == 90, so a naive dict lookup would
    label 90.5 deg as "fire_drop_angle", which is how a typo in a parameter
    gets read back as a value that was never configured.
    """
    if angle != int(angle):
        return ""
    return NODE_ANGLES.get(int(angle), "")


def print_table(angles, min_pulse_us, max_pulse_us, pwm_frequency):
    period_ms = 1000.0 / pwm_frequency
    print("角度 -> 脉宽 -> ticks   (min %d us, max %d us, %d Hz, 周期 %.2f ms, 4096 ticks)"
          % (min_pulse_us, max_pulse_us, pwm_frequency, period_ms))
    print("  %8s  %10s  %6s  %s" % ("angle", "pulse_us", "ticks", "used by"))
    for angle in angles:
        ticks = ticks_for(angle, min_pulse_us, max_pulse_us, pwm_frequency)
        notes = []
        clamped = clamp_angle(angle)
        if abs(clamped - angle) > 1e-9:
            notes.append("<<< 超出 0..180，节点会钳到 %.1f" % clamped)
        if ticks <= 0 or ticks >= int(TICKS_PER_PERIOD):
            notes.append("<<< ticks 超出 0..4095，脉宽无效")
        label = node_label(angle)
        suffix = " ".join([label] + notes)
        print("  %8.1f  %10.1f  %6d  %s"
              % (angle, pulse_us_for(angle, min_pulse_us, max_pulse_us),
                 ticks, suffix))


def main():
    parser = argparse.ArgumentParser(
        description="直接驱动 PCA9685 的一个舵机通道到任意角度（舵机标定用）。",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/i2c-1")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x40,
                        help="PCA9685 地址，默认 0x40")
    parser.add_argument("--channel", type=int, default=1,
                        help="舵机通道；1 = drop_channel（投放），0 = camera_channel")
    parser.add_argument("--frequency", type=int, default=50)
    parser.add_argument("--min-pulse-us", type=int, default=750)
    parser.add_argument("--max-pulse-us", type=int, default=2250)
    parser.add_argument("--settle", type=float, default=1.0,
                        help="每个角度保持多久才打印下一行，默认 1.0 s")
    parser.add_argument("--park", type=float, default=30.0,
                        help="退出时回到的安全角（drop_safe_angle），默认 30")
    parser.add_argument("--keep", action="store_true",
                        help="退出时不要回安全角（停在最后一个角度）")
    parser.add_argument("--no-init", action="store_true",
                        help="跳过 initializePwm（芯片已由节点配好、且没被复位时）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不碰 I2C，只做算术与打印（可在无硬件的机器上跑）")
    parser.add_argument("--force", action="store_true",
                        help="即使 servo_controller_node 在跑也继续")
    parser.add_argument("--angles", type=float, nargs="+",
                        default=sorted(NODE_ANGLES.keys()),
                        help="--dry-run 时打印这些角度的换算表")
    parser.add_argument("--angle", type=float,
                        help="单发：到指定角度")
    parser.add_argument("--sweep", type=float, nargs=3,
                        metavar=("START", "END", "STEP"),
                        help="扫描一段角度")
    parser.add_argument("--interactive", action="store_true",
                        help="手工输入角度，p 回安全角，q 退出")
    args = parser.parse_args()

    if args.dry_run:
        print_table(args.angles, args.min_pulse_us, args.max_pulse_us,
                    args.frequency)
        print()
        print("（--dry-run：以上为换算表，未接触硬件。加 --angle / --sweep / "
              "--interactive 前请先读文件头的 SAFETY 段。）")
        return 0

    if args.min_pulse_us <= 0 or args.max_pulse_us <= args.min_pulse_us:
        raise SystemExit("脉宽范围非法：%d..%d" % (args.min_pulse_us,
                                                  args.max_pulse_us))

    warn_if_node_running(args.force)

    driver = Pca9685(args.device, args.address, args.frequency,
                     args.min_pulse_us, args.max_pulse_us)
    driver.open()
    try:
        if not args.no_init:
            driver.initialize_pwm()

        if args.angle is not None:
            driver.set_angle(args.channel, args.angle)
            time.sleep(max(0.0, args.settle))

        elif args.sweep is not None:
            start, end, step = args.sweep
            if step == 0 or (end - start) * step < 0:
                raise SystemExit("扫描参数非法：start=%g end=%g step=%g"
                                 % (start, end, step))
            count = int(abs((end - start) / step)) + 1
            print("扫描 ch%d：%.1f -> %.1f 步进 %.1f（%d 个点）"
                  % (args.channel, start, end, step, count))
            angle = start
            for _ in range(count):
                driver.set_angle(args.channel, angle)
                time.sleep(max(0.0, args.settle))
                angle += step

        elif args.interactive:
            print("手工模式：输入角度回车即动；p = 回 %.1f 度；q = 退出"
                  % args.park)
            while True:
                try:
                    line = input("angle> ").strip()
                except EOFError:
                    break
                if line in ("q", "quit", "exit"):
                    break
                if line == "p":
                    driver.set_angle(args.channel, args.park)
                    continue
                try:
                    driver.set_angle(args.channel, float(line))
                except ValueError:
                    print("  不是数字（p = 回 %.1f，q = 退出）" % args.park)
        else:
            print_table(args.angles, args.min_pulse_us, args.max_pulse_us,
                        args.frequency)
            print()
            print("只打印了换算表，没有动作。要给角度请用 --angle / --sweep / "
                  "--interactive。")

        if not args.keep:
            driver.set_angle(args.channel, args.park)
            print("已回到安全角 %.1f 度" % args.park)
    finally:
        driver.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
