#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

class ServoController
{
public:
  ServoController()
      : private_nh_("~")
  {
    private_nh_.param<std::string>("i2c_device", i2c_device_, "/dev/i2c-1");
    private_nh_.param<int>("i2c_address", i2c_address_, 0x40);
    private_nh_.param<int>("camera_channel", camera_channel_, 0);
    private_nh_.param<int>("drop_channel", drop_channel_, 1);
    private_nh_.param<int>("pwm_frequency", pwm_frequency_, 50);
    private_nh_.param<int>("min_pulse_us", min_pulse_us_, 750);
    private_nh_.param<int>("max_pulse_us", max_pulse_us_, 2250);
    private_nh_.param<double>("front_angle", front_angle_, 170.0);
    private_nh_.param<double>("down_angle", down_angle_, 55.0);
    private_nh_.param<double>("drop_safe_angle", drop_safe_angle_, 30.0);
    private_nh_.param<double>("fire_drop_angle", fire_drop_angle_, 90.0);
    private_nh_.param<double>("rescue_drop_angle", rescue_drop_angle_, 140.0);
    private_nh_.param<double>("servo_motion_seconds", servo_motion_seconds_, 0.8);
    private_nh_.param<double>("servo_seconds_per_degree", servo_seconds_per_degree_, 0.006);
    private_nh_.param<double>("startup_settle_seconds", startup_settle_seconds_, 0.8);
    private_nh_.param<double>("drop_action_seconds", drop_action_seconds_, 1.0);
    private_nh_.param<bool>("restore_after_drop", restore_after_drop_, false);
    // Bench-only: accept an arbitrary angle on /servo_debug_cmd. Default false,
    // and the launch file that flies the mission must leave it false - the whole
    // point of the four fixed commands above is that a stray message cannot move
    // the drop servo. tools/check_config.py fails if the mission launch turns it
    // on. See servoCommandCallback for the accepted strings.
    private_nh_.param<bool>("angle_debug", angle_debug_, false);

    if (i2c_address_ < 0x03 || i2c_address_ > 0x77 ||
        camera_channel_ < 0 || camera_channel_ > 15 ||
        drop_channel_ < 0 || drop_channel_ > 15 ||
        camera_channel_ == drop_channel_ ||
        pwm_frequency_ < 40 || pwm_frequency_ > 1000 ||
        min_pulse_us_ <= 0 || max_pulse_us_ <= min_pulse_us_)
    {
      ROS_FATAL("Invalid PCA9685 I2C address, channel, frequency, or pulse range");
      ros::shutdown();
      return;
    }

    // The two competitions use two different materials, and on this mechanism the
    // servo ANGLE is what selects which one releases: DROP_FIRE goes to
    // fire_drop_angle, DROP_RESCUE to rescue_drop_angle. Three things are worth
    // catching here rather than in the air:
    //  * an angle outside 0..180 is silently clamped by setAngle, so the aircraft
    //    would move to a different angle than the parameter claims;
    //  * two angles that are nearly equal cannot be distinguished by the
    //    mechanism, so one of the two drops would release the wrong container;
    //  * the parked angle sitting on top of one of them would release that
    //    container before the flight even starts.
    if (drop_safe_angle_ < 0.0 || drop_safe_angle_ > 180.0 ||
        fire_drop_angle_ < 0.0 || fire_drop_angle_ > 180.0 ||
        rescue_drop_angle_ < 0.0 || rescue_drop_angle_ > 180.0)
    {
      ROS_WARN("A drop servo angle is outside 0..180 (safe %.1f, fire %.1f, "
               "rescue %.1f); setAngle clamps, so the servo would not go where "
               "the parameter says",
               drop_safe_angle_, fire_drop_angle_, rescue_drop_angle_);
    }
    if (std::fabs(fire_drop_angle_ - rescue_drop_angle_) < 5.0)
    {
      ROS_WARN("fire_drop_angle (%.1f) and rescue_drop_angle (%.1f) are only %.1f "
               "deg apart, so the mechanism cannot tell the two materials apart",
               fire_drop_angle_, rescue_drop_angle_,
               std::fabs(fire_drop_angle_ - rescue_drop_angle_));
    }
    if (std::fabs(drop_safe_angle_ - fire_drop_angle_) < 5.0 ||
        std::fabs(drop_safe_angle_ - rescue_drop_angle_) < 5.0)
    {
      ROS_WARN("drop_safe_angle (%.1f) is within 5 deg of a drop angle "
               "(fire %.1f, rescue %.1f); the parked position would already be a "
               "release position",
               drop_safe_angle_, fire_drop_angle_, rescue_drop_angle_);
    }

    if (!openDevice() || !initializePwm() ||
        !setAngle(camera_channel_, front_angle_) ||
        !setAngle(drop_channel_, drop_safe_angle_))
    {
      ROS_FATAL("Unable to initialize PCA9685 outputs; refusing servo operation");
      closeDevice();
      ros::shutdown();
      return;
    }
    camera_angle_ = front_angle_;
    drop_angle_ = drop_safe_angle_;
    initialized_ = true;

    // Printed once so a run's two drop angles can be read back out of the log
    // alone, next to the mission node's "Drop command sent: DROP_FIRE (servo 1
    // -> fire_drop_angle)" line.
    ROS_INFO("Drop servo: channel %d parked at %.1f deg; DROP_FIRE -> %.1f deg, "
             "DROP_RESCUE -> %.1f deg, hold %.2f s",
             drop_channel_, drop_safe_angle_, fire_drop_angle_, rescue_drop_angle_,
             drop_action_seconds_);

    command_sub_ = nh_.subscribe("/gpio_cmd", 4,
                                 &ServoController::commandCallback, this);
    ack_pub_ = nh_.advertise<std_msgs::String>("/gpio_ack", 4, false);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/servo_ready", 1, true);
    ready_timer_ = nh_.createTimer(
        ros::Duration(std::max(0.1, startup_settle_seconds_)),
        &ServoController::publishReady, this, true);
    ROS_INFO("PCA9685 initialized; camera is front-facing while waiting for servo settle");

    if (angle_debug_)
    {
      debug_sub_ = nh_.subscribe("/servo_debug_cmd", 4,
                                 &ServoController::servoCommandCallback, this);
      debug_ack_pub_ = nh_.advertise<std_msgs::String>("/servo_debug_ack", 4, false);
      ROS_WARN("angle_debug is ON: /servo_debug_cmd accepts "
               "\"SET_DROP <deg>\", \"SET_CAMERA <deg>\", \"PARK\", \"STATUS\"; "
               "replies come back on /servo_debug_ack. This is a bench setting - "
               "it must never be on for a competition run.");
    }
    else
    {
      ROS_INFO("angle_debug is off; /servo_debug_cmd is not subscribed, so only "
               "the four flight commands can move a servo");
    }
  }

  ~ServoController()
  {
    closeDevice();
  }

private:
  bool openDevice()
  {
    fd_ = ::open(i2c_device_.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0)
    {
      ROS_FATAL("Cannot open %s: %s", i2c_device_.c_str(), std::strerror(errno));
      return false;
    }
    if (::ioctl(fd_, I2C_SLAVE, i2c_address_) < 0)
    {
      ROS_FATAL("Cannot select PCA9685 address 0x%02x on %s: %s",
                i2c_address_, i2c_device_.c_str(), std::strerror(errno));
      closeDevice();
      return false;
    }
    return true;
  }

  void closeDevice()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool writeRegister(uint8_t reg, uint8_t value)
  {
    const uint8_t bytes[2] = {reg, value};
    if (::write(fd_, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes)))
    {
      ROS_ERROR("PCA9685 register write 0x%02x failed: %s", reg, std::strerror(errno));
      return false;
    }
    return true;
  }

  bool initializePwm()
  {
    const double prescale_value =
        25000000.0 / (4096.0 * static_cast<double>(pwm_frequency_)) - 1.0;
    const uint8_t prescale = static_cast<uint8_t>(std::lround(prescale_value));
    if (!writeRegister(0x00, 0x10) ||  // sleep before changing prescale
        !writeRegister(0x01, 0x04) ||  // totem-pole output mode
        !writeRegister(0xFE, prescale) ||
        !writeRegister(0x00, 0x20))    // wake with auto-increment
    {
      return false;
    }
    ::usleep(500);
    return writeRegister(0x00, 0xA0);  // restart oscillator
  }

  // The angle -> pulse -> tick arithmetic, in one place. setAngle() and the
  // bench readout on /servo_debug_ack both have to agree with each other and
  // with tools/servo_angle_calib.py, so neither of them may keep its own copy.
  //
  // Note the clamp: it happens BEFORE the pulse width is computed, so an angle
  // of 200 goes to the 180 position and not to a 2417 us pulse. A bench tool
  // that measured the unclamped value would report a pulse this node never
  // produces - tools/servo_tick_check.cpp caught exactly that in the first
  // version of servo_angle_calib.py.
  double pulseUsFor(double angle) const
  {
    angle = std::max(0.0, std::min(180.0, angle));
    return min_pulse_us_ + (max_pulse_us_ - min_pulse_us_) * angle / 180.0;
  }

  uint16_t ticksFor(double angle) const
  {
    return static_cast<uint16_t>(
        std::lround(pulseUsFor(angle) * pwm_frequency_ * 4096.0 / 1000000.0));
  }

  bool setAngle(int channel, double angle)
  {
    if (!std::isfinite(angle))
    {
      ROS_ERROR("Ignoring non-finite servo angle");
      return false;
    }
    const uint16_t ticks = ticksFor(angle);
    const uint8_t bytes[5] = {
        static_cast<uint8_t>(0x06 + 4 * channel),
        0,
        0,
        static_cast<uint8_t>(ticks & 0xFF),
        static_cast<uint8_t>(ticks >> 8)};
    if (::write(fd_, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes)))
    {
      ROS_ERROR("PCA9685 channel %d write failed: %s", channel, std::strerror(errno));
      return false;
    }
    return true;
  }

  void publishReady(const ros::TimerEvent &)
  {
    if (!initialized_)
      return;
    std_msgs::Bool ready;
    ready.data = true;
    ready_pub_.publish(ready);
    ROS_INFO("Servo controller ready; startup camera position is front-facing");
    ready_timer_ = nh_.createTimer(
        ros::Duration(1.0), &ServoController::publishReady, this, true);
  }

  bool publishAck(const std::string &command)
  {
    if (command.empty())
      return false;
    std_msgs::String ack;
    ack.data = command + "_DONE";
    ack_pub_.publish(ack);
    return true;
  }

  void commandCallback(const std_msgs::String::ConstPtr &message)
  {
    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "Ignoring servo command before successful PCA9685 initialization");
      return;
    }
    const std::string command = message->data;
    if (command == "CAMERA_FRONT" || command == "Front")
    {
      beginCameraMove(front_angle_, "CAMERA_FRONT");
    }
    else if (command == "CAMERA_DOWN" || command == "Down")
    {
      beginCameraMove(down_angle_, "CAMERA_DOWN");
    }
    else if (command == "DROP_FIRE" || command == "Drop")
    {
      startDrop(fire_drop_angle_, "DROP_FIRE");
    }
    else if (command == "DROP_RESCUE")
    {
      startDrop(rescue_drop_angle_, "DROP_RESCUE");
    }
    else
    {
      ROS_WARN("Unknown servo command: %s", command.c_str());
    }
  }

  // ---------------------------------------------------------------------------
  // bench angle entry - compiled in always, reachable only when angle_debug is
  // true.  On the aircraft the four flight commands above are the whole
  // vocabulary on purpose (a stray message must not be able to move the drop
  // servo); this is the escape hatch that lets you type an arbitrary angle and
  // watch where the horn goes, which is the only way to bracket the two release
  // angles on the real mechanism.
  // ---------------------------------------------------------------------------

  // "SET_DROP 90" / "SET_DROP:90" / "SET_DROP=90" -> ("SET_DROP", "90").
  // The verb is upper-cased so "set_drop 90" works, and an unrecognised verb is
  // answered rather than ignored: on a bench "nothing happened" and "the angle
  // was refused" look exactly the same, and that ambiguity is what costs time.
  static void splitVerbValue(const std::string &text, std::string &verb,
                             std::string &value)
  {
    std::string::size_type begin = 0;
    std::string::size_type end = text.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])))
      ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
      --end;
    const std::string body = text.substr(begin, end - begin);

    std::string::size_type split = 0;
    while (split < body.size() &&
           !std::isspace(static_cast<unsigned char>(body[split])) &&
           body[split] != ':' && body[split] != '=')
      ++split;
    verb = body.substr(0, split);
    for (std::string::size_type i = 0; i < verb.size(); ++i)
      verb[i] = static_cast<char>(
          std::toupper(static_cast<unsigned char>(verb[i])));

    std::string::size_type value_begin = split;
    while (value_begin < body.size() &&
           (std::isspace(static_cast<unsigned char>(body[value_begin])) ||
            body[value_begin] == ':' || body[value_begin] == '='))
      ++value_begin;
    value = body.substr(value_begin);
  }

  std::string describeAngle(int channel, double angle, const char *what) const
  {
    std::ostringstream out;
    out << what << " ch" << channel << " -> " << angle << " deg ("
        << pulseUsFor(angle) << " us, " << ticksFor(angle) << " ticks)";
    return out.str();
  }

  void publishDebugAck(const std::string &text)
  {
    std_msgs::String ack;
    ack.data = text;
    debug_ack_pub_.publish(ack);
    ROS_INFO("servo debug reply: %s", text.c_str());
  }

  void setDebugAngle(int channel, const std::string &value, const char *what)
  {
    if (value.empty())
    {
      publishDebugAck(std::string("ERR ") + what +
                      " needs an angle in degrees, e.g. \"SET_DROP 90\"");
      return;
    }
    double angle = 0.0;
    try
    {
      angle = std::stod(value);
    }
    catch (const std::exception &)
    {
      publishDebugAck(std::string("ERR ") + what + " \"" + value +
                      "\" is not a number");
      return;
    }
    if (!std::isfinite(angle))
    {
      publishDebugAck(std::string("ERR ") + what + " \"" + value +
                      "\" is not finite");
      return;
    }
    const double clamped = std::max(0.0, std::min(180.0, angle));
    if (!setAngle(channel, clamped))
    {
      publishDebugAck(std::string("ERR ") + what + " write to the PCA9685 failed");
      return;
    }
    if (channel == drop_channel_)
      drop_angle_ = clamped;
    else
      camera_angle_ = clamped;

    std::string reply = describeAngle(channel, clamped, what);
    if (std::fabs(clamped - angle) > 1e-9)
      reply += " [clamped from " + value + "; the servo cannot go past 0..180]";
    publishDebugAck(reply);
  }

  void servoCommandCallback(const std_msgs::String::ConstPtr &message)
  {
    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(
          1.0, "Ignoring servo debug command before successful PCA9685 initialization");
      return;
    }
    std::string verb;
    std::string value;
    splitVerbValue(message->data, verb, value);

    if (verb == "SET_DROP")
    {
      setDebugAngle(drop_channel_, value, "DROP");
    }
    else if (verb == "SET_CAMERA")
    {
      setDebugAngle(camera_channel_, value, "CAMERA");
    }
    else if (verb == "PARK")
    {
      if (!setAngle(drop_channel_, drop_safe_angle_))
      {
        publishDebugAck("ERR PARK write to the PCA9685 failed");
        return;
      }
      drop_angle_ = drop_safe_angle_;
      publishDebugAck(describeAngle(drop_channel_, drop_safe_angle_, "PARK"));
    }
    else if (verb == "STATUS")
    {
      // The angles the node would actually use right now, so a bench reading can
      // be compared against the launch file without guessing.
      std::ostringstream out;
      out << "STATUS camera ch" << camera_channel_ << " " << camera_angle_
          << " deg, drop ch" << drop_channel_ << " " << drop_angle_
          << " deg; DROP_FIRE " << fire_drop_angle_ << " deg, DROP_RESCUE "
          << rescue_drop_angle_ << " deg, parked " << drop_safe_angle_
          << " deg; restore_after_drop " << (restore_after_drop_ ? "true" : "false");
      publishDebugAck(out.str());
    }
    else
    {
      publishDebugAck("ERR unknown command \"" + verb +
                      "\"; use SET_DROP <deg>, SET_CAMERA <deg>, PARK, STATUS");
    }
  }

  void beginCameraMove(double angle, const std::string &command)
  {
    const double previous_angle = camera_angle_;
    if (!setAngle(camera_channel_, angle))
      return;
    camera_angle_ = angle;
    pending_ack_ = command;
    action_timer_.stop();
    const double motion_duration = std::max(
        servo_motion_seconds_, moveDuration(previous_angle, angle));
    if (std::fabs(previous_angle - angle) < 1e-6)
    {
      action_timer_ = nh_.createTimer(
          ros::Duration(std::max(0.0, servo_motion_seconds_)),
          &ServoController::finishCameraMove, this, true);
      return;
    }
    action_timer_ = nh_.createTimer(
        ros::Duration(motion_duration),
        &ServoController::finishCameraMove, this, true);
  }

  double moveDuration(double from, double to) const
  {
    return std::max(0.1, std::fabs(to - from) * servo_seconds_per_degree_);
  }

  void finishCameraMove(const ros::TimerEvent &)
  {
    if (!publishAck(pending_ack_))
    {
      ROS_ERROR("Camera servo move was not acknowledged because its PWM write failed");
    }
  }

  void startDrop(double angle, const std::string &command)
  {
    const double previous_angle = drop_angle_;
    if (!setAngle(drop_channel_, angle))
      return;
    drop_angle_ = angle;
    pending_ack_ = command;
    ROS_INFO("%s: drop servo %d -> %.1f deg (was %.1f), releasing, ack in %.2f s",
             command.c_str(), drop_channel_, angle, previous_angle,
             drop_action_seconds_);
    action_timer_.stop();
    action_timer_ = nh_.createTimer(
        ros::Duration(std::max(0.0, drop_action_seconds_)),
        &ServoController::finishDrop, this, true);
  }

  void finishDrop(const ros::TimerEvent &)
  {
    if (restore_after_drop_ && !setAngle(drop_channel_, drop_safe_angle_))
      return;
    if (restore_after_drop_)
      drop_angle_ = drop_safe_angle_;
    if (!publishAck(pending_ack_))
    {
      ROS_ERROR("Drop action was not acknowledged because its PWM write failed");
      return;
    }
    ROS_INFO("%s complete; drop servo held at %.1f deg", pending_ack_.c_str(),
             drop_angle_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber command_sub_;
  // Only subscribed when angle_debug is true; see servoCommandCallback.
  ros::Subscriber debug_sub_;
  ros::Publisher ack_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher debug_ack_pub_;
  ros::Timer ready_timer_;
  ros::Timer action_timer_;
  std::string i2c_device_;
  std::string pending_ack_;
  int fd_ = -1;
  int i2c_address_ = 0x40;
  int camera_channel_ = 0;
  int drop_channel_ = 1;
  int pwm_frequency_ = 50;
  int min_pulse_us_ = 750;
  int max_pulse_us_ = 2250;
  double front_angle_ = 170.0;
  double down_angle_ = 55.0;
  double camera_angle_ = 170.0;
  double drop_angle_ = 30.0;
  double drop_safe_angle_ = 30.0;
  double fire_drop_angle_ = 90.0;
  double rescue_drop_angle_ = 140.0;
  double servo_motion_seconds_ = 0.8;
  double servo_seconds_per_degree_ = 0.006;
  double startup_settle_seconds_ = 1.0;
  double drop_action_seconds_ = 1.0;
  bool restore_after_drop_ = false;
  // Bench-only arbitrary-angle entry on /servo_debug_cmd. False in flight.
  bool angle_debug_ = false;
  bool initialized_ = false;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_controller_node");
  ServoController controller;
  if (ros::ok())
    ros::spin();
  return 0;
}
