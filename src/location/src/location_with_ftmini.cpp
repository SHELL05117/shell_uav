// SLAM + measured-height bridge. Missing/stale samples never acquire new time.
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <TFplus.h>
#include <tf_mini_i2c.h>
#include <localization_guard.h>

class LocalizationBridge
{
public:
  explicit LocalizationBridge(ros::NodeHandle nh) : nh_(nh), listener_(buffer_)
  {
    nh_.param("target_frame_id", target_frame_, std::string("odom"));
    nh_.param("source_frame_id", source_frame_, std::string("base_link"));
    nh_.param("tf_mini_type", range_type_, std::string("i2c"));
    nh_.param("use_tf_mini", use_range_, true);
    nh_.param("height_reference_calibrated", calibrated_, false);
    nh_.param("publish_rate", publish_rate_, 20.0);
    nh_.param("tf_timeout", tf_timeout_, 0.25);
    nh_.param("range_timeout", range_timeout_, 0.30);
    nh_.param("imu_timeout", imu_timeout_, 0.30);
    nh_.param("tf_mini_height", ground_clearance_, 0.09);
    nh_.param("tf_mini_z_min", z_min_, -0.20);
    nh_.param("tf_mini_z_max", z_max_, 3.00);
    nh_.param("tf_mini_max_tilt_deg", max_tilt_, 35.0);
    nh_.param("tf_mini_min_range", range_gate_.minimum, 0.05);
    nh_.param("tf_mini_max_range", range_gate_.maximum, 6.0);
    nh_.param("tf_mini_max_delta", range_gate_.max_delta, 0.60);
    nh_.param("tf_mini_jump_override", range_gate_.confirmations_required, 20);
    nh_.param("tf_pose_max_jump", position_gate_.max_delta, 0.30);
    nh_.param("tf_pose_jump_override", position_gate_.confirmations_required, 10);
    nh_.param("sensor_offset_x", offset_.x, 0.0);
    nh_.param("sensor_offset_y", offset_.y, 0.0);
    nh_.param("sensor_offset_z", offset_.z, 0.0);
    nh_.param("sensor_beam_x", beam_.x, 0.0);
    nh_.param("sensor_beam_y", beam_.y, 0.0);
    nh_.param("sensor_beam_z", beam_.z, -1.0);
    nh_.param("range_frame_id", range_frame_, std::string("tfmini_link"));
    if (!validConfiguration()) throw std::runtime_error("Invalid localization limits or geometry");
    position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("status", 1, true);
    range_audit_pub_ = nh_.advertise<std_msgs::String>("range_audit", 4);
    status("STARTING");
    if (use_range_)
    {
      std::string imu_topic, range_topic;
      nh_.param("imu_topic", imu_topic, std::string("/mavros/imu/data"));
      imu_sub_ = nh_.subscribe(imu_topic, 5, &LocalizationBridge::imuCallback, this);
      if (range_type_ == "topic")
      {
        nh_.param("range_topic", range_topic, std::string("/location_test/range"));
        range_sub_ = nh_.subscribe(range_topic, 5, &LocalizationBridge::rangeCallback, this);
      }
      else if (range_type_ == "i2c")
      {
        int bus, address;
        nh_.param("bus", bus, 6); nh_.param("address", address, 16);
        if (bus < 0 || bus > 255 || address < 0x03 || address > 0x77)
          throw std::runtime_error("Invalid I2C bus/address");
        i2c_.reset(new TFMiniPro(static_cast<uint8>(address), static_cast<uint8>(bus)));
        height_timer_ = nh_.createTimer(ros::Duration(0.05), &LocalizationBridge::readHardware, this);
      }
      else if (range_type_ == "uart")
      {
        std::string port; int baud;
        nh_.param("serial_port", port, std::string("/dev/ttyUSB0"));
        nh_.param("baud_rate", baud, 115200);
        uart_.reset(new benewake::TFplus(port, baud));
        if (!uart_->isOpen()) throw std::runtime_error("TFmini UART unavailable");
        height_timer_ = nh_.createTimer(ros::Duration(0.05), &LocalizationBridge::readHardware, this);
      }
      else throw std::runtime_error("tf_mini_type must be i2c, uart or topic");
    }
    pose_timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &LocalizationBridge::publish, this);
  }
private:
  bool validConfiguration() const
  {
    return std::isfinite(publish_rate_) && publish_rate_ > 0.0 && publish_rate_ <= 100.0 &&
      std::isfinite(tf_timeout_) && std::isfinite(range_timeout_) && std::isfinite(imu_timeout_) &&
      tf_timeout_ > 0.0 && range_timeout_ > 0.0 && imu_timeout_ > 0.0 &&
      range_gate_.minimum > 0.0 && range_gate_.maximum > range_gate_.minimum &&
      std::isfinite(range_gate_.maximum) && std::isfinite(range_gate_.max_delta) &&
      range_gate_.max_delta > 0.0 && range_gate_.confirmations_required > 0 &&
      std::isfinite(position_gate_.max_delta) &&
      position_gate_.max_delta > 0.0 && position_gate_.confirmations_required > 0 &&
      std::isfinite(ground_clearance_) && ground_clearance_ >= 0.0 &&
      std::isfinite(z_min_) && std::isfinite(z_max_) && z_min_ < z_max_ &&
      max_tilt_ > 0.0 && max_tilt_ < 80.0 && location_guard::norm(beam_) > 1e-6 &&
      std::isfinite(location_guard::norm(beam_)) && std::isfinite(location_guard::norm(offset_));
  }
  void status(const std::string &text)
  {
    if (last_status_ == text) return;
    last_status_ = text;
    std_msgs::String message; message.data = text; status_pub_.publish(message);
    if (text == "READY") ROS_INFO("location: READY");
    else ROS_WARN("location: %s; vision pose withheld", text.c_str());
  }
  void observeRange(double range, const ros::Time &stamp)
  {
    if (!location_guard::fresh(stamp.toSec(), ros::Time::now().toSec(), range_timeout_)) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    const bool accepted=range_gate_.observe(range, stamp.toSec());
    if (!accepted)
      ROS_WARN_THROTTLE(2.0, "location: range sample rejected (range, jump, or repeated timestamp)");
    if (range_audit_pub_.getNumSubscribers())
    {
      std::ostringstream out; out.precision(16);
      out << "{\"stamp\":" << stamp.toSec() << ",\"range_m\":";
      if (std::isfinite(range)) out << range; else out << "null";
      out << ",\"accepted\":" << (accepted ? "true" : "false")
          << ",\"accepted_stamp\":" << range_gate_.stamp() << '}';
      std_msgs::String message; message.data=out.str(); range_audit_pub_.publish(message);
    }
  }
  void rangeCallback(const sensor_msgs::RangeConstPtr &message)
  {
    if (message->header.frame_id != range_frame_) return;
    if (message->range < message->min_range || message->range > message->max_range) return;
    observeRange(message->range, message->header.stamp);
  }
  void imuCallback(const sensor_msgs::ImuConstPtr &message)
  {
    if (message->header.frame_id != source_frame_ || message->orientation_covariance[0] < 0.0 ||
        !location_guard::fresh(message->header.stamp.toSec(), ros::Time::now().toSec(), imu_timeout_)) return;
    const auto &q = message->orientation;
    location_guard::Quaternion candidate{q.x,q.y,q.z,q.w};
    location_guard::Vector ignored{};
    if (!location_guard::rotate(candidate, beam_, ignored)) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (message->header.stamp <= imu_stamp_) return;
    attitude_ = candidate; imu_stamp_ = message->header.stamp;
  }
  void readHardware(const ros::TimerEvent &)
  {
    std::lock_guard<std::mutex> hardware_lock(hardware_mutex_);
    try { observeRange(i2c_ ? i2c_->ReadData() : uart_->getDist(), ros::Time::now()); }
    catch (const std::exception &error) { ROS_WARN_THROTTLE(2.0, "location: TFmini read: %s", error.what()); }
  }
  void publish(const ros::TimerEvent &)
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    // The installed SLAM is 2D; disabling the independent height sensor is
    // not permission to send its zero z to PX4 as a complete 3D observation.
    if (!use_range_) { status("HEIGHT_DISABLED"); return; }
    geometry_msgs::TransformStamped transform;
    try { transform = buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0)); }
    catch (const tf2::TransformException &) { status("WAIT_TF"); return; }
    const ros::Time now = ros::Time::now();
    if (!location_guard::fresh(transform.header.stamp.toSec(), now.toSec(), tf_timeout_))
    { status("TF_STALE"); return; }
    const auto &translation = transform.transform.translation;
    const auto &rotation = transform.transform.rotation;
    location_guard::Vector ignored{};
    if (!std::isfinite(translation.z) ||
        !location_guard::rotate({rotation.x,rotation.y,rotation.z,rotation.w}, beam_, ignored))
    { status("TF_INVALID"); return; }
    if (!position_gate_.observe(translation.x, translation.y, transform.header.stamp.toSec()))
    { status("TF_JUMP"); return; }
    geometry_msgs::PoseStamped message;
    message.header = transform.header;
    message.pose.position.x = translation.x; message.pose.position.y = translation.y;
    message.pose.orientation = rotation;
    double height = translation.z;
    if (use_range_)
    {
      double range, range_stamp; location_guard::Quaternion attitude; ros::Time imu_stamp;
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!range_gate_.valid()) { status("WAIT_RANGE"); return; }
        range = range_gate_.value(); range_stamp = range_gate_.stamp();
        attitude = attitude_; imu_stamp = imu_stamp_;
      }
      if (!location_guard::fresh(range_stamp, now.toSec(), range_timeout_))
      { status("RANGE_STALE"); return; }
      if (!location_guard::fresh(imu_stamp.toSec(), now.toSec(), imu_timeout_))
      { status("IMU_STALE"); return; }
      if (!calibrated_) { status("HEIGHT_REFERENCE_UNCALIBRATED"); return; }
      if (!location_guard::projectHeight(range, attitude, beam_, offset_, ground_clearance_,
                                        max_tilt_, z_min_, z_max_, height))
      { status("HEIGHT_GEOMETRY_OR_LIMIT"); return; }
      tf2::Quaternion imu(attitude.x,attitude.y,attitude.z,attitude.w); imu.normalize();
      tf2::Quaternion slam(rotation.x,rotation.y,rotation.z,rotation.w); slam.normalize();
      double roll,pitch,yaw,unused_roll,unused_pitch;
      tf2::Matrix3x3(imu).getRPY(roll,pitch,yaw);
      tf2::Matrix3x3(slam).getRPY(unused_roll,unused_pitch,yaw);
      tf2::Quaternion combined; combined.setRPY(roll,pitch,yaw);
      message.pose.orientation.x=combined.x(); message.pose.orientation.y=combined.y();
      message.pose.orientation.z=combined.z(); message.pose.orientation.w=combined.w();
      message.header.stamp.fromSec(std::min(transform.header.stamp.toSec(),
                                           std::min(range_stamp, imu_stamp.toSec())));
    }
    if (!std::isfinite(height) || height < z_min_ || height > z_max_)
    { status("Z_OUT_OF_RANGE"); return; }
    message.pose.position.z = height;
    position_pub_.publish(message);
    status("READY");
  }
  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  ros::Publisher position_pub_, status_pub_, range_audit_pub_;
  ros::Subscriber imu_sub_, range_sub_;
  ros::Timer height_timer_, pose_timer_;
  std::unique_ptr<TFMiniPro> i2c_;
  std::unique_ptr<benewake::TFplus> uart_;
  std::mutex input_mutex_, publish_mutex_, hardware_mutex_;
  location_guard::RangeGate range_gate_;
  location_guard::PositionGate position_gate_;
  location_guard::Quaternion attitude_{0.0,0.0,0.0,0.0};
  location_guard::Vector offset_{0.0,0.0,0.0}, beam_{0.0,0.0,-1.0};
  ros::Time imu_stamp_;
  std::string target_frame_, source_frame_, range_type_, range_frame_, last_status_;
  bool use_range_=true, calibrated_=false;
  double publish_rate_=20.0, tf_timeout_=0.25, range_timeout_=0.30, imu_timeout_=0.30;
  double ground_clearance_=0.09, z_min_=-0.20, z_max_=3.0, max_tilt_=35.0;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "location");
  try
  {
    LocalizationBridge bridge(ros::NodeHandle("~"));
    ros::MultiThreadedSpinner spinner(2); spinner.spin();
  }
  catch (const std::exception &error)
  { ROS_FATAL("location: %s", error.what()); return 1; }
  return 0;
}
