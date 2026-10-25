/***************************************************************************************************************************
 * 机载定位桥：把 SLAM 的 odom->base_link 位姿 + TFmini 高度送给 PX4。
 *
 * 2026 机载硬件固定为：RPLIDAR S2（2D 激光）+ Pixhawk 6C + TFmini（I2C 或 UART）。
 *
 * TFmini 只用来定高：向下装在机臂上，起飞时离地 tf_mini_height（默认 0.09 m）。
 * 它读到的距离 d 减去这个离地间隙，就是“相对起飞点的离地高度”：
 *
 *     z = d - tf_mini_height
 *
 * 起飞时机臂上的 TFmini 读到 0.09，z=0；之后 z 就是机体（传感器）相对起飞点的高度。
 * PX4 的 local z 原点本来就在起飞点，所以这个 z 直接作为 /mavros/vision_pose/pose
 * 的高度即可，无需 IMU 姿态、三维安装偏移或额外的高度基准。
 *
 * 参数 tf_mini_height 只是“起飞时传感器离地多少”；只做定高时，它填 0 也不影响
 * 高度保持（常数在 PX4 的起飞零点里被抵消），填对只是让绝对高度更好读。
 ***************************************************************************************************************************/
#include <cmath>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf_mini_i2c.h>
#include <TFplus.h>

namespace
{
ros::Publisher position_pub;
ros::Timer position_timer, height_timer;
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

std::mutex data_mutex;
double range_distance = 0.0;
bool range_valid = false;

double publish_rate = 20.0;
double tf_mini_height = 0.09;  // 起飞时 TFmini 离地高度
std::string target_frame_id = "odom";
std::string source_frame_id = "base_link";
std::string tf_mini_type = "i2c";
std::string serial_port = "/dev/ttyUSB0";
int baud_rate = 115200;
int i2c_bus = 1;
int i2c_address = 0x10;
bool use_tf_mini = true;

TFMiniPro *tf_mini = nullptr;
benewake::TFplus *tf_uart = nullptr;
}  // namespace

void heightCallback(const ros::TimerEvent &event)
{
  (void)event;
  double distance = -1.0;
  if (tf_mini_type == "i2c" && tf_mini != nullptr)
  {
    try
    {
      distance = tf_mini->ReadData();
    }
    catch (const std::exception &error)
    {
      ROS_WARN_THROTTLE(2.0, "location: TFmini I2C read failed: %s", error.what());
    }
  }
  else if (tf_uart != nullptr)
  {
    distance = tf_uart->getDist();
  }

  if (!std::isfinite(distance) || distance <= 0.0)
    return;

  std::lock_guard<std::mutex> lock(data_mutex);
  range_distance = distance;
  range_valid = true;
}

void positionCallback(const ros::TimerEvent &event)
{
  (void)event;
  if (!tf_buffer->canTransform(target_frame_id, source_frame_id, ros::Time(0)))
  {
    ROS_WARN_THROTTLE(2.0, "location: TF %s -> %s is not available", target_frame_id.c_str(),
                      source_frame_id.c_str());
    return;
  }

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer->lookupTransform(target_frame_id, source_frame_id, ros::Time(0));
  }
  catch (const tf2::TransformException &error)
  {
    ROS_WARN_THROTTLE(2.0, "location: %s", error.what());
    return;
  }

  double distance = 0.0;
  bool have_range = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    distance = range_distance;
    have_range = range_valid;
  }

  geometry_msgs::PoseStamped position;
  position.header.stamp = ros::Time::now();
  position.header.frame_id = transform.header.frame_id;
  position.pose.position.x = transform.transform.translation.x;
  position.pose.position.y = transform.transform.translation.y;
  position.pose.orientation = transform.transform.rotation;

  if (use_tf_mini && have_range)
  {
    // 只做定高：d - 起飞离地间隙，起飞点就是 0。
    position.pose.position.z = distance - tf_mini_height;
  }
  else
  {
    position.pose.position.z = transform.transform.translation.z;
    if (use_tf_mini)
      ROS_WARN_THROTTLE(5.0, "location: TFmini has no reading; using SLAM z");
  }

  position_pub.publish(position);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "location");
  ros::NodeHandle nh_private("~");

  nh_private.param("target_frame_id", target_frame_id, std::string("odom"));
  nh_private.param("source_frame_id", source_frame_id, std::string("base_link"));
  nh_private.param("publish_rate", publish_rate, 20.0);

  nh_private.param("use_tf_mini", use_tf_mini, true);
  nh_private.param("tf_mini_type", tf_mini_type, std::string("i2c"));
  nh_private.param("tf_mini_height", tf_mini_height, 0.09);
  nh_private.param("bus", i2c_bus, 1);
  nh_private.param("address", i2c_address, 0x10);
  nh_private.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate, 115200);

  tf_buffer.reset(new tf2_ros::Buffer());
  tf_buffer->setUsingDedicatedThread(true);
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

  position_pub = nh_private.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

  if (use_tf_mini)
  {
    if (tf_mini_type == "i2c")
    {
      try
      {
        tf_mini = new TFMiniPro(static_cast<uint8>(i2c_address), static_cast<uint8>(i2c_bus));
        ROS_INFO("location: TFmini Plus on /dev/i2c-%d, address 0x%02x, ground clearance %.3f m",
                 i2c_bus, i2c_address, tf_mini_height);
      }
      catch (const std::exception &error)
      {
        ROS_FATAL("location: cannot open TFmini on /dev/i2c-%d: %s", i2c_bus, error.what());
        return 1;
      }
    }
    else if (tf_mini_type == "uart")
    {
      tf_uart = new benewake::TFplus(serial_port, baud_rate);
      if (!tf_uart->isOpen())
      {
        ROS_FATAL("location: cannot open TFmini on %s", serial_port.c_str());
        return 1;
      }
      ROS_INFO("location: TFmini UART on %s @ %d, ground clearance %.3f m", serial_port.c_str(),
               baud_rate, tf_mini_height);
    }
    else
    {
      ROS_FATAL("location: tf_mini_type must be 'i2c' or 'uart', got '%s'", tf_mini_type.c_str());
      return 1;
    }

    height_timer = nh_private.createTimer(ros::Duration(0.05), &heightCallback);
  }

  ROS_INFO("location: publishing %s -> %s at %.0f Hz", target_frame_id.c_str(),
           source_frame_id.c_str(), publish_rate);
  position_timer = nh_private.createTimer(ros::Duration(1.0 / publish_rate), &positionCallback);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  if (tf_uart != nullptr)
    tf_uart->closePort();
  delete tf_mini;
  delete tf_uart;
  return 0;
}
