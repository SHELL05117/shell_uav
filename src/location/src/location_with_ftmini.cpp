#include <TFplus.h>
#include <tf_mini_i2c.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <memory>   // 添加智能指针支持
#include <atomic>
#include <algorithm>
#include <cmath>

using namespace std;

ros::Publisher position_pub;
tf2_ros::Buffer tfBuffer;
// ros::Subscriber pose_sub, height_sub;
geometry_msgs::PoseStamped position;
tf2::Quaternion quat_body;
std::unique_ptr<benewake::TFplus> tfplus_obj;      // 改为智能指针
// height_listener 在独立线程里一直写，position_listener 30 Hz 读。原来是裸 float，
// 读到一个撕裂/半更新的值也会被当成一次测量直接算成 z。用 atomic 把"读到什么"
// 这件事至少变成定义良好的。
std::atomic<float> dist{0.0f};
int baud_rate;
std::string target_frame_id;
std::string source_frame_id, tf_mini_type;
double publish_rate, roll_obj, pitch_obj, yaw_obj, tf_mini_height;
bool is_simulation, rplidar_location, use_tf_mini;
bool height_from_tf_mini;
int bus, address;
std::unique_ptr<TFMiniPro> tf_mini_plus_;          // 改为智能指针

// ---- TF mini 的输入保护 ------------------------------------------------
//
// 2026-10-17 实机：起飞后 odom z 从 1.375 一步跳到 -0.098，0.5 s 内掉了
// 1.47 m，随后 -1.879 / -3.854 / ... 一路积到 -22.6，再被 EKF 重置回 1.22。
// 算术对得上：`z = dist - tf_mini_height`，tf_mini_height 取代码默认的 0.1，
// z=-0.098 就是 dist=0.002 —— TFmini 在飞行中报了一次 2 mm 的假回波（低反射
// 地面/超出量程/被桨叶遮挡都会这样），而这行代码把它当成真值直接发给了
// /mavros/vision_pose/pose。PX4 的高度被这份 vision 拽下去，之后整条估计链
// 都在追一个不存在的高度。
//
// 原来只有一个 `dist > 0.0` 的判断：它挡不住 0.002 这种"正的但不可能"的值，
// 也挡不住一次 1.4 m 的跳变。下面三个门各自挡一件事：
//   量程   —— 挡掉 0 附近的假读数（TFmini Plus 的标称下限 0.1 m，这里留到 0.05）
//   跳变   —— 挡掉单次毛刺；连续 N 个周期都越界则认为是真的快速变化（起飞/下降），
//             重新接受，避免一次拒收之后永久拒收
//   限幅   —— z 是相对起飞点的飞行高度，场地墙高 3.00 m，负数没有物理意义
// 被测数据不合格时保持上一次**有效**的 z，而不是发一个新算出来的错值。
double last_valid_tf_z = 0.0;      // 0.0 表示"还没有过有效测量"
bool have_valid_tf_z = false;
double last_accepted_raw = -1.0;
int reject_streak = 0;

double tf_mini_min_range = 0.05;
double tf_mini_max_range = 6.0;
double tf_mini_max_delta = 0.60;
int tf_mini_jump_override = 20;
double tf_mini_z_min = -0.20;
double tf_mini_z_max = 3.00;

// xy 跳变门的状态与门限，含义见 position_listener 里那一段注释。
double last_ok_x = 0.0;
double last_ok_y = 0.0;
bool have_ok_xy = false;
int xy_reject_streak = 0;
double tf_pose_max_jump = 0.30;
int tf_pose_jump_override = 10;

void position_listener(const ros::TimerEvent &event)
{
  static tf2::Quaternion quat_obj, quat_body;
  geometry_msgs::TransformStamped transformStamped;

  try
  {
    if (tfBuffer.canTransform(target_frame_id, source_frame_id, ros::Time(0)))
    {
      transformStamped = tfBuffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0));

      position.header.stamp = ros::Time::now();
      position.header.frame_id = transformStamped.header.frame_id;

      position.pose.position.x = transformStamped.transform.translation.x;
      position.pose.position.y = transformStamped.transform.translation.y;

      // ---- xy 跳变门 ---------------------------------------------------
      //
      // 2026-10-17 实机那段日志里，odom 在 0.5 s 内
      //     (1.95, -0.74) -> (2.57, -0.96) -> (2.86, -0.77) -> (2.52, -0.83)
      // 而同一时刻 map 稳定在 (2.10, -1.06) 附近。0.35 m/s 的机体不可能在
      // 50 ms 内走 0.6 m，所以这是局部 SLAM 的跳变，不是飞行。它被原样转发
      // 给 PX4 之后，EKF 的位置和高度一起被带走 —— 同一段日志里 z 就是在这
      // 之后开始 5 m/s 地下滑。
      //
      // 拒收时保持上一次有效值（继续以原频率发布，不能让 EKF 饿死）；连续
      // tf_pose_jump_override 帧都越界则认为是真的重定位，重建基准。
      {
        const double tx = position.pose.position.x;
        const double ty = position.pose.position.y;
        const bool jumped = have_ok_xy &&
                            std::hypot(tx - last_ok_x, ty - last_ok_y) > tf_pose_max_jump;
        const bool accept = !jumped || xy_reject_streak >= tf_pose_jump_override;
        if (accept)
        {
          if (jumped)
          {
            ROS_WARN("TF pose moved %.3f m in one cycle; re-baselining after %d "
                     "rejected frames (SLAM relocalisation, not flight)",
                     std::hypot(tx - last_ok_x, ty - last_ok_y), xy_reject_streak);
          }
          last_ok_x = tx;
          last_ok_y = ty;
          have_ok_xy = true;
          xy_reject_streak = 0;
        }
        else
        {
          ++xy_reject_streak;
          ROS_WARN_THROTTLE(1.0,
                            "TF pose rejected: (%.2f, %.2f) is %.3f m from the last "
                            "good (%.2f, %.2f), %d in a row. Forwarding a SLAM jump "
                            "to vision_pose hands it straight to the EKF.",
                            tx, ty, std::hypot(tx - last_ok_x, ty - last_ok_y),
                            last_ok_x, last_ok_y, xy_reject_streak);
          position.pose.position.x = last_ok_x;
          position.pose.position.y = last_ok_y;
        }
      }

      // cout << "use_tf_mini:" << use_tf_mini << endl;
      // cout << "dist:" << dist << endl;
      if (use_tf_mini)
      {
        const double raw = static_cast<double>(dist.load());
        const bool in_range = raw >= tf_mini_min_range && raw <= tf_mini_max_range;
        const bool jumped = last_accepted_raw >= 0.0 &&
                            std::fabs(raw - last_accepted_raw) > tf_mini_max_delta;
        // 连续越界超过 tf_mini_jump_override 个周期，就认为"世界真的变了"
        // （起飞、下降、被重新安装），重建基准。否则一次拒收会永久拒收。
        const bool accept_jump = jumped && reject_streak >= tf_mini_jump_override;

        if (in_range && (!jumped || accept_jump))
        {
          if (!height_from_tf_mini)
          {
            ROS_WARN("Using TF mini's height (raw %.3f m).", raw);
            height_from_tf_mini = true;
          }
          if (accept_jump)
          {
            ROS_WARN("TF mini moved %.3f m in one step; re-baselining after %d "
                     "rejected reads", raw - last_accepted_raw, reject_streak);
          }
          last_accepted_raw = raw;
          reject_streak = 0;

          double z = raw - tf_mini_height;
          if (z < tf_mini_z_min || z > tf_mini_z_max)
          {
            ROS_WARN_THROTTLE(1.0,
                              "TF mini raw %.3f m -> z %.3f m is outside [%.2f, %.2f]; "
                              "clamping. tf_mini_height=%.3f is probably wrong or the "
                              "beam is not looking at the floor.",
                              raw, z, tf_mini_z_min, tf_mini_z_max, tf_mini_height);
            z = std::min(std::max(z, tf_mini_z_min), tf_mini_z_max);
          }
          position.pose.position.z = z;
          last_valid_tf_z = z;
          have_valid_tf_z = true;
        }
        else
        {
          ++reject_streak;
          // 无论以前有没有在"用 TF mini"，拒收都要能看见 —— 只打一次的话，
          // 传感器之后整段失效在日志里就是静默的。
          ROS_WARN_THROTTLE(1.0,
                            "TF mini reading %.3f m rejected (%s), %d in a row; "
                            "holding the last good height%s. A bad ranging value used "
                            "to go straight into vision_pose and take the EKF's z "
                            "with it.",
                            raw,
                            in_range ? "step change too large" : "outside range",
                            reject_streak,
                            have_valid_tf_z ? "" : " - none yet, publishing the previous z");
          height_from_tf_mini = false;
          if (have_valid_tf_z)
          {
            position.pose.position.z = last_valid_tf_z;
          }
        }
      }
      else
      {
        position.pose.position.z = transformStamped.transform.translation.z;
        if (height_from_tf_mini)
        {
          if (rplidar_location)
            ROS_WARN("TF mini is not using, height is 0!");
          else
            ROS_WARN("TF mini is not using, use T265's height!");
          height_from_tf_mini = false;
        }
      }

      if (!rplidar_location)
      {
        quat_obj = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

        quat_body.setRPY(roll_obj, pitch_obj, yaw_obj);
        quat_body = quat_obj * quat_body;
        quat_body.normalize();

        position.pose.orientation.x = quat_body.x();
        position.pose.orientation.y = quat_body.y();
        position.pose.orientation.z = quat_body.z();
        position.pose.orientation.w = quat_body.w();
      }
      else
      {
        position.pose.orientation = transformStamped.transform.rotation;
      }
      position_pub.publish(position);
      if (rplidar_location)
        ROS_WARN_ONCE("publish pose from rplidar");
      else
        ROS_WARN_ONCE("publish pose from t265");

      // body_path.header.stamp = cur_position.header.stamp;
      // body_path.header.frame_id = cur_position.header.frame_id;
      // body_path.poses.push_back(cur_position);
      // body_path_pubisher.publish(body_path);
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void height_listener(const ros::TimerEvent &event)
{
  while (ros::master::check() && ros::ok())
  {
    if (tf_mini_type == "i2c")
    {
      dist.store(tf_mini_plus_->ReadData());
      ros::Duration(0.01).sleep();
    }
    else
    {
      dist.store(tfplus_obj->getDist());
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "location");
  ros::NodeHandle nh_private("~");

  std::string portName;

  nh_private.param("is_simulation", is_simulation, false);
  nh_private.param("rplidar_location", rplidar_location, true);
  nh_private.param("use_tf_mini", use_tf_mini, true);

  nh_private.param("target_frame_id", target_frame_id, std::string("odom"));
  nh_private.param("source_frame_id", source_frame_id, std::string("base_link"));
  nh_private.param("publish_rate", publish_rate, 30.0);

  if (!rplidar_location)
  {
    nh_private.param("roll_obj", roll_obj, 0.0);
    nh_private.param("pitch_obj", pitch_obj, 0.0);
    nh_private.param("yaw_obj", yaw_obj, 0.0);
    quat_body.setRPY(roll_obj, pitch_obj, yaw_obj);
  }

  nh_private.param("tf_mini_height", tf_mini_height, 0.1);
  // TF mini 的输入保护，含义见文件上方那段注释。默认值按"向下测地、场地墙高
  // 3.00 m、飞行高度 1.5 m"给的，不需要在 launch 里写一遍；只有换量程更大的
  // 型号（TFmini Plus 是 0.1..12 m）或改成朝上安装时才需要动。
  nh_private.param("tf_mini_min_range", tf_mini_min_range, 0.05);
  nh_private.param("tf_mini_max_range", tf_mini_max_range, 6.0);
  nh_private.param("tf_mini_max_delta", tf_mini_max_delta, 0.60);
  nh_private.param("tf_mini_jump_override", tf_mini_jump_override, 20);
  nh_private.param("tf_mini_z_min", tf_mini_z_min, -0.20);
  nh_private.param("tf_mini_z_max", tf_mini_z_max, 3.00);
  nh_private.param("tf_pose_max_jump", tf_pose_max_jump, 0.30);
  nh_private.param("tf_pose_jump_override", tf_pose_jump_override, 10);
  ROS_INFO("TF mini gate: accept %.3f..%.3f m, max step %.3f m (%d rejected reads "
           "before re-baselining), z clamped to [%.2f, %.2f], tf_mini_height=%.3f",
           tf_mini_min_range, tf_mini_max_range, tf_mini_max_delta,
           tf_mini_jump_override, tf_mini_z_min, tf_mini_z_max, tf_mini_height);
  ROS_INFO("vision_pose gate: xy jump > %.3f m in one cycle is rejected for up to "
           "%d frames before re-baselining (target frame '%s', source '%s')",
           tf_pose_max_jump, tf_pose_jump_override,
           target_frame_id.c_str(), source_frame_id.c_str());
  nh_private.param("tf_mini_type", tf_mini_type, std::string("i2c"));
  nh_private.param("bus", bus, 1);
  nh_private.param("address", address, 0);

  nh_private.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate, 115200);

  // cout << "target_frame_id:" << target_frame_id << endl;
  // cout << "source_frame_id:" << source_frame_id << endl;
  // cout << "output_rate:" << output_rate << endl;
  // cout << "roll_obj:" << roll_obj << endl;
  // cout << "pitch_obj:" << pitch_obj << endl;
  // cout << "yaw_obj:" << yaw_obj << endl;
  // cout << "use_tf_mini:" << use_tf_mini << endl;

  tf2_ros::TransformListener tfListener(tfBuffer);
  // Release timer handles inside main, before the static ROS TimerManager.
  ros::Timer height_timer;
  ros::Timer timer = nh_private.createTimer(ros::Duration(1.0 / publish_rate), &position_listener); // Define timer for constant loop rate

  // ros::Duration(1.0).sleep();
  if (!is_simulation)
  {
    if (use_tf_mini)
    {
      height_timer = nh_private.createTimer(ros::Duration(0.01), &height_listener, true);
      if (tf_mini_type == "i2c")
      {
        tf_mini_plus_ = std::make_unique<TFMiniPro>(address, bus);  // 使用智能指针
        ROS_INFO("Range sensor ready on i2c bus: %i, address: 0x%x.", bus, address);
      }
      else
      {
        tfplus_obj = std::make_unique<benewake::TFplus>(portName, baud_rate);  // 使用智能指针
      }
      height_from_tf_mini = false;
    }
    else
      height_from_tf_mini = true;
  }
  else
  {
    use_tf_mini = false;
  }

  position_pub = nh_private.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

  ros::MultiThreadedSpinner s(3);
  ros::spin(s);
  return 0;
}
