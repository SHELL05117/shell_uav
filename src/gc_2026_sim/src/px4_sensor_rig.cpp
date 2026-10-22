#include <algorithm>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

class Px4SensorRig
{
public:
  Px4SensorRig()
      : private_nh_("~")
  {
    private_nh_.param<std::string>("vehicle_model", vehicle_model_, "iris");
    private_nh_.param<std::string>("sensor_model", sensor_model_,
                                    "px4_sensor_rig");
    private_nh_.param<std::string>("model_state_service", model_state_service_,
                                    "/gazebo/set_model_state");
    private_nh_.param<double>("update_rate", update_rate_, 20.0);
    // Simulated payload release.
    //
    // The mission ends HAZARD_DROP / RESCUE_DROP when the servo acknowledges
    // (/gpio_ack). In simulation use_real_servos is false, so no servo node runs
    // and this rig is the thing that acks - the mission therefore "drops" a
    // payload that does not exist anywhere. That is invisible in every log and
    // was read as "detection worked but nothing was dropped" on 2026-10-18.
    //
    // The obvious fix - a payload link under the vehicle with a joint that is
    // broken on command - does not work here, for two reasons that are both
    // structural rather than cosmetic:
    //   * the vehicle is PX4's stock iris.sdf, spawned straight out of
    //     $PX4-Autopilot/Tools/sitl_gazebo/models/iris/. It is not in this
    //     repository, so a payload link added to it is lost on every PX4 update;
    //   * Gazebo Classic exposes no ROS service that breaks a joint. Detachable
    //     joints belong to Gazebo Sim; in Classic the C++ API can do it, but only
    //     from inside a model plugin, i.e. again from inside that same PX4 SDF.
    // So the payload is spawned at the moment of release instead: one model
    // created at the vehicle's ground-truth pose minus payload_release_depth,
    // then left to gravity. The released object appears and falls exactly where
    // and when the real one would, and while it is still being carried there is
    // nothing hanging in the down-facing camera's field of view, which is what
    // keeps the task-area cards framed (tools/render_field.py --check-vision).
    private_nh_.param<bool>("payload_on_drop", payload_on_drop_, true);
    private_nh_.param<double>("payload_release_depth", payload_release_depth_,
                              0.30);
    private_nh_.param<std::string>("spawn_service", spawn_service_,
                                    "/gazebo/spawn_sdf_model");
    // Publish odom -> base_link from the Gazebo ground truth. Cartographer then
    // only owns map -> odom, so SLAM drift cannot leak into the odom frame that
    // the flight controller and mission logic actually use. Set false only if
    // something else already provides odom -> base_link.
    private_nh_.param<bool>("publish_odom_tf", publish_odom_tf_, true);
    // Measure the Gazebo-world / PX4-local offset on the first mavros pose
    // instead of trusting the configured values. Set false to use them as-is.
    // Gazebo's world origin and the odom origin are different points: Gazebo
    // spawns this drone ~1.79 m above its own origin, while both
    // /mavros/local_position/pose and cartographer's map origin start at the
    // vehicle. Publishing raw Gazebo coordinates therefore put odom 1.79 m
    // above where every other consumer thinks the drone starts, which surfaced
    // as map->odom holding a constant z of about -1.6 m.
    //
    // auto_odom_origin measures this once at startup from the first mavros
    // pose. It is on by default; the manual odom_offset_* values below are only
    // a fallback for when it is disabled.
    private_nh_.param<bool>("auto_odom_origin", auto_odom_origin_, true);
    private_nh_.param<double>("odom_offset_x", odom_offset_x_, 0.0);
    private_nh_.param<double>("odom_offset_y", odom_offset_y_, 0.0);
    private_nh_.param<double>("odom_offset_z", odom_offset_z_, 0.0);

    // Queue of 1 is deliberate but only safe because the callback no longer
    // blocks: we always want the freshest ground truth, and dropping an
    // intermediate sample is harmless now that the rig sync happens on a timer.
    model_states_sub_ = nh_.subscribe(
        "/gazebo/model_states", 1, &Px4SensorRig::modelStatesCallback, this);
    gpio_sub_ = nh_.subscribe("/gpio_cmd", 10,
                              &Px4SensorRig::gpioCallback, this);
    // Used only to discover the fixed offset between the Gazebo world origin
    // and PX4's local origin, so odom means the same thing to every consumer.
    mavros_pose_sub_ = nh_.subscribe(
        "/mavros/local_position/pose", 1,
        &Px4SensorRig::mavrosPoseCallback, this);
    gpio_ack_pub_ = nh_.advertise<std_msgs::String>("/gpio_ack", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
        "/simulation/ground_truth/odom", 1);
    // The payload only exists inside Gazebo, and the simulator's own GUI is not
    // where this project looks while it flies. Marker array in odom, latched, so
    // a Foxglove panel opened after the drop still shows where it landed.
    payload_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/simulation/payloads", 1, true);
    model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(
        model_state_service_);
    spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(spawn_service_);

    ROS_INFO("PX4 sensor rig ready: vehicle=%s sensor_model=%s",
             vehicle_model_.c_str(), sensor_model_.c_str());

    // Drive the rig from its own timer, NOT from modelStatesCallback.
    //
    // This is not a cosmetic refactor, it is the fix for a hard stall. While
    // the rig sync lived inside the callback, every /gazebo/model_states
    // message performed a BLOCKING SetModelState service call before returning
    // to the ROS spinner. When Gazebo is busy the service waits, the callback
    // waits with it, and because the same callback is what broadcasts
    // odom->base_link and what feeds "Rig check", the transform stream and the
    // ground-truth stream stop together. The log shows exactly that: odom->base_link
    // froze at a single stamp (pose stamp: 50.5880) while the node clock ran on
    // to 54.4 s, i.e. 3.8 s of missing odom while every consumer waited.
    //
    // Splitting them makes the transform stream independent of the service.
    // The rig may lag the newest pose by up to one timer period, which is
    // acceptable: the rig is a visual/lidar attachment, not the odom source.
    sync_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, update_rate_)),
        &Px4SensorRig::syncTimerCallback, this);
  }

private:
  // One released payload: the model name Gazebo knows it by, which material it
  // was, and its latest pose in the Gazebo world frame.
  struct PayloadRecord
  {
    std::string name;
    std::string kind;
    geometry_msgs::Pose pose;
  };

  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    updatePayloadPoses(*msg);
    // Remember what Gazebo says the sensor rig is really doing. Comparing this
    // against the pose we commanded is the only way to tell "the rig follows the
    // drone" apart from "the rig is stuck while the drone flies away".
    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == sensor_model_ && i < msg->pose.size())
      {
        last_rig_actual_ = msg->pose[i];
        have_rig_actual_ = true;
        break;
      }
    }

    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == vehicle_model_ && i < msg->pose.size() &&
          i < msg->twist.size())
      {
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          vehicle_pose_ = msg->pose[i];
          have_vehicle_pose_ = true;
        }
        publishFromGroundTruth(msg->pose[i], msg->twist[i]);
        return;
      }
    }
  }

  void publishFromGroundTruth(const geometry_msgs::Pose &pose,
                              const geometry_msgs::Twist &twist)
  {
    const ros::Time stamp = ros::Time::now();
    if (stamp.isZero())
      return;
    // Suppress duplicate-stamp broadcasts.
    //
    // Under Gazebo lockstep the sim clock only advances when physics steps,
    // while /gazebo/model_states can fire more than once per step. Two
    // callbacks then see the SAME ros::Time, and broadcasting odom->base_link
    // twice at one stamp makes tf2 complain on every occurrence:
    //   TF_REPEATED_DATA ignoring data with redundant timestamp for frame
    //   base_link (parent odom) at time X
    // tf2 drops the second copy anyway, so skipping it here loses nothing and
    // keeps the log readable. It also keeps the odom stream STRICTLY
    // increasing, which is a hard requirement of cartographer's
    // OptimizationProblem2D::AddOdometryData (map_by_time.h asserts
    // data.time > previous).
    //
    // Keep this dedup even though cartographer_sim.lua currently runs
    // use_odometry = false (the baseline's rplidar branch). Turning odometry
    // back on is a live experiment, not a one-line config change - see the
    // "landmines" comment in simulation_slam.launch - and without this guard
    // the very first thing that experiment hits is that assert, i.e.
    // "F0924 map_by_time.h:43 Check failed" and exit code 134.
    if (stamp == last_stamp_)
      return;
    last_stamp_ = stamp;
    nav_msgs::Odometry odom;
    // The message must describe the SAME pose as the odom->base_link transform
    // broadcast below. They are two views of one measurement, so they apply the
    // same offset: the transform subtracts odom_offset_*, and so does this. That
    // offset is normally measured at startup by auto_odom_origin (the
    // odom_offset_* params in px4_sitl_field.launch are only a fallback for when
    // auto_odom_origin is false), which is what makes odom agree with
    // /mavros/local_position/pose instead of sitting at the Gazebo world origin.
    //
    // frame_id/child_frame_id were "world"/"ground_truth/base_link" while this
    // topic was a pure diagnostic. They are now "odom"/"base_link": these are
    // the names that exist in the TF tree, and cartographer_ros resolves the
    // odometry frame through TF - a frame id that is not in the tree would make
    // the sample silently unusable.
    geometry_msgs::Pose odom_pose = pose;
    odom_pose.position.x -= odom_offset_x_;
    odom_pose.position.y -= odom_offset_y_;
    odom_pose.position.z -= odom_offset_z_;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = odom_pose;
    tf::Transform transform;
    tf::poseMsgToTF(pose, transform);

    // Gazebo twists are in world axes; Odometry twists use child axes.
    const tf::Matrix3x3 inverse = transform.getBasis().transpose();
    tf::vector3TFToMsg(inverse * tf::Vector3(
        twist.linear.x, twist.linear.y, twist.linear.z),
        odom.twist.twist.linear);
    tf::vector3TFToMsg(inverse * tf::Vector3(
        twist.angular.x, twist.angular.y, twist.angular.z),
        odom.twist.twist.angular);
    odom_pub_.publish(odom);
    // Broadcast odom->base_link straight from the Gazebo ground truth.
    // Cartographer runs with published_frame=odom / provide_odom_frame=false,
    // so this is the only thing that defines odom. Every consumer of odom
    // (mavros local position, flight setpoints in FRAME_LOCAL_NED, route
    // waypoints, costmaps) therefore agrees with the real vehicle, and any SLAM
    // error stays confined to map->odom where it belongs.
    //
    // Orientation: the ground-truth attitude is published as-is, INCLUDING
    // roll and pitch. An earlier revision flattened this to yaw-only out of
    // fear that a tilted tracking frame would confuse the scan matcher, but
    // that flattening is itself a bug here: /mavros/local_position/odom and
    // /mavros/local_position/pose both carry the full attitude, so a
    // yaw-only odom->base_link disagrees with them by exactly the roll/pitch,
    // and navigation_2d (which consumes TF, not mavros) would then see a
    // vehicle orientation that no flight-controller frame agrees with.
    //
    // The rotation channel is instead handled in cartographer_sim.lua by
    // using an IMU (so the 2D trajectory builder has a rotation model) and by
    // keep the odom->base_link frame *unprojected* only where cartographer
    // consumes it. If map->odom is still seen rotating, the diagnosis belongs
    // there, not here.
    if (publish_odom_tf_)
    {
      tf::Transform odom_to_base = transform;
      const tf::Vector3 origin = odom_to_base.getOrigin();
      odom_to_base.setOrigin(tf::Vector3(origin.x() - odom_offset_x_,
                                         origin.y() - odom_offset_y_,
                                         origin.z() - odom_offset_z_));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(odom_to_base, stamp, "odom", "base_link"));
    }
    // Payload markers are published from HERE, not from the sync timer, on
    // purpose: this callback is the thread that writes odom_offset_*, so reading
    // the offset from here cannot race with its one-time measurement.
    publishPayloadMarkers();
    // Does Gazebo agree that the drone is where Cartographer thinks it is?
    // If iris stays parked at the origin while the map pose runs away, the
    // drift is in SLAM, not in the physics.
    ROS_INFO_THROTTLE(1.0,
                      "Rig check: iris=(%.2f, %.2f, %.2f) "
                      "rig_cmd=(%.2f, %.2f) rig_actual=(%.2f, %.2f)",
                      vehicle_pose_.position.x, vehicle_pose_.position.y,
                      vehicle_pose_.position.z,
                      have_rig_target_ ? last_rig_target_.position.x : 0.0,
                      have_rig_target_ ? last_rig_target_.position.y : 0.0,
                      have_rig_actual_ ? last_rig_actual_.position.x : 0.0,
                      have_rig_actual_ ? last_rig_actual_.position.y : 0.0);
    // NOTE: the rig is NOT synced here. See the comment on sync_timer_ in the
    // constructor: doing the blocking SetModelState call from this callback
    // stalls the odom->base_link broadcast. The timer picks the pose up.
  }

  // Runs on its own thread/period. Pulls the newest ground-truth pose and moves
  // the sensor rig onto it, so the blocking SetModelState call can never hold
  // up the odom transform or the model_states callback.
  void syncTimerCallback(const ros::TimerEvent &)
  {
    geometry_msgs::Pose target;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!have_vehicle_pose_)
        return;
      target = vehicle_pose_;
    }
    syncRigToVehiclePose(target);
    releasePendingPayloads(target);
  }

  // The real servo changes one camera between front and down. The SITL rig
  // stays down-facing for ground-card validation and acknowledges both
  // commands so the mission state machine keeps its normal timing.
  //
  // It also fakes the one physical consequence the mission cannot see: a drop
  // command actually puts a payload into the world. See payload_on_drop in the
  // constructor for why this is a spawn rather than a released joint.
  void gpioCallback(const std_msgs::String::ConstPtr &msg)
  {
    std_msgs::String ack;
    ack.data = msg->data + "_DONE";
    gpio_ack_pub_.publish(ack);

    const bool fire = (msg->data == "DROP_FIRE");
    const bool rescue = (msg->data == "DROP_RESCUE");
    if (!fire && !rescue)
      return;
    // Named before the message so the argument list carries no string literal:
    // tools/fmt_stub_diff.py joins a format string onto the next line only when
    // that line holds a single literal, and a literal argument breaks the join,
    // which silently stops the stub from checking this message.
    const char *kind = fire ? "fire" : "rescue";
    if (!payload_on_drop_)
    {
      ROS_WARN("GPIO %s acked, but payload_on_drop is false: the mission will "
               "believe a payload was released and nothing will appear in "
               "Gazebo", msg->data.c_str());
      return;
    }
    // Queue only. The spawn happens on the timer thread: this callback runs on
    // the spinner thread that also feeds odom->base_link, and a blocking service
    // call here is exactly the stall that sync_timer_ was created to remove.
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      pending_drops_.push_back(kind);
    }
    ROS_INFO("GPIO %s acked; a '%s' payload will be released from the next "
             "ground-truth pose", msg->data.c_str(), kind);
  }

  void releasePendingPayloads(const geometry_msgs::Pose &vehicle)
  {
    std::deque<std::string> pending;
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      if (pending_drops_.empty())
        return;
      pending.swap(pending_drops_);
    }
    for (std::size_t i = 0; i < pending.size(); ++i)
      spawnPayload(pending[i], vehicle);
  }

  void spawnPayload(const std::string &kind, const geometry_msgs::Pose &vehicle)
  {
    if (!spawn_client_.exists())
    {
      ROS_WARN("Service %s is not available, so the %s payload cannot be "
               "spawned; the drop is still real as far as the mission is "
               "concerned", spawn_service_.c_str(), kind.c_str());
      return;
    }

    geometry_msgs::Pose pose = vehicle;
    pose.position.z -= payload_release_depth_;
    if (pose.position.z < 0.05)
    {
      // Spawning below the floor does not fail - the ground plane pushes the
      // model back out somewhere unpredictable - so clamp instead.
      pose.position.z = 0.05;
    }

    char name[48];
    std::snprintf(name, sizeof(name), "payload_%s_%d", kind.c_str(),
                  ++payload_serial_);

    const std::string sdf = payloadSdf(name, kind);
    if (sdf.empty())
    {
      return;  // payloadSdf already reported why
    }

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = name;
    srv.request.model_xml = sdf;
    srv.request.reference_frame = "world";
    srv.request.initial_pose = pose;
    if (!spawn_client_.call(srv))
    {
      ROS_WARN("Call to %s failed for payload '%s'", spawn_service_.c_str(), name);
      return;
    }
    if (!srv.response.success)
    {
      // A duplicate name is the usual cause ("model already exists") after a
      // node restart inside a still-running Gazebo.
      ROS_WARN("Could not spawn payload '%s': %s", name,
               srv.response.status_message.empty()
                   ? "(no message)"
                   : srv.response.status_message.c_str());
      return;
    }
    ROS_INFO("Payload '%s' released at world (%.2f, %.2f, %.2f), %.2f m below "
             "the body; gravity takes it from here",
             name, pose.position.x, pose.position.y, pose.position.z,
             payload_release_depth_);
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      PayloadRecord record;
      record.name = name;
      record.kind = kind;
      record.pose = pose;
      payloads_.push_back(record);
    }
  }

  // Follow every payload we spawned, so the marker is where the box actually is.
  // Publishing only the release pose would leave a marker hanging in mid-air
  // while the real box falls, which is worse than no marker at all.
  void updatePayloadPoses(const gazebo_msgs::ModelStates &msg)
  {
    std::lock_guard<std::mutex> lock(payload_mutex_);
    for (std::size_t i = 0; i < payloads_.size(); ++i)
    {
      for (std::size_t j = 0; j < msg.name.size() && j < msg.pose.size(); ++j)
      {
        if (msg.name[j] == payloads_[i].name)
        {
          payloads_[i].pose = msg.pose[j];
          break;
        }
      }
    }
  }

  void publishPayloadMarkers()
  {
    std::vector<PayloadRecord> snapshot;
    {
      std::lock_guard<std::mutex> lock(payload_mutex_);
      if (payloads_.empty())
        return;
      snapshot = payloads_;
    }
    const ros::Time stamp = ros::Time::now();
    visualization_msgs::MarkerArray array;
    array.markers.reserve(snapshot.size());
    for (std::size_t i = 0; i < snapshot.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = stamp;
      // odom, not world: the fixed frame in this project's Foxglove layout is
      // odom, and odom is the field frame every waypoint is written in.
      marker.header.frame_id = "odom";
      marker.ns = "payloads";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = snapshot[i].pose;
      // Same conversion as odom->base_link: a pure translation, so the
      // orientation of the box carries over unchanged.
      marker.pose.position.x -= odom_offset_x_;
      marker.pose.position.y -= odom_offset_y_;
      marker.pose.position.z -= odom_offset_z_;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.08f;
      const bool fire = (snapshot[i].kind == "fire");
      marker.color.r = fire ? 0.85f : 0.10f;
      marker.color.g = fire ? 0.12f : 0.42f;
      marker.color.b = fire ? 0.10f : 0.85f;
      marker.color.a = 1.0f;
      array.markers.push_back(marker);
    }
    payload_marker_pub_.publish(array);
  }

  // A 80 mm cube, 50 g, coloured per material so the two drops stay
  // distinguishable on screen. <static>false</static> is what makes it fall:
  // a static payload spawns perfectly and then hangs in mid-air, which looks
  // like a release that did not happen. tools/check_config.py asserts it.
  static std::string payloadSdf(const std::string &name, const std::string &kind)
  {
    const char *color = (kind == "fire") ? "0.85 0.12 0.10 1"
                                         : "0.10 0.42 0.85 1";
    char buffer[1600];
    const int written = std::snprintf(
        buffer, sizeof(buffer),
        "<sdf version='1.6'>\n"
        "  <model name='%s'>\n"
        "    <static>false</static>\n"
        "    <link name='body'>\n"
        "      <inertial>\n"
        "        <mass>0.05</mass>\n"
        "        <inertia>\n"
        "          <ixx>5.4e-5</ixx><ixy>0</ixy><ixz>0</ixz>\n"
        "          <iyy>5.4e-5</iyy><iyz>0</iyz><izz>5.4e-5</izz>\n"
        "        </inertia>\n"
        "      </inertial>\n"
        "      <collision name='collision'>\n"
        "        <geometry><box><size>0.08 0.08 0.08</size></box></geometry>\n"
        "        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode>"
        "</friction></surface>\n"
        "      </collision>\n"
        "      <visual name='visual'>\n"
        "        <geometry><box><size>0.08 0.08 0.08</size></box></geometry>\n"
        "        <material>\n"
        "          <ambient>%s</ambient><diffuse>%s</diffuse>\n"
        "        </material>\n"
        "      </visual>\n"
        "    </link>\n"
        "  </model>\n"
        "</sdf>\n",
        name.c_str(), color, color);
    if (written <= 0 || written >= static_cast<int>(sizeof(buffer)))
    {
      ROS_ERROR("Payload SDF for '%s' did not fit in %d bytes; spawn skipped",
                name.c_str(), static_cast<int>(sizeof(buffer)));
      return std::string();
    }
    return std::string(buffer, written);
  }

  // Record the first PX4 local position. Gazebo spawns the iris with a small
  // vertical offset and its own world origin, while PX4 measures from wherever
  // it powered up. The difference is constant, so capturing it once lets the
  // odom->base_link transform be expressed in PX4's local frame instead of
  // Gazebo's. Without this the published odom would disagree with
  // /mavros/local_position/pose by that constant.
  void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (odom_origin_captured_ || !have_vehicle_pose_)
    {
      return;
    }
    if (!auto_odom_origin_)
    {
      odom_origin_captured_ = true;
      return;
    }
    // vehicle_pose_ is the Gazebo pose at (about) this same moment: both
    // arrive at startup and the vehicle is stationary before arming.
    //
    // The offset is what makes odom->base_link *relative to the odom origin*
    // rather than absolute Gazebo coordinates. Publishing absolute Gazebo
    // coordinates is what made map->odom come out as z = -1.6 m:
    //   map->odom(z=-1.6) + odom->base_link(z=+1.79) + base_to_laser(0.25)
    //   = 0.44, yet map->laser_link measures 0.25. Self-inconsistent.
    // Cartographer places the map origin on the floor at the start pose, while
    // Gazebo's origin sits 1.79 m below where the drone happens to begin, so
    // the two origins differ by a constant. Subtracting that constant here is
    // what aligns them; it also must match what /mavros/local_position/pose
    // reports, which starts at zero for the same reason.
    const tf::Vector3 level_origin = computeLevelOrigin(vehicle_pose_);
    odom_offset_x_ = level_origin.x() - msg->pose.position.x;
    odom_offset_y_ = level_origin.y() - msg->pose.position.y;
    odom_offset_z_ = level_origin.z() - msg->pose.position.z;
    odom_origin_captured_ = true;
    ROS_INFO("odom origin aligned to PX4 local frame: offset=(%.3f, %.3f, %.3f)",
             odom_offset_x_, odom_offset_y_, odom_offset_z_);
  }

  // The position that odom->base_link publishes for a raw Gazebo pose.
  static tf::Vector3 computeLevelOrigin(const geometry_msgs::Pose &pose)
  {
    tf::Transform raw;
    tf::poseMsgToTF(pose, raw);
    return raw.getOrigin();
  }

  // Move the lidar/camera rig onto the pose that was just published. Doing this
  // in the pose callback instead of a separate timer keeps the rig's physical
  // position and the stamped odom frame describing the same instant; a 20 Hz
  // follow timer always trailed the newest pose by up to one period.
  //
  // Do not try to synthesise an IMU here. Cartographer hard-fails
  // (sensor_bridge.cc) unless every IMU field is present with a non-negative
  // covariance, and driving an extra topic from /gazebo/model_states dropped
  // the IMU rate from 50 Hz to 17-22 Hz. The vehicle uses /mavros/imu/data.
  void syncRigToVehiclePose(const geometry_msgs::Pose &pose)
  {
    if (!model_state_client_.exists())
    {
      ROS_WARN_THROTTLE(5.0, "Service %s is not available; rig cannot follow the drone",
                        model_state_service_.c_str());
      return;
    }

    gazebo_msgs::SetModelState request;
    request.request.model_state.model_name = sensor_model_;
    request.request.model_state.pose = pose;
    request.request.model_state.reference_frame = "world";
    if (!model_state_client_.call(request))
    {
      ROS_WARN_THROTTLE(5.0, "Call to %s failed for '%s'",
                        model_state_service_.c_str(), sensor_model_.c_str());
      return;
    }
    last_rig_target_ = pose;
    have_rig_target_ = true;
    if (!request.response.success)
    {
      // A silently empty status_message usually means the target model is
      // marked <kinematic>true in its SDF; Gazebo refuses to move those.
      ROS_WARN_THROTTLE(5.0, "Unable to synchronize sensor rig '%s': %s",
                        sensor_model_.c_str(),
                        request.response.status_message.empty()
                            ? "(no message; is the model <kinematic>true in its SDF?)"
                            : request.response.status_message.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber model_states_sub_;
  ros::Subscriber gpio_sub_;
  ros::Subscriber mavros_pose_sub_;
  ros::Publisher gpio_ack_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher payload_marker_pub_;
  ros::ServiceClient model_state_client_;
  ros::ServiceClient spawn_client_;
  ros::Timer sync_timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  // Guards vehicle_pose_ / have_vehicle_pose_: written by the model_states
  // callback, read by the (independent) sync timer.
  std::mutex pose_mutex_;
  // Guards pending_drops_: written by gpioCallback (spinner thread), drained by
  // the sync timer. Separate from pose_mutex_ on purpose - a release must never
  // make the odom path wait.
  std::mutex payload_mutex_;
  std::deque<std::string> pending_drops_;
  std::vector<PayloadRecord> payloads_;
  int payload_serial_ = 0;
  // Last stamp actually broadcast on odom->base_link. Used to drop duplicate
  // stamps produced by lockstep clock resolution (see publishFromGroundTruth).
  ros::Time last_stamp_;

  geometry_msgs::Pose vehicle_pose_;
  geometry_msgs::Pose last_rig_target_;
  geometry_msgs::Pose last_rig_actual_;
  std::string vehicle_model_;
  std::string sensor_model_;
  std::string model_state_service_;
  std::string spawn_service_;
  bool have_vehicle_pose_ = false;
  bool have_rig_target_ = false;
  bool have_rig_actual_ = false;
  bool publish_odom_tf_ = true;
  bool auto_odom_origin_ = true;
  bool odom_origin_captured_ = false;
  bool payload_on_drop_ = true;
  double odom_offset_x_ = 0.0;
  double odom_offset_y_ = 0.0;
  double odom_offset_z_ = 0.0;
  double update_rate_ = 20.0;
  double payload_release_depth_ = 0.30;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_sensor_rig");
  Px4SensorRig rig;
  ros::spin();
  return 0;
}
