-- Cartographer 2D，抄自去年的基线。
--
-- 基线位置：C:/Users/expload/Desktop/fsdownload/catkin_ws_bak/src，
-- location/param/ROSSLAM/cartographer_2Dlidar_mapping_location_params.lua 与
-- location/launch/cartographer_2d.launch。那套软件栈是在真机上跑完整场赛题任务的
-- 配置（Cartographer + DWA），也是本工程里唯一有实战记录的配置。
--
-- 这次重写的方针：2026-09-24 排障期间手工臆造出来的每一项覆盖，全部删除。
-- 剩下的是基线自己的设置加上 cartographer 的出厂默认值。例外项都列了理由；
-- 每一处例外都是**抄不过来**的差异，而不是意见分歧。
--
-- 本次重写删掉的内容（全是我们自己发明的，没有一个来自基线）：
--   TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.0
--   TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matcher.linear_search_window = 0.15
--   TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = ...
--   TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025
--   TRAJECTORY_BUILDER_2D.motion_filter.*
--   POSE_GRAPH.constraint_builder.min_score = 0.72
-- 其中 motion_filter 那项的副作用最明确：0.015 m / 0.5 s 往一个 submap 里塞进
-- 的节点数大约是出厂值 0.2 m / 5 s 的 13 倍，而那次把地图往北拖了 2.2~3.0 m 的
-- 假回环闭合，恰恰是以一批约 70 个节点、全都声称同一个 2.24 m 平移的形式到来的
-- —— 这正是节点流过密才可能出现的情况。
--
-- min_score 恢复成出厂值 0.55 是唯一需要盯住的一处回退。
-- 基线也从未设过它，所以 0.55 是诚实的抄写；但本场地的障碍是**部分镜像成对**出现的
-- （圆柱在 x 0.70/2.10，方块在 x 0.70/2.10，而 x 0.70 那根横杆和 45° 那根横杆没有
-- 镜像对应物），而部分镜像恰恰是最糟的一种：重合点足够多、足以越过 0.55 的门限，
-- 又太少、不足以区分真假。如果地图仍然发生刚性跳变，该查的就是
-- constraint builder，而 min_score 是唯一的旋钮。
--
-- 例外一 —— 坐标系布局。上面那份基线文件用的是
--   published_frame = "base_link", provide_odom_frame = true
-- 也就是由 cartographer 自己发布 odom->base_link。我们做不到：本仿真里
-- odom->base_link 必须由 px4_sensor_rig_node 掌握，因为 odom 必须与
-- /mavros/local_position/odom 一致（PX4 靠它飞行、mission 也靠它测量），
-- 而在这条边上出现第二个发布者，就是早先那次
-- "Tf has two or more unconnected trees" 失败的原因。基线里的**另一份**配置
-- location/param/ROSSLAM/cartographer_2Dlidar_mapping_params.lua 用的正是这里保留的
-- 布局：
--   published_frame = "odom", provide_odom_frame = false
-- 所以这也同样是抄来的 —— 只是抄自它的兄弟文件，那份是为"由外部节点掌握
-- odom->base_link"的情形（在那边是 T265）而写的。
--
-- 例外二 —— 传感器量程，而这一项不是外观问题。基线保留了出厂值
-- min_range = 0、max_range = 30。它当时可以这么做：RPLidar S2 打得很远，
-- 超出比赛场地，所以一束什么都没打到的光，确实是真地离开了房间。
-- 本仿真里的雷达（gc_2026_sim/gazebo/models/px4_sensor_rig/model.sdf）
-- 声明 <range><min>0.10</min><max>3.50</max>，而 Gazebo 的 ray sensor 对一束
-- 什么都没打到的光会返回 maxRange —— 它不返回 inf，也不会省略这一束。于是
-- 发布的 LaserScan 对"未命中"和"恰好命中 3.50"给出**同样**的 range == 3.50，
-- 没有任何办法区分两者。cartographer_ros 把 echo <= msg.range_max 判为有效
-- HIT，所以每一束未命中都变成了一个恰好位于 3.50 m 处的真实障碍。
--
-- 在 4.05 m 的场地里，这相当于把整幅扫描的远侧大半都变成了障碍。从出发角点看，
-- 360 束里有 57 束（15.8%）其真实墙面位于 3.50 m 之外，最坏情况是方位角 -45°
-- 处 1.24 m 的误差，那里墙面实际在 4.74 m。比误差量级更糟的是它的**形状**：
-- 这些假回波构成一段以传感器为圆心、半径**恒定**的圆弧，而等半径圆弧对绕传感器
-- 的任何旋转都是不变的。把这样一个旋转不变的幻影和真实墙面一起喂给前端，
-- 是让 scan matching 在偏航上产生歧义的教科书做法，而这也正是我们见过地图跳变
-- 的成因之一。
--
-- max_range = 3.40 严格小于传感器上报的 range_max 3.50，所以那些束会落进
-- "missing" 分支、不再是命中。missing_data_ray_length 也取 3.40 而不是 3.50，
-- 因为它代表前端沿着一束无回波的光声称多远之内是**自由空间**
-- （cartographer_ros 把这类端点构造成 origin + length * direction）。一束光要落进
-- missing 分支，其真实交点必须超过 3.50 m，所以把自由空间刻到 3.40 永远不可能
-- 切穿墙面 —— 这是一对诚实且保守的取值。代价是 (3.40, 3.50] 这一段真实命中，
-- 即 0.10 m 的墙，最多影响一个 0.05 m 的 submap 栅格。
--
-- 本项目早先用的 6.0 按同样的论证就是错的，已经删掉；出厂的 5.0 也一样要删，
-- 它会直接把自由空间刻穿围墙。
--
-- 实现它的三个值（min_range / max_range / missing_data_ray_length）写在本文件的
-- **底部**，位于 include "trajectory_builder.lua" 之后。它们不能写在这里：
-- TRAJECTORY_BUILDER_2D 在那句 include 执行之前并不存在，提前索引它会让
-- cartographer 在启动时直接中止并报
--   lua_parameter_dictionary.cc:83 Check failed: status == 0
--   :82: attempt to index global 'TRAJECTORY_BUILDER_2D' (a nil value)
--
-- 例外三 —— 里程计关闭。两份基线文件在这里互相矛盾：rplidar 那一支
-- （默认支，location.launch 里 rplidar_location=true）跑的是
-- use_odometry=false / use_imu_data=true，T265 那一支则相反。
-- 关闭是这里**可知**安全的一侧。19:40 那次在保持上述坐标系布局的前提下把它打开，
-- 结果把 cartographer 的前端永久卡死在仿真时间 44.804 s：打开后里程计变成
-- 一个被收集的传感器，它的队列加入了 OrderedMultiQueue，而 rig 的 odom 是
-- 突发式到达的
--   "odom rate: 31.25 Hz 3.20e-02 s +/- -nan s (pulsed at 0.20% real time)"
-- 而同一时刻健康的 "scan rate: 10.00 Hz (pulsed at 99.00% real time)"。
-- 它的队列一旦饿死，下游就再也不会被派发，地图在 (-0.42, -2.89) 上冻住直到这轮结束，
-- 随后位姿发布器把剩下的日志时间全花在反复重试 odom->base_link 查询上 ——
-- 用那个已冻结的节点时间 44.804 s，而它早已超出了 tf 的 10 s 缓冲。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 例外二（完整推理见文件头）。这是 SITL 雷达的传感器物理，不是调参，
-- 也不是两份基线里的任何取值：比传感器自身 range_max 低 0.1 m。
-- SDF 声明 <range><max>3.50</max>，所以 Gazebo 对一束**什么都没打到**的光
-- 也发布 range == 3.50，而 cartographer_ros 把它算作有效命中；因此 max_range
-- 必须严格小于 3.50，才能把这些束推进 "missing" 分支。而
-- missing_data_ray_length（前端沿着一束无回波的光声称多远的自由空间）必须小于或
-- 等于 max_range，否则会把自由空间刻穿围墙。
-- 文件头记录了为什么 3.50（截至 2026-09-24 20:45 这里用的值）和 6.0（更早的一个值）
-- 都是错的，以及背后 57/360 的幻影束实测。
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 3.4
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.4
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
