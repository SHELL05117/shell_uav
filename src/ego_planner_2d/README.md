# ego_planner_2d (ROS1)

将 JackJu-HIT/Ego-Planner-2D-ROS2（算法源自浙大 FAST-Lab 的 ego-planner）移植到 **ROS1**，支持 **Melodic (18.04) / Noetic (20.04)**，面向地面机器人与定高飞行的无人机，作为**局部规划器**使用。

- 项目地址：https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2
- 原始算法：https://github.com/ZJU-FAST-Lab/ego-planner

核心算法层（B 样条优化器、2D A*、2D 栅格地图）为纯 C++ / Eigen 实现，直接复用上游代码；ROS 节点层按 ROS1 (roscpp) 重写，接口按“机载局部规划器”的形态设计：

- 订阅里程计位姿、全局参考路径（或目标点）、障碍物点云
- 周期性重规划（默认 5 Hz），轨迹碰撞 / 障碍更新 / 路径更新时自动触发
- 发布平滑避障的局部轨迹（x、y，z 填充当前飞行高度）

## 编译

    cd <catkin_ws>
    catkin_make          # 或 catkin build

依赖：roscpp、std_msgs、nav_msgs、geometry_msgs、sensor_msgs、visualization_msgs、tf2、Eigen3。C++14（Melodic/Noetic 默认）。

## 运行

    roslaunch ego_planner_2d ego_planner_2d.launch

启动后 RViz 中的交互方式（无需任何传感器即可测试）：

| RViz 工具 | 话题 | 作用 |
| --- | --- | --- |
| 2D Pose Estimate | /initialpose | 设置/重置当前位姿（无里程计时） |
| 2D Nav Goal | /goal | 设定目标点，自动生成直线参考路径 |
| Publish Point | /clicked_point | 放置一个障碍物 |

启停规划：

    rostopic pub /trigger_plan std_msgs/Bool "data: false"

## 话题接口

订阅（均可 remap）：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| odom | nav_msgs/Odometry | 当前位姿（优先级高于 /initialpose） |
| global_path | nav_msgs/Path | 全局参考路径（来自全局规划器） |
| goal | geometry_msgs/PoseStamped | 单目标点（生成直线参考路径） |
| obstacles | sensor_msgs/PointCloud2 | 障碍物点，须处于 map 坐标系 |
| scan | sensor_msgs/LaserScan | 2D 激光雷达（如 RPLIDAR），自动经 TF 变换到 map 坐标系 |
| clicked_point | geometry_msgs/PointStamped | RViz 交互放置障碍物 |
| initialpose | geometry_msgs/PoseWithCovarianceStamped | RViz 交互重置位姿 |
| trigger_plan | std_msgs/Bool | 规划启停 |

发布（**全部为 latched 话题**，新加入的订阅者立刻收到最后一帧）：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| visual_global_path | nav_msgs/Path | 参考路径（直线插值或来自 global_path） |
| visual_local_trajectory | nav_msgs/Path | B 样条 / A* 局部轨迹 |
| trajectories | visualization_msgs/MarkerArray | A* 回弹搜索路径（LINE_STRIP） |
| visual_obstacles | sensor_msgs/PointCloud2 | 原始障碍物 |
| visual_local_obstacles | sensor_msgs/PointCloud2 | 栅格内障碍物 |

三条路径通道（global_path / local_trajectory / trajectories）**画在同一高度** z = `output_traj_height`（该参数 < 0 时用当前位姿 z），全部以 `global_frame` 为 `frame_id`。

### 在 Foxglove 中查看

新建 3D 面板后：

1. **Fixed frame 设为 `global_frame` 的值**（本项目 `navigation_2d.launch` 里是 `odom`）。这是最容易踩的一步：话题全部以该 frame 打戳，如果面板的 fixed frame 是别的坐标系（例如 `map`）而 `map → odom` 这条 TF 不存在或已过期，3D 面板会**静默丢弃**这些消息——节点侧一切正常，`rostopic hz` 也正常，就是画面里什么都没有。
2. 添加话题面板：`/visual_global_path`、`/visual_local_trajectory`、`/trajectories`，以及 `/visual_obstacles`。
3. 若 `nav_msgs/Path` 在你的 Foxglove 版本里不渲染，用 `/trajectories`（MarkerArray）作为替代——它是标准 Marker，各版本都支持。
4. 判断"节点没发"还是"面板没画"：看节点日志里每 5 s 一行的
   `EGO visualisation: global_path=N pts local_traj=M pts a_star=K paths frame=... z=...`。
   N/M 为 0 说明规划侧确实没有内容，非 0 而画面空白则是面板/frame 配置问题。

注意：`/trigger_plan` 为 false（规划关闭）时，节点**停止重规划但保留最后一帧可视化**，所以画面上会留一条不再更新的轨迹——这是"已停止"的正常表现，不是还在规划。

## 参数

下表是节点的代码默认值。`navigation_2d.launch` 会覆盖其中多项
（`map_x_size/map_y_size=8`、`map_inflate_value=0.35`、`sensing_range=3.5`、
`path_sample_interval=0.20`、`local_window_points=20`、`min_traj_points=5`、
`min_ref_points=8`、`global_frame=odom`、`pose_source=tf`、
`output_traj_height=$(arg fly_height)`），实际以 launch 为准。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| max_vel / max_acc / max_jerk | 2.0 / 3.0 / 4.0 | 轨迹物理限值 |
| planning_rate | 5.0 | 规划频率 Hz |
| map_resolution | 0.1 | 局部栅格分辨率 m |
| map_x_size / map_y_size | 50 / 50 | 局部地图尺寸 m（以当前位姿为中心） |
| map_inflate_value | 0.5 | 障碍物膨胀半径 m |
| sensing_range | 10.0 | 障碍物纳入范围 m |
| path_sample_interval | 0.3 | 参考路径重采样间隔 m |
| local_window_points | 50 | 局部规划窗口点数 |
| min_traj_points | 10 | 低于该点数触发重规划 |
| min_ref_points | 8 | 参考路径最少点数（优化器需要 > 4，不足时按同跨度加密重采样） |
| obstacle_height_band | 1.0 | 障碍物高度过滤带（以当前高度为中心），<=0 关闭 |
| obstacle_accumulate | true | true: 障碍物累积；false: 每帧点云替换（适合实时传感器） |
| output_traj_height | -1 | 输出轨迹的 z 高度；-1 用当前位姿 z（2D 定位后端 z 恒 0，建议显式设定高值） |
| global_frame | map | 发布消息的 frame_id |

## 传感器接入

规划器本身与传感器无关，只需要三类输入：位姿（odom 话题或 TF，由 pose_source 选择）、参考路径（global_path 或 goal）、障碍物（obstacles 点云 或 scan 激光）。

### RPLIDAR S2（2D 激光，推荐）

支持。标准链路：

    rplidar_ros 驱动（新版 SDK 支持 S2，输出 /scan，frame 通常为 laser）
      -> 本节点直接订阅 scan，自动完成：
         1. TF 变换 laser -> map（需定位模块提供 map->odom->base_link->laser 的 TF）
         2. 高度带过滤、感知范围裁剪、栅格去重
         3. 障碍物更新触发重规划

注意：

- TF 用的是最新可用变换（ros::Time(0)），低速运动下误差可忽略。
- 激光扫描平面应接近飞行高度；若点被高度带过滤掉（看不到障碍物），调大 obstacle_height_band 或设为 <=0 关闭过滤。
- 实时激光建议 obstacle_accumulate=false（每帧替换，障碍物能“消失”）；true 时旧障碍只增不减。
- 默认订阅话题名 scan 即 rplidar_ros 的默认输出，无需 remap。

### USB 单目相机

仅靠单目相机**无法提供障碍物输入**（无深度信息）。可行/不可行：

- 可行：单目+IMU 跑 VIO（如 VINS-Mono）为 odom 提供位姿 —— 但没有避障感知。
- 不可行：纯单目直接给 obstacles（除非上单目深度估计模型，噪声大，不建议机载使用）。
- 没有障碍物输入时规划器仍可运行（沿参考路径跟踪），RViz 手动放障碍可做台架测试。
- 需要视觉避障请用深度相机（D435i/D455 等，输出点云经变换后接 obstacles）或双目。

## 整机集成示例：Jetson Nano + RPLIDAR S2 + TFmini + Pixhawk 6C + USB 单目相机

各部件分工：

| 部件 | 角色 | 与本规划器的关系 |
| --- | --- | --- |
| Jetson Nano | 机载计算 | 运行本规划器 + 定位 + 识别 |
| RPLIDAR S2 | 定位 + 避障感知 | scan 同时喂 cartographer（定位）与规划器（障碍物） |
| Pixhawk 6C | 飞控 | 经 mavros 提供里程计/接收控制指令 |
| TFmini | 高度传感器（朝下接飞控 RNGFND） | 与 2D 规划无关，定高/仿地由飞控负责 |
| USB 单目相机 | 目标识别 | 识别结果换算成 map 系目标点发布到 goal |

数据流：

    RPLIDAR S2 --> rplidar_ros --> /scan --+--> cartographer --> TF: map->odom->base_link->laser
                                             |
                                             +--> ego_planner_2d (scan -> 障碍物, TF 取位姿)
    相机识别节点 --> 目标点 --> ego_planner_2d /goal
    ego_planner_2d --> /visual_local_trajectory --> 轨迹跟踪节点 --> mavros --> Pixhawk

### 第 1 步：定位（cartographer，GPS 拒止环境推荐）

用 RPLIDAR 跑 2D cartographer，输出 map->odom->base_link TF（provide_odom_frame=true 或由 mavros 提供 odom->base_link）。规划器侧开启 TF 位姿模式，二者共用同一条 TF 链，位姿与 scan 变换天然一致：

    <node pkg="ego_planner_2d" type="ego_planner_2d_node" name="ego_planner_2d" output="screen">
      <param name="pose_source" value="tf"/>
      <param name="base_frame" value="base_link"/>
      <param name="obstacle_accumulate" value="false"/>
    </node>

与 cartographer 集成的三个注意点：

- **轨迹 z 为 0**：2D cartographer 的 TF 没有高度（z 恒 0），规划器输出轨迹的 z 也因此为 0。定高由飞控负责；若希望轨迹携带指令高度（方便跟踪器直接用），设置参数 output_traj_height（如 1.5，单位米；默认 -1 表示用位姿 z）。
- **frame 名对齐**：规划器的 base_frame 参数要与你 cartographer 配置（lua）里的 tracking_frame / published_frame 一致（base_link / base_footprint 等）。
- **高度带与雷达安装高度**：scan 点变换后的 z 等于雷达相对 base_link 的安装高度。若雷达架高超过 obstacle_height_band 的一半（默认带宽 1.0m 即 0.5m），点会被误过滤，届时调大该参数或设为 <=0 关闭。

户外 GPS 环境可以不用 cartographer：开启 mavros local_position 插件的 TF 广播（local_position/tf/send=true），同样用 pose_source=tf。

### 第 2 步：障碍物（已内置，零配置）

rplidar_ros 默认输出 /scan，规划器默认订阅 scan 并自动 TF 变换到 map。注意激光扫描平面要接近飞行高度（2D 规划只感知该平面上的障碍物）。

### 第 3 步：目标点（单目相机 + TFmini）

相机仅用于识别时，把识别结果换算成 map 系目标点发布到 goal（geometry_msgs/PoseStamped）：

- 目标方位：相机检测框中心相对图像中心的偏移 + 相机安装角 -> 目标方位角
- 目标距离：若 TFmini 与相机同向安装，可直接读 TFmini 距离（需规划器与识别共用同一时刻的位姿）；或按已知目标尺寸估算
- 目标点 = 当前位姿 + 距离 * [cos(方位角), sin(方位角)]

规划器收到 goal 后自动生成直线参考路径并开始避障规划。远距离/复杂环境可换成全局规划器（move_base + cartographer 建的图）接 global_path。

### 第 4 步：轨迹跟踪（需要自备或自写控制节点）

规划器输出的是轨迹（visual_local_trajectory，z 为当前高度），机载控制需要一层跟踪器把它转成 mavros 设定点。参考做法（纯跟踪）：

- 订阅 visual_local_trajectory，取距当前位置前瞻距离 Ld 的轨迹点
- 以固定频率（PX4 OFFBOARD 建议 >= 20Hz）发布 /mavros/setpoint_position/local（PoseStamped，z 取当前定高值）
- 到达终点附近停止发送或切回悬停
- ArduPilot 用 GUIDED 模式 + setpoint 话题；PX4 用 OFFBOARD

注意：跟踪器属于飞行安全关键代码，增益/前瞻距离/高度来源务必实测调参后再投入飞行。

### 安全提醒

- 先台架（RViz 手动放障碍）再室内系留再外场
- 2D 规划只避障激光平面上的障碍物，低于/高于扫描平面的障碍（悬空线缆、桌底）不可见
- planning_rate 默认 5Hz，Jetson Nano 上单次重规划耗时毫秒级，余量充足；同机跑 cartographer + 识别时注意 CPU 分配

## 话题级集成示例（点云障碍物输入）

    <node pkg="ego_planner_2d" type="ego_planner_2d_node" name="ego_planner_2d" output="screen">
      <remap from="odom" to="/mavros/local_position/odom"/>
      <remap from="global_path" to="/move_base/NavfnROS/plan"/>
      <remap from="obstacles" to="/cloud_obstacles_map_frame"/>
      <param name="obstacle_accumulate" value="false"/>
    </node>

注意：obstacles 点云必须已在 map 坐标系下（可在感知节点中用 tf 变换后再发布），本包不做坐标变换。

## 相对上游 (Ego-Planner-2D-ROS2) 的改动

除 ROS2 → ROS1 的节点层重写外，修复了上游的几个问题：

1. **A* 路径索引越界（崩溃修复）**：bspline_optimizer.cpp 中三处 while 循环先取 a_star_pathes[i][Astar_id] 再判边界，索引可为 -1 或 size()，属于未定义行为（实测同场景下原版会段错误）。已在索引变化后立即做边界检查。
2. **优化器物理限值未传递（绕障失败修复）**：上游 setParam() 把优化器内部限值写死为 max_vel=1.0 / max_acc=0.5，与 initParam() 传入的限值（用于时间分配）互相矛盾，导致障碍物正对路径时优化几乎必然失败。新增 setPhysicalLimits() 将规划器限值传入优化器。
3. **refine 阶段控制点回写错误**：时间重分配后的优化结果写错变量，导致 refine 无效。
4. **热路径日志清理**：栅格地图每周期重置/设障碍/查询时的逐格打印在真实点云（上千点）下会刷爆控制台，已移除。
5. makePlan 从当前位姿出发（取自上游 develop 分支），并对过少的路径点做保护。
6. 新增 sensor_msgs/LaserScan 原生订阅（RPLIDAR 等 2D 激光直接接入，内置 TF 变换、高度带过滤、范围裁剪与栅格去重）。

## 已知局限（继承自上游 2D 移植）

- 障碍物**略微偏离**参考路径（约 0.1~0.3 m，落在膨胀区边缘附近）时，反弹优化可能反复振荡而失败；障碍物正对路径或偏离较远（>=0.5 m）时均可正常绕开。该行为与上游 ROS2 版本一致（经无 ROS 冒烟测试对比确认）。
- 膨胀半径较大（>=0.7 m）时绕行幅度需求大，优化成功率下降，建议 map_inflate_value 取 0.3~0.5。
- 无障碍物移除机制（栅格地图非概率型）：obstacle_accumulate=true 时障碍物只增不减，适合静态环境演示；动态环境请用 false（每帧替换）。
- 修复后经无 ROS 冒烟测试验证：无障碍/正对障碍/障碍墙/旁侧障碍场景均可输出避障轨迹。

## 致谢

- JackJu-HIT/Ego-Planner-2D-ROS2（MIT License）
- ZJU-FAST-Lab/ego-planner
