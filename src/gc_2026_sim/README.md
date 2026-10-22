# GC 2026 仿真

这个包只包含 2026 消防与救援任务的 Gazebo Classic / PX4 SITL 部分。它故意与 `navigation_2d` 分开，所以不需要拷到 Jetson 飞控计算机上。

## PX4 SITL + Cartographer

在 WSL 里，先编译工作空间并 source `devel/setup.bash`：

```bash
cd ~/catkin_ws
catkin build gc_2026_sim navigation_2d ego_planner_2d ssd_detection
source devel/setup.bash
bash src/gc_2026_sim/scripts/start_px4_slam.sh gui:=true
```

跑自主任务就是把 `auto_start` 打开：

```bash
bash src/gc_2026_sim/scripts/start_px4_slam.sh auto_start:=true
```

这个 launch 会起自建的 4 m x 4 m 场地、Iris、水平激光与朝下相机组成的传感器架、Cartographer、EGO 规划器及其跟随器、Foxglove Bridge，以及 `navigation_2d` 的任务节点。传感器架是机上 RPLidar S2 和舵机挂载 USB 相机的**仿真专用**替代品。

2026-10-17 之后 EGO（`ego_planner_2d` 加它的轨迹跟随器）是唯一的局部规划器：DWA、TEB、选择它们的 `planner_backend` 开关，以及它们依赖的 costmap YAML 都删了。跑与跑之间值得比的是完成情况、过点时间、最小障碍净空、规划失败次数，以及 PX4 的模式/失效保护消息。

只验证控制流的老 Gazebo 模型还留着，用于快速冒烟：

```bash
roslaunch gc_2026_sim gazebo_field.launch gui:=true
```

不要和 PX4 的 launch 并行跑 `make ... gazebo`。PX4 运行时不要重置 Gazebo 时间；要重来就整个 launch 重启。

## Gazebo 卡在 "Preparing your world"

world 里为任务区材质脚本和卡片贴图引用了 `model://gc_task_markers`。Gazebo 会到 `GAZEBO_MODEL_PATH` 里找这个名字，找不到的一律交给**在线模型库**——而 `start_px4_slam.sh` 用 `GAZEBO_MODEL_DATABASE_URI=""` 把它关掉了。结果就是世界永远加载不完，GUI 停在 "Preparing your world"，而 **ROS 日志里一个字都不报**。

`GAZEBO_MODEL_PATH` 由 `px4_gazebo_slam.launch` 和 `gazebo_field.launch` 各自导出，两者都把本包自带的模型目录（`gazebo/models`）前置进去。任何往 world 里加 `model://` 引用、或搬动 launch 文件的改动之后，跑一遍 `tools/check_config.py` 的第 11 项：它会顺着每个 world 的加载 launch **以及所有 include 它的 launch** 走一遍，链上没有任何一处导出这个路径就判失败。

如果你绕开本包、直接用 PX4 自带的 Gazebo 加载这个世界，`scripts/install_px4_field_world.sh` 会把 `gazebo/models/*` 一起拷到它安装 world 的位置旁边，这样 PX4 自己的模型目录就覆盖到了。
