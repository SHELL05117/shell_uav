# Lichtblick 独立监看配置

准备日期：2026-10-13。**本目录已完成静态配置与XML核对，尚未上传Nano或启动ROS，未验收真实图像、地图、桥握手、网络延迟或无线覆盖。** 源码、模型、原归档保持不变；Windows客户端安装和界面布局由本次主任务另行记录。

这组配置把Lichtblick作为显示客户端。桥只订阅现有ROS数据，不启动任务、飞行控制、定位或舵机。相机预览是单独的可选入口，需在地面受控状态且摄像头未被其它节点占用时启用；它会访问V4L2相机并发布图像，但不改变舵机角度或发送飞行设定值。

## 文件与依赖

| 文件 | 内容 |
|---|---|
| [bridge_readonly.launch](bridge_readonly.launch) | include Nano原有Foxglove Bridge，禁用客户端发布、参数和服务能力，精确显示白名单 |
| [start_monitor_bridge.sh](start_monitor_bridge.sh) | 后续Nano运行桥的入口；核已有Master/版本/端口，拒绝同名桥和8765冲突，不自动杀进程 |
| [camera_preview.launch](camera_preview.launch) | 可选USB相机＋image_transport JPEG，仅两个感知/图像节点，不自动启动桥 |
| [validation_2026-10-13.json](validation_2026-10-13.json) | 原tar参数依据、XML静态检查、白名单与文件SHA；不代表ROS运行通过 |

直接从 `../../nanooutput_origin/Nano补充导出/2026-10-11/软件补档/02_opt_runtime.tar.gz` 只读核对原成员：

| tar成员（均位于opt/ros/melodic/share） | 归档版本/用途 |
|---|---|
| foxglove_bridge/package.xml、launch/foxglove_bridge.launch | 0.6.3；参数address/port/tls、capabilities、topic/client/param/service白名单全部存在 |
| usb_cam/package.xml | 0.3.7；相机驱动 |
| image_transport/package.xml | 1.11.13；republish |
| compressed_image_transport/package.xml | 1.9.6；JPEG插件 |

这些是保存的环境快照，并非10月13日已在Nano现场重新检查。脚本对当前bridge版本要求0.6.3；包/设备缺失时先核原因，不自动apt安装或替换机载软件。

## 白名单和显示数据

桥默认address=0.0.0.0、port=8765、tls=false；`capabilities=[]`、`client_topic_whitelist=[]`、`param_whitelist=[]`、`service_whitelist=[]`。显示白名单每条都用`^完整话题$`，未用`.*`或`/mavros/...`前缀通配；不暴露`/cmd_vel`、目标、投放命令、规划触发或飞控setpoint。

| 显示数据 | 精确话题 | 典型用途/来源 |
|---|---|---|
| 地图 | /map | nav_msgs/OccupancyGrid；Cartographer占用栅格节点产生，桥本身不生成地图 |
| 激光 | /scan | sensor_msgs/LaserScan；RPLidar数据 |
| 坐标变换 | /tf、/tf_static | 3D显示数据间的frame关系 |
| 全局/局部路径 | /visual_global_path、/visual_local_trajectory | nav_msgs/Path；现有EGO节点 |
| 搜索轨迹 | /trajectories | visualization_msgs/MarkerArray；EGO |
| 障碍点云 | /visual_obstacles、/visual_local_obstacles | sensor_msgs/PointCloud2；EGO局部障碍，不是完整三维重建 |
| 位置和模式 | /mavros/local_position/pose、/mavros/local_position/odom、/mavros/state | 只订阅飞控回传状态 |
| 检测 | /ssd_detection/detections、/mission/vision/detection | 自定义消息；Raw/Plot查看字段，不保证任意Image面板自动叠框 |
| 机构通知 | /servo_ready、/gpio_ack | 原节点的软件初始化/动作计时通知，并非物理到位传感器 |
| 一路JPEG | 下表三选一 | sensor_msgs/CompressedImage；默认独立预览 |

| jpeg_source | 唯一允许的JPEG话题 | 使用条件 |
|---|---|---|
| preview（默认） | /monitor/camera/image/compressed | 可选camera_preview已经单独启动 |
| mission | /mission/video/compressed | 现有任务视觉数据源已由另一次受控操作启动；本目录不启动任务来制造这个话题 |
| ssd | /ssd_detection/debug_image/compressed | 现有SSD独立检测数据源已受控运行 |

选择由launch固定映射，非法值会拒绝配置解析。桥只允许上述一种JPEG；白名单仅限制可见范围，实际带宽仍由客户端打开的面板/订阅决定。相机原始图像和CameraInfo留在Nano本地，不在桥白名单内。

在Lichtblick 3D面板通常选Display frame=`map`；实际以/map.header.frame_id为准。当前EGO路径打odom戳，激光为laser_link，需要有效TF连接到显示帧。若只有相机预览，没有map/TF，则只用Image面板，不凭空建立map或camera_link外参。预览图像frame_id=`camera_link`，压缩插件保留输入header；没有相机标定/TF时可二维看图，不能认定相机在三维地图中定位正确。

## 后续上传和Nano启动

将本目录保存至Nano新目录 `/home/jcfh/monitor_display/`；运行只需两份launch和启动脚本，布局供Windows使用。本日尚未执行上传，无需编译这个目录。SHELL确认Nano返修暂不能开机，完整上传、复用已有相机和故障排查步骤见[接续指南](../项目资料/Lichtblick安装与回传接续_2026-10-13.md)。

先由SHELL在Nano SSH窗口核对现有环境/数据源：

```bash
source /opt/ros/melodic/setup.bash
source /home/jcfh/catkin_ws/devel/setup.bash
rosversion foxglove_bridge
rosnode list
rostopic list
ss -ltn 'sport = :8765'
```

本目录不自动启动location/navigation。若/map、/scan尚未产生，连接桥也不会出现地图。原location入口使用传感器并向飞控发布外部位姿，原navigation入口初始化真实舵机并发送setpoint；它们需要单独的受控启动安排。**不要为了消除“地图为空”直接启动navigation。**

在已有ROS Master和所需数据源受控运行时，只起桥：

```bash
bash /home/jcfh/monitor_display/start_monitor_bridge.sh preview
```

若已有经过批准的任务/SSD数据源，可在本目录桥关闭后改选 `mission` 或 `ssd`。这只选择显示话题，不启动检测或飞行。不要同时开两个桥；使用本入口时，原navigation的`start_foxglove_bridge`应保持false。遇同名桥/8765已监听，脚本退出，先查当前会话及能力配置，没有自动关闭它。

可选独立相机预览另开SSH窗口。若根本没有ROS Master，可由SHELL另开窗口执行`roscore`，其本身不启动无人机任务。确认 `/dev/video0` 存在且没有usb_cam/MissionVision占用后，按需运行：

```bash
source /opt/ros/melodic/setup.bash
roslaunch /home/jcfh/monitor_display/camera_preview.launch
```

默认640×480、5fps、V4L2 mjpeg/mmap，重编码JPEG质量50；可通过launch arg调分辨率/fps/设备，但摄像头是否接受这些值需实际日志和rostopic hz验证。相机方向保持当前物理姿态；该入口不会发CAMERA_DOWN/FRONT，因此机头前视相机不会自行变为下视。

压缩出口是 `/monitor/camera/image/compressed`；JPEG配置位于该话题参数命名空间，不在republish节点的私有空间。这个命名与header保留行为已对照[image_transport 1.11.13 republish源码](https://raw.githubusercontent.com/ros-perception/image_common/1.11.13/image_transport/src/republish.cpp)、[SimplePublisherPlugin源码](https://raw.githubusercontent.com/ros-perception/image_common/1.11.13/image_transport/include/image_transport/simple_publisher_plugin.h)、[compressed_image_transport 1.9.6源码](https://raw.githubusercontent.com/ros-perception/image_transport_plugins/1.9.6/compressed_image_transport/src/compressed_publisher.cpp)，仍未在Nano执行。

## Windows连接及后续验收

1. 首次地面USB链路可沿既有地址 `ws://192.168.55.1:8765`，但要重新核Nano当前IP和8765监听。Windows只需能到该WebSocket单一端口；ROS内部通信留在Nano/其既有Master网络，不要求在Windows安装ROS/WSL。
2. Lichtblick选择Foxglove WebSocket连接；本机已安装1.29.1并实际导入[monitor_layout.json](monitor_layout.json)，布局来自GUI导出的真实模板，字段核对官方版本源码，见[字段依据](layout_schema_notes.md)。Image选择上表唯一JPEG，3D显示map/scan/TF/路径；当前No data source，实时刷新尚未验收。
3. 核Nano上的桥实际参数：`rosparam get /foxglove_bridge/capabilities`及三份client/param/service白名单应为[]；再确认连接握手/客户端没有发布目标、参数修改、服务操作能力。能力限制只约束这一个桥，不约束其它ROS客户端、QGC、SSH或另一条宽权限桥。
4. 保存/map和/scan频率、TF年龄、位置/模式字段、JPEG频率与字节率、画面延迟和连接稳定性。只有数据出现且刷新才记录对应部分通过；截图不是全链正常或飞行验收。
5. 0.0.0.0/tls=false是本次局域网/USB监听配置，未提供认证/加密或公网部署；无线回传改实际机载局域网IP，并另测覆盖、断线和延迟。USB线连接不适合直接作为飞行回传链。

退出：在桥的SSH终端Ctrl+C只停止显示桥，已有数据源继续按各自会话管理。退出相机预览终端Ctrl+C释放该预览进程使用的设备。不要把其它终端的任务/定位节点一并退出当作显示断线处理。
