# RK3399ProD／Ubuntu18.04部署依赖清单

> **最新视觉决定｜2026-10-23：** SHELL采用YOLO重新训练，放弃SSD／MobileNet及其对照路线，本机仅方案／记录。通用依赖已经七包构建与回环验证，当前RK不必为下载接互联网；NPU运行包可由电脑获取、核对后SSH传板。旧SSD后端及Python3 ABI章节保留为历史基线，不作为新路线前置，见[新方案](YOLO重训与RK离线部署方案_2026-10-23.md)。

[返回项目目录](../../logs/低空消防无人机工程日志.md) · [任务状态](任务状态.md) · [总技术指南](../技术文档.md)

工程记录日期：**2026-10-21**（依SHELL本轮约定）；本次反馈与资料核对日期：2026-10-22。

**2026-10-23接续：** Ubuntu18.04控制台启动已有照片及刷写日志，SHELL确认空间充足。可逐段执行的新入口见[联网与依赖安装路线](RK板联网与依赖安装路线_2026-10-23.md)，其中已核对FishROS快捷安装和官方final快照ARM64包索引；本清单保留依赖分类依据。尚未执行板端安装或构建，不能将下方参考命令解释为运行验收。

**[已确认／SHELL反馈] 当前项目平台为TB-RK3399ProD上的Ubuntu18.04。** 此说明更新此前“待到货、优先评估厂商Debian BSP”的接续前提；历史采购、评估和操作记录保留。助手未登录板端，未确认ROS发行版、已装包、系统镜像来源、NPU组合或现场是否另有修改版源码。

本清单依据当前 `src/` 七个包的 `CMakeLists.txt`、`package.xml`、launch及Python导入项。下方ROS包名**按Ubuntu18.04／ROS1 Melodic／ARM64**列出；Ubuntu版本本身不能证明板上已装Melodic。先用 `rosversion -d`、`dpkg --print-architecture` 核对，已有兼容安装无需重复安装。未在板端执行任何安装、构建、推理或硬件验收。

## 1. 先区分复制源码、构建和安装依赖

- **复制源码、联网、启动roslaunch或运行catkin_make，都不会替项目自动补齐外部库。** 当前工程未提供联网自安装脚本，也没有CMake远程拉取依赖的逻辑。`find_package`是在查找已经安装的库。
- 主动执行 `apt install` 后，apt会继续安装该软件包声明的间接依赖；主动执行 `rosdep install` 后，rosdep才会解析清单并调用系统包管理器。两者都需要可用的仓库和版本。
- catkin会编译项目自带源码、生成 `.msg` 的语言接口；这是生成构建产物，不是联网安装系统库。
- 下文 `*-dev`、GCC、CMake主要是**从源码构建所需**。若只部署已在匹配ARM64环境构建并验证的程序，飞行板不必为运行额外装全部编译工具和Eigen头文件；仍须满足ROS、OpenCV、libi2c等实际动态库依赖。不能直接复制Nano的旧 `build/devel` 代替适配。

依据：[catkin顶层构建](../../src/CMakeLists.txt)、[rosdep官方说明](https://github.com/ros-infrastructure/rosdep/blob/master/doc/commands.rst)。

## 2. 机载部署可不额外安装的内容

| 内容 | 省略条件或原因 |
|---|---|
| JetPack、CUDA、cuDNN、TensorRT、jetson-inference、jetson-utils、Tegra多媒体库 | 原Nano专用视觉栈，不是RK板的依赖解决方案；对应视觉后端和相机入口必须适配。旧 `.engine` 不可用于RK NPU |
| PyTorch、torchvision、TensorFlow、TensorBoard、训练数据集和导出工具 | 板上只运行已导出的模型时不需要训练环境；训练/转换放在另一台电脑 |
| Gazebo、PX4 SITL、仿真插件、`gc_2026_sim` | 真机机载运行不使用；PX4实际固件在飞控上。若让当前七包全部参加编译，`gc_2026_sim`仍要求 `gazebo_msgs`，不能直接删依赖后声称七包完整构建 |
| Windows WSL／Docker／Lichtblick客户端 | 不是板上ROS运行前置；Lichtblick客户端在电脑上，板侧桥单独选装 |
| move_base、DWA、TEB、AMCL、MoveIt | 当前实际默认链不调用；`location/package.xml`中的旧MoveIt导出标签不等于运行依赖 |
| 另行下载RPLIDAR SDK／安装第二份 `rplidar_ros` | 本项目已有ROS驱动和 `src/rplidar_ros/sdk/`，随catkin编译；Linux串口设备驱动和权限仍须有效 |
| 外部EGO包、单独 `liblbfgs-dev` | 本项目已有EGO实现与 `lbfgs.hpp`；Eigen仍是外部构建依赖 |
| Adafruit PCA9685库、Jetson.GPIO、RPi.GPIO、Python smbus／pyserial | 当前TFmini和PCA9685采用项目内C++实现；`location`仍需要Linux的libi2c，板级I²C和串口配置仍需核对 |
| RealSense／librealsense／T265驱动 | 保持 `rplidar_location=true`、不启用T265时可省 |
| RViz、rqt、xacro、joint_state_publisher、robot_state_publisher | 不在板上开GUI，且保持 `open_rviz=false`、`load_model=false` 时可省 |

关于相机：旧真机路径不启动 `usb_cam`，但这不代表RK板自动有相机输入。若选用现有Python检测节点，它只订阅ROS `Image`，需要另有相机发布节点，例如经验证的 `usb_cam`／`v4l2_camera`，或新增直采适配。普通GStreamer是否需要，按选定的RK相机方案判断；不复用Jetson的NVMM／NVIDIA专用插件。

## 3. 联网后哪些可以由工具处理

| 项目 | 实际触发条件 |
|---|---|
| ROS基础组件及其系统库 | 执行 `apt install ros-melodic-ros-base`；已安装兼容版本时不重复装 |
| MAVROS依赖的 `mavros_msgs`、MAVLink、GeographicLib库等 | apt安装MAVROS时按所选deb的依赖声明拉取；**地理数据集另外核对** |
| Cartographer依赖的Ceres、Protobuf、日志库、Lua等 | 使用匹配的Cartographer二进制deb时由apt处理声明的间接依赖，不需先逐个源码安装；源码构建另按其锁定版本处理 |
| 本项目 `package.xml`已声明且rosdep有规则的外部项 | 主动运行rosdep后可以补装；它不会扫描C++源码推断所有遗漏项 |
| `VisionDetection`、`VisionDetectionArray`、`TargetDetection`接口，RPLIDAR SDK和EGO内部库 | catkin构建时从项目文件生成，无需下载另一份；消息生成工具本身要先装 |

当前包声明并不完整：例如 `location` 的CMake要求 `nav_msgs`、`tf2_ros`、`cmake_modules`、Eigen和libi2c，清单没有完整反映；`navigation_2d`的CMake还要求OpenCV和 `cmake_modules`。外部启动项MAVROS／Cartographer及Python的onnxruntime也不能仅依靠这六个机载包的清单补齐。因此 **rosdep成功不等于运行依赖齐全**。

rosdep可只扫描六个机载包，避免 `gc_2026_sim` 的仿真依赖一起进入安装清单。以下为**后续参考命令，未执行**，要求ROS与rosdep已正确安装、初始化：

```bash
# 在项目根目录，Melodic已结束支持，需让rosdep包含历史发行版。
rosdep update --include-eol-distros
rosdep install --from-paths \
  src/location src/navigation_2d src/ego_planner_2d \
  src/rplidar_ros src/servo_control src/ssd_detection \
  --ignore-src --rosdistro melodic -y
```

若使用的旧rosdep没有相应选项，需先核对其版本与历史发行版处理方式；不能把一条命令当作已通过的板端安装方案。

## 4. 需主动用apt补齐的主要项

以下按源码构建所需加默认真机运行所需列出；已经存在且兼容的项无需重复安装。

| 类别 | apt包名 | 项目用途 |
|---|---|---|
| ROS基础 | `ros-melodic-ros-base` | roscore／roslaunch／roscpp／rospy及基础消息；不要求desktop-full |
| 构建工具 | `build-essential cmake` | GCC／G++／make／CMake；`pkg-config`为辅助查询工具 |
| 数学与图像开发库 | `libeigen3-dev libopencv-dev` | Eigen用于定位／规划等构建；OpenCV被navigation的CMake无条件要求，不能因为更换检测器而整体省略 |
| I²C开发库 | `libi2c-dev` | `i2c/smbus.h`及location链接的libi2c；仅 `i2c-tools` 不足以替代 |
| ROS构建辅助 | `ros-melodic-cmake-modules ros-melodic-message-generation ros-melodic-message-runtime` | CMake模块与自定义消息生成／运行接口 |
| TF | `ros-melodic-tf ros-melodic-tf2-ros ros-melodic-tf2-geometry-msgs` | TF坐标树；底层tf2由包依赖处理 |
| 图像ROS桥 | `ros-melodic-cv-bridge` | ssd_detection的CMake无条件查找；与C++／Python所选OpenCV保持兼容 |
| 飞控通信 | `ros-melodic-mavros` | `location.launch`直接include `mavros/launch/px4.launch`；只装mavros-msgs不能运行MAVROS |
| 建图 | `ros-melodic-cartographer-ros` | 当前定位launch直接启动Cartographer及occupancy grid；不在本项目源码内 |
| 可选机载监看 | `ros-melodic-foxglove-bridge` | 要从Lichtblick查看实时话题才安装／启用；历史Melodic版本另按兼容性锁定 |
| 可选排查 | `i2c-tools v4l-utils` | I²C／相机排查工具，不是所有节点的链接依赖 |

下列为**有可用Bionic／ARM64及Melodic源时的安装参考，未在板端验证**：

```bash
sudo apt update
sudo apt install \
  build-essential cmake pkg-config \
  libeigen3-dev libopencv-dev libi2c-dev \
  ros-melodic-ros-base ros-melodic-cmake-modules \
  ros-melodic-message-generation ros-melodic-message-runtime \
  ros-melodic-tf ros-melodic-tf2-ros ros-melodic-tf2-geometry-msgs \
  ros-melodic-cv-bridge ros-melodic-mavros ros-melodic-cartographer-ros
```

标准ROS基础包及上述依赖会带入常用消息包；若选的是自行裁剪的ROS构建，另逐项核对 `roscpp rospy rosconsole std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs visualization_msgs actionlib_msgs tf tf2 tf2_ros tf2_geometry_msgs mavros_msgs cv_bridge message_generation message_runtime`。只编译但不运行飞控时可仅用 `mavros_msgs`，这与完整机载运行是不同覆盖范围。

Melodic是Ubuntu18.04／ARM64的历史目标平台，官方支持期已结束；本次未检查板端仓库或所有候选deb的可获取性。`apt`找不到包时先核对源、签名、架构和候选版本，不混用Noetic／其他Ubuntu发行版的软件源。[ROS官方REP-3](https://github.com/ros-infrastructure/rep/blob/master/rep-0003.rst)

## 5. 需单独处理，不能只靠上面apt命令的内容

| 项目 | 处理方式与边界 |
|---|---|
| MAVROS的GeographicLib地理数据 | 核对 `egm96-5` 数据集；库和数据是两件事。缺失时主动运行安装版本随附的 `install_geographiclib_datasets.sh`，或安装geographiclib-tools后使用对应下载工具。脚本需要联网和权限；不会因为启动MAVROS而自动补齐 |
| CPU检测方案 | 现有Python脚本需要Python3、NumPy、cv2、onnxruntime（优先后端），以及该解释器可用的rospy／cv_bridge／生成消息。按ARM64、Python和glibc选择匹配wheel／源码构建，不能直接照抄其他电脑的 `pip install` 版本 |
| Melodic的Python兼容 | 官方Melodic目标为Python2.7；仅 `apt install python3-opencv` 或 `pip3 install onnxruntime` 不会自动把rospy、cv_bridge和catkin生成消息切换成兼容的Python3组合。该路线需要单独验证，不在此要求升级整套系统 |
| 当前ONNX的导入支持 | 模型为opset17；源码记录较旧OpenCV DNN无法导入相关Slice。不能把Ubuntu18.04自带OpenCV3.2当作已经可用的CPU推理后端；须实测兼容onnxruntime／OpenCV或调整模型 |
| NPU检测方案 | RK3399Pro对应第一代RKNN。按板上实际镜像核对 `librknn_api.so`、头文件、NPU固件／驱动、`npu_transfer_proxy`、转换工具和 `.rknn` 模型；所缺部分从本型号配套SDK安装或适配。不能改装Toolkit2替代 |
| 模型转换环境 | 第一代RKNN-Toolkit放在兼容的转换电脑环境；若板端采用C／C++ API运行已转换模型，不必在板端安装全套训练和转换环境 |
| 板级驱动和设备配置 | UART／USB／I²C／相机设备、权限及BSP支持须与实际Ubuntu镜像配套；这不是ROS依赖下载可以自动完成的配置 |

MAVROS说明明确区分GeographicLib库和数据，缺少所需geoid数据可使节点退出：[官方MAVROS安装说明](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)、[数据安装脚本](https://github.com/mavlink/mavros/blob/master/mavros/scripts/install_geographiclib_datasets.sh)。应优先用安装版本随附脚本，未在本轮下载或执行远端脚本。

RK3399Pro使用旧RKNN工具链，且NPU固件按USB／PCIe区分；官方描述部分配套固件默认启动通信代理，这属于**预装服务的自动启动**，不代表联网自动安装。[旧RKNN官方仓库](https://github.com/rockchip-linux/rknn-toolkit)、[RK3399Pro专属驱动与代理说明](https://github.com/airockchip/RK3399Pro_npu)

## 6. 当前本地源码仍需适配的两个入口

1. `navigation_2d.launch` 的 `use_usb_cam=true` 会选 `ssd_detection_trt.launch`，仍调用Jetson C++后端；RK上缺Jetson依赖时CMake会跳过该目标，不能把构建成功等同于相机识别可运行。
2. 现有Python入口只订阅 `/camera/image_raw`，不会因设置 `input_URI` 自动打开USB相机；选它作为RK的CPU基线时，还需可工作的相机发布源及一致的Python3 ROS环境。选NPU时需另接RKNN推理节点与既有检测消息。

证据：[视觉CMake](../../src/ssd_detection/CMakeLists.txt)、[任务视觉入口选择](../../src/navigation_2d/launch/navigation_2d.launch)、[Python检测入口](../../src/ssd_detection/scripts/ssd_detection_node.py)、[定位构建](../../src/location/CMakeLists.txt)、[任务构建](../../src/navigation_2d/CMakeLists.txt)、[定位启动链](../../src/location/launch/location.launch)。

## 7. 接续需要的只读板端信息

```bash
cat /etc/os-release
dpkg --print-architecture
rosversion -d
python3 --version
rospack find mavros
rospack find cartographer_ros
apt-cache policy libi2c-dev libopencv-dev ros-melodic-cv-bridge \
  ros-melodic-mavros ros-melodic-cartographer-ros
```

以上仅用于把“依赖应该是什么”收敛到“这块板已经有什么、还缺什么”。本轮未获得这些输出，未连接设备，也未改动源码或原始归档。
