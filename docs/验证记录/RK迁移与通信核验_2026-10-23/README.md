# RK软件迁移与回环核验

[项目目录](../../../logs/低空消防无人机工程日志.md)｜[当天日志§28–32](../../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md)｜[任务状态](../../项目资料/任务状态.md)

真实操作时间：2026-10-24约02:59–03:26，Asia/Shanghai；按SHELL的08:00前归前日规则记入2026-10-23。SHELL授权SSH及root操作，并明确当前只有RK板，先完成软件迁移与回环；所有备份和证据保存在本地工作空间，板端测试临时文件已清理。

**后续条件澄清（10月24日约03:48）：** SHELL说明当时RK没有接入互联网。下述外网解析失败和NTP未同步是该条件下的实际输出，不认定DNS配置有故障；需要下载时先联网再核验。原始证据不改写，详见[日志§34](../../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md#34-shell澄清rk尚未接入互联网)。

## 1. 结论与验收范围

| 项目 | 实际结果 | 主要证据 |
|---|---|---|
| SSH、sudo与网络 | jcfh登录成功、sudo UID0；USB192.168.77.1/24、无线192.168.1.102/24 | [初始清单](01_板端初始只读清单.txt)、[root复核](02_root与服务复核.txt) |
| 通用依赖 | Melodic、MAVROS1.16.0、Cartographer、OpenCV3.2、Eigen、libi2c等已装；egm96-5非空；dpkg无未完成配置 | [初始清单](01_板端初始只读清单.txt) |
| 补齐系统组件 | 补dbus1.12.2-1ubuntu1.4；jcfh加入dialout/video/i2c；手动校时 | [安装模拟](03_dbus缺项与安装模拟.txt)、[离线安装](05_离线补齐dbus与校时.txt) |
| 七包构建 | 10个非Jetson目标生成；235个源码文件与本地预期一致；10个AArch64 ELF及动态链接检查通过 | [构建](06_七包catkin构建.txt)、[修复后构建](11_定位生命周期修复后构建.txt)、[最终产物核对](回环与调试输出/final-source-and-binaries.json) |
| 已有单元测试 | 修复测试入口后8项TEST全部通过 | [首次失败](07_gtest首次链接失败.txt)、[通过记录](08_gtest补入口后8项通过.txt)、[XML](回环与调试输出/test_results/navigation_2d/gtest-ego_path_tracking_test.xml) |
| ROS消息 | String、PoseStamped、State、PositionTarget、VisionDetection、VisionDetectionArray、TargetDetection七种消息字段及TCPROS传输通过 | [消息JSON](回环与调试输出/results02/messages.json) |
| MAVROS回环 | 合成MAVLink→真实MAVROS，connected且未armed；NED→ENU、FRD→FLU、姿态及合法下行心跳通过；节点正常退出0 | [MAVROS JSON](回环与调试输出/results02/mavros.json) |
| 定位与跟随应用 | 两组TF→位姿；正X速度、路径超时清零、恢复、trigger=false清零；定位和跟随器均正常退出0 | [最终应用JSON](回环与调试输出/results03/application.json)、[执行输出](12_应用回环与正常退出通过.txt) |
| 收尾 | 临时文件／测试二进制及对象清理；无验证ROS／GDB进程，测试监听已退出；SSH与USB仍active/enabled | [清理实录](14_板端清理与收尾复核.txt) |

以上是**仅RK板的软件构建、消息与合成协议回环验收**。没有真实飞控／雷达／测距／舵机／相机接入，没有飞行、投放、真实定位质量或完整负载性能验收。`tfmini_test`等原项目正式工具属于正式构建目标，未运行；本轮新生成的gtest可执行文件、对象和结果已清理。

## 2. 板端环境与实际补项

Ubuntu18.04 ARM64、厂商内核4.4.194、CPU侧可见内存3.8GiB、无swap，根分区29G／首次可用26G。设备节点video0–9都来自内置ISP，不能据此宣布USB相机可用；未发现ttyACM／ttyUSB。只列I²C适配器，未扫描从机或读写总线。

原系统缺少dbus守护进程及系统总线socket，普通账户systemctl失败；root通过systemd私有通道确认SSH与USB服务正常。安装模拟仅新增dbus、无升级／移除。板端腾讯源DNS解析失败后，电脑从[Ubuntu官方包端点](https://ports.ubuntu.com/ubuntu-ports/pool/main/d/dbus/dbus_1.12.2-1ubuntu1.4_arm64.deb)下载，核对板端apt索引SHA256 `de27c5251080176b9eac9e1a279d7387b3bd7f8f1f3db1da62d6edc0b25b7f95` 后传板用dpkg安装。未更换无线配置、DNS或系统源，未做系统升级。参见[在线安装失败](04_板端在线安装DNS失败.txt)。

板端原时钟比本机慢约50分钟，在ROS启动前手动校准。timesyncd已运行，但NTP同步仍未成立，RTC仍为旧值；不能把本次手动校时记为自动校时通过。设备组在后续新登录生效。

Python2的rospy/cv_bridge原生模块/numpy可用；Python3缺rospkg/numpy且cv_bridge存在ABI不匹配。没有为此更换系统Python或盲装推理库。rosdep包已装，但其全局sources/cache尚未初始化；本轮以已装包、实际构建及ldd确认通用构建依赖，未声称rosdep流程通过。

## 3. 两处源码修复

1. [ego_path_tracking_test.cpp](../../../src/navigation_2d/test/ego_path_tracking_test.cpp)：原测试缺main，catkin仅链接gtest时undefined reference to main。补标准InitGoogleTest/RUN_ALL_TESTS入口，原8项断言不变。catkin_test_results显示16的汇总口径不当作新增16项测试。
2. [location_with_ftmini.cpp](../../../src/location/src/location_with_ftmini.cpp)：功能输出正确，但首次SIGINT退出为-6。GDB捕获[回溯](回环与调试输出/location-shutdown-gdb.txt)，栈经过TimerManager::remove、Timer::Impl::stop／析构及Timer析构。将main之前构造的全局timer/height_timer移入main外层作用域，避免静态析构时访问先释放的TimerManager；参数、频率、回调与定位计算保持。修复后重建及严格退出码测试均通过。

生命周期判断还核对了本机roscpp1.14.13对应的[Timer实现](https://raw.githubusercontent.com/ros/ros_comm/1.14.13/clients/roscpp/src/libros/timer.cpp)和[TimerManager实现](https://raw.githubusercontent.com/ros/ros_comm/1.14.13/clients/roscpp/include/ros/timer_manager.h)。[修复前文件](修复前/)保存原字节；当前两份源码维持原换行风格。

首轮测试工具另有两个已纠正问题：仅通过argv设置Python节点__ns未生效，改在首次导入rospy前设置ROS_NAMESPACE；应用工具原只检查子进程已停止，改为必须退出0，避免将定位-6误记通过。[首轮结果](10_首轮功能回环与退出异常.txt)保留此历史，最终以results03为准。GDB编排脚本读取inferior时调试器已结束，因此编排返回错误；其完整回溯已产生并保留，正式修复验收采用随后正常退出的应用回环。

## 4. 工作空间、重现与文件完整性

正式RK工作空间为`/home/jcfh/catkin_ws`，首次创建前确认不存在；只迁移当前`src/`，排除engine/plan/pyc，不复制Nano构建目录。初始源码传输包32,961,862字节、SHA256 `fa1d67c76eb7340fb7c3df83722e84e7336f08670400b055c2da6f154f373147`；[源码清单](源码传输清单.json)同时保存初始235文件哈希及两处修复后的部署哈希。最终板端与本地匹配，0处差异。

今后普通SSH新登录时手动加载：

```bash
source /home/jcfh/catkin_ws/devel/setup.bash
```

通用构建命令为`catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON`。没有设置任务、ROS master或模型开机自启。源码变更仅上述两处；Jetson检测节点是第11个条件目标，当前跳过，RKNN及Python3视觉仍待适配。

本地保留的复现工具：[只读依赖采集](../../../tools/rk_dependency_audit.sh)、[消息测试](../../../tools/rk_ros_loopback_test.py)、[MAVROS测试](../../../tools/rk_mavros_loopback_test.py)、[应用测试](../../../tools/rk_application_loopback_test.py)、[运行器](../../../tools/rk_run_validation.sh)。测试要求独立非11311回环master和空图，只使用随机验证命名空间；应用测试显式关闭真实测距，不启动任务或舵机节点。若复测，临时复制这四个回环工具到同一目录后执行：

```bash
bash rk_run_validation.sh /home/jcfh/catkin_ws /指定临时证据目录 all
# 可将all改为messages、mavros或application进行专项复测。
```

测试脚本不常驻RK。输出先下载、核对后再清理，不删除正式catkin源码或现有USB自启文件。

## 5. 证据归档与板端清理

[原始输出归档](回环与调试原始输出.tar.gz)的SHA256为`1f0dbcd1d8d2143e526f6a80c83ddd526fee52919d1f9bf1bbf26166dbb82765`，两端一致；本地完整读取62个普通文件，随后在[输出目录](回环与调试输出/)保留6份关键JSON／回溯／XML便于阅读。全部原始ROS日志、PID／缓存及4个latest符号链接保存在归档，未将冗余缓存逐个提交。归档包括前序失败、最终JSON、ROS节点日志、GDB回溯和gtest XML，没有密码或Wi-Fi配置。

03:26板端已删除本轮临时cache目录（含测试工具、调试器、解压库、日志、归档）、初始审计脚本、源码传输包、离线dbus包、gtest结果、gtest可执行文件及其编译对象／库。正式src/build/devel及CMake构建规则保留，以便继续开发；原有rk_usb_ssh.sh及开机服务不动。检查无测试ROS/GDB进程和测试监听，SSH与USB仍active/enabled，dpkg无未完成配置。认证会话已关闭，密码未写文件。

## 6. 后续边界

- 实际飞控、雷达、TFmini、PCA9685及相机通信待接设备后按身份、端口、参数逐项验证；I²C电平／排针／独立舵机电源仍需实物核对。
- 已测消息传输，不代表真实视觉检测、MissionVision整链、任务状态机或串口／I²C设备协议通过；此次未启动rescue_mission_node。
- Jetson视觉、Python3/ROS ABI、旧RKNN组合及模型推理另行适配；不因10个目标构建成功宣布全功能部署。
- 板端此前未接入互联网；联网后再验外网解析和自动校时，不将此前结果认定为DNS故障。正式离线LAN／USB仍可用。
- 静态审查发现TFmini校验失败分支等历史问题仍需独立修复与针对性验证，未在本轮无设备条件下宣称解决。
