# RK飞控USB、GPS与相机退出修复验收

[当天日志](../../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md)｜[前序台架检查](../RK外设台架核验_2026-10-24/README.md)｜[十根线接线表](../../项目资料/RK板_PCA9685与TFmini接线表_2026-10-24.md)

时间：2026-10-24约21:54–22:45，Asia/Shanghai。SHELL授权完成前三项，并补充GPS亮灯、连接PX4的GPS1；沿用已拆桨、无动力电池、仅USB供电条件。依赖全部电脑获取、核对后SSH传入，未要求RK接互联网。

## 1. 最终结果

| 项目 | 已取得证据 | 边界 |
|---|---|---|
| 正式飞控串口／MAVROS | cdc_acm加载，`/dev/ttyACM0`及Auterion by-id出现；RK正常重启后自动加载。正式MAVROS连接且armed=false，IMU1114条、约50.68Hz | 只接收遥测，不代表外部位姿融合或任务／飞行通过 |
| GPS | 查明GPS_1_CONFIG=0且驱动未运行；GPS1启动后识别u-blox NEO-M9N／SPG4.04／协议32.01／UBX115200，保存GPS_1_CONFIG=201；飞控重启后自动启动。正式MAVROS收到190条raw/fix，约8.63Hz | 当前无定位、卫星数0，NavSatFix.status=-1；无有效位置精度／室外定位验收 |
| 相机 | 项目内修复usb_cam0.3.7，系统包未覆盖；1项重复shutdown回归通过，MJPEG1280×720／YUYV640×480各两轮出图后均正常退出0；最终同跑650帧、约29.99FPS | 未加载标定，未测YOLO推理或长期负载性能 |
| 雷达共同运行 | 最终同跑217帧、约10.02Hz，有效距离点持续出现 | 本轮未改雷达源码，不扩大到建图／定位验收 |
| 收尾 | 相机、雷达、MAVROS、master最终均退出0，数据／日志落本地 | 临时测试工具清理，正式驱动、源码及运行入口保留 |

最终结构化证据：[正式并行检查结果](正式并行检查结果.json)、[执行输出](20_相机雷达正式MAVROS并行验收.txt)、[相机反复启停](相机反复启停结果.json)、[gtest XML](相机生命周期测试.xml)。`passed_receive_and_exit=true`与`gps_position_fix_passed=false`分开保存，不能将GPS收到了消息写成已定位。

## 2. cdc_acm来源、兼容性与持久化

运行内核仍为4.4.194／AArch64／SMP／mod_unload，未更换boot、DTB或整机镜像。采用[Toybrick官方stable源码](https://github.com/rockchip-toybrick/kernel/tree/a80be5749ac552821967eff313df53f9e0cd1e01)，固定commit `a80be5749ac552821967eff313df53f9e0cd1e01`；压缩包SHA256：`a502c5af837ca291c9b0b2f1e2ae28233b84fc5b7e6438d79301eda9f2fb9a16`。

原Image没有嵌入IKCONFIG，板端没有配套headers。构建配置使用公开rockchip_linux_defconfig加USB_ACM=m、MODVERSIONS=n、空LOCALVERSION、关闭LOCALVERSION_AUTO；[保存的构建配置](CDC构建配置.config)不是从运行内核恢复的完整原配置。板端原模块vermagic没有modversions；本次native GCC7.5与原内核GCC6.3不同，不能仅凭同版本字符串判兼容。

为此在加载前另行核对：

- 固件内Image与板端boot_linux分区中的Image SHA256相同，均为`8163ea96ace6573ad4585aebd0102f972d9698f84f512061e533526e2a0175c7`。
- [原有mwifiex模块](原有MWIFIEX模块DWARF布局.json)与[候选cdc模块](候选CDC模块DWARF布局.json)的11个共同结构大小及已记录成员偏移完全一致，含module、usb_device、usb_interface、usb_host_interface、urb、device、mutex、work_struct等；[原8188eu布局](原有模块DWARF布局.json)也留作交叉证据。
- 候选模块88个未定义符号全部可在原Image的10027项导出表中定位，见[核对结果](11_原始Image导出符号核对.json)。运行内核未公开__ksymtab符号，首次用/proc/kallsyms枚举导出的检查方法不适用，保留失败记录，未把它误报为88项驱动依赖缺失。
- vermagic为`4.4.194 SMP mod_unload aarch64`，未使用force、忽略版本或修改模块校验数据。原生insmod成功后实际出现ttyACM0，随后正式MAVROS实测通过。

安装位置：`/lib/modules/4.4.194/extra/cdc-acm.ko`，运行depmod，独立`/etc/modules-load.d/rk-cdc-acm.conf`声明开机加载。RK正常重启后的systemd-modules-load退出0，cdc_acm已加载并绑定飞控。外部构建模块正常产生taint4096（O标记），记录为来源标记，不伪称仍为0；未出现本轮模块相关Oops或BUG。

模块SHA256：`584bc82639089c9c1cdb0c8c3a8b10cfd74ab4b3989085ef7da66740275a7bb8`。本地二进制另存于被忽略的firmware/rk-cdc-acm-4.4.194-20261024目录，源码／构建方式及校验值入记录；不把其它内核的ko套到本板。

复现工具：[独立构建](../../../tools/rk_build_cdc_acm.sh)、[DWARF布局提取](../../../tools/elf_dwarf_layout.py)、[原Image导出核对](../../../tools/rk_kernel_export_check.py)。构建工具不自动安装；配置、布局、实际Image与导出核对均针对本次BSP，不能当成其它版本的通用保证。[构建输出](08_Toybrick同版本模块准备与构建.txt)、[加载](12_CDC模块加载与飞控标准串口.txt)、[安装](13_CDC模块持久安装.txt)、[重启结果](19_RK重启后自动加载与串口.txt)。

## 3. GPS参数与定位策略

USB产品报告FMU_V6C；NSH ver all确认硬件PX4_FMU_V6C／V6C01、固件Release1.13.3，git `1c8ab2a0d7db2d14a6f320ebd8766b5ffaea28fa`。初始GPS_1_CONFIG=0、GPS_2_CONFIG=0，gps status为not running，sensor_gps未发布；并非只凭室内环境推测收不到数据。

按[1.13.3板级串口定义](https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/boards/px4/fmu-v6c/default.px4board)，GPS1对应/dev/ttyS0。确认MAV_0_CONFIG=101（TELEM1）、MAV_1_CONFIG=0，启动GPS1接收后取得NEO-M9N模块与实时sensor_gps，随后仅将GPS_1_CONFIG从0改为201。[官方参数含义](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference#GPS_1_CONFIG)与[GPS启动配置](https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/src/drivers/gps/module.yaml)用于核对。

通过标准串口确认未解锁后执行普通飞控reboot；启动后GPS1配置仍201，驱动自启，状态OK、约898B/s接收及4.94Hz发布的启动期样本。随后正式MAVROS最终接收约8.63Hz GPS消息，不能把两个不同时间窗的频率冒写成固定频率。

**保留现有视觉定位策略：** SYS_HAS_GPS=0、EKF2_AID_MASK=24均未修改。前者仍限制GPS进入传感器处理链，后者未开启GPS融合；本轮仅恢复硬件接收和MAVLink／ROS观测，不宣称飞控已经使用GPS导航。定位有效性需后续室外测试和明确的融合方案；当前fix_type=0／0颗卫星，ROS status=-1，输出中的零经纬度及无定位高度不能用于飞行。

证据：[初始版本／参数](02_PX4版本GPS驱动与参数.txt)、[端口检查](04_GPS1串口占用与融合参数.txt)、[运行时启动](07_GPS1驱动启动与实际数据.txt)、[参数保存](14_GPS1配置保存与标准串口诊断.txt)、[飞控重启](15_PX4正常重启验证配置.txt)、[重启后状态](16_PX4重启后GPS自启与配置保持.txt)。只发送心跳、诊断shell命令及上述GPS1配置／普通重启，没有解锁、切模式、飞行设定值或电机测试。

## 4. 相机修复与运行入口

将上游usb_cam0.3.7／commit `addab4a65fdf65c460fec2cc3eee8fab94699370`按BSD许可证纳入[项目源码](../../../src/usb_cam/UPSTREAM.md)。修复未初始化AVCodecParserContext、节点／成员两次shutdown导致的重复munmap／close，以及FFmpeg帧和图像存储的释放；移除重复帧分配。采集与颜色转换接口保留。系统/opt/ros/melodic原包未覆盖，新工作空间加载后使用/home/jcfh/catkin_ws的同名包和库。

本次新增第8个包usb_cam及相机节点／共享库。只构建相关包，不把条件跳过的Jetson视觉目标写成RK视觉通过。1项无设备重复shutdown回归与真实四轮启停全部通过；四轮收到60／61／60／60帧且进程均退出0。最终正常重启后与雷达及正式MAVROS同跑，650帧相机数据且全部子进程退出0，前序停止崩溃问题按这些配置验收。

独立入口（均需先source /home/jcfh/catkin_ws/devel/setup.bash）：

```bash
roslaunch usb_cam rk_usb_camera.launch
roslaunch location rk_mavros_bench.launch
```

相机发布/camera/image_raw与/camera/camera_info；飞控入口只开sys_status／imu／global_position／local_position插件，不启动任务、舵机、定位或导航节点。上面是各自独立入口，不等同已经配置常驻或整机启动；使用同一串口／相机的程序不能重复打开。

验证工具：[相机生命周期](../../../tools/rk_camera_lifecycle_check.py)、[并行台架](../../../tools/rk_peripheral_bench_check.py)、[标准串口GPS诊断](../../../tools/rk_px4_serial_shell.py)、[无驱动阶段的USB诊断](../../../tools/rk_px4_usb_diagnostics.cpp)。诊断工具的维护模式有明确固定命令及未解锁检查，使用前核对设备身份和端口占用。

## 5. 离线依赖、通信中断与清理

源码、bc与pyelftools均在电脑下载。Toybrick源码与bc经SHA256核对后传板，bc只解压到临时构建目录；pyelftools只在电脑解包用于DWARF，不装到板上。bc1.07.1-2 ARM64 SHA256为`ef5a6e1e0686527083a7744f22eed432aee163ca33d75537af4fb46d18e0172a`；pyelftools0.31 wheel为`f52de7b3c7e8c64c8abc04a79a1cf37ac5fb0b8a49809827130b858944840607`。

模块安装前出现一次USB网卡从Windows消失，SSH连接尚未建立，安装命令未执行；SHELL反馈原因不明、按RESET后恢复。原失败输出保留，不能将该中断归因cdc_acm加载。之后执行一次正常RK重启验证自动加载／USB SSH恢复，成功。每次RK重启后均按电脑手动校时再验ROS；NTP／RTC仍未验收，不把错时systemd历史时间当成当前执行时间。

最终归档包含相机回归／并行台架原始ROS日志，SHA256为`fa76e85e603deb0bec8535352872aa01c22bb44b01789fbd807f73402ff6dcad`，与板端一致：[原始证据](相机与正式MAVROS原始证据.tar.gz)。正式模块、autoload声明、源码与构建产物保留，板端下载／内核构建目录及测试工具／输出在本地核对后清理，测试可执行文件与gtest产物另行移除。没有将本轮测试节点设为开机任务。

新增的18项已部署源码／launch文件与本地SHA256一致，见[一致性结果](22_新源码一致性核对.json)；上游CI模板未部署到RK。22:45确认临时构建／测试缓存约1.2GB已删除，测试二进制、对象和gtest输出已清理，正式模块、autoload声明及相机程序／共享库保留，无测试进程／端口，SSH与USB服务active、dpkg审计无输出，见[清理实录](23_板端清理与正式产物保留.txt)。

I²C只读映射另见[证据](17_I2C总线与引脚复用只读映射.txt)及[十根线接线表](../../项目资料/RK板_PCA9685与TFmini接线表_2026-10-24.md)。没有连接、扫描或驱动PCA9685／TFmini；照片未确认的1脚朝向、具体TFmini型号／模式、实际线序与电平仍须实物核对。
