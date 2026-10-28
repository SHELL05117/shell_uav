# RK真实USB外设台架核验｜2026-10-24

[项目目录](../../../logs/低空消防无人机工程日志.md)｜[当天日志](../../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md)｜[任务状态](../../项目资料/任务状态.md)

操作时间：2026-10-24晚间，约21:11–21:30，Asia/Shanghai。SHELL反馈USB相机、雷达、PX4及接在飞控上的GPS已连接，授权通过本机专用密钥、USB网络SSH检查；随后确认已拆桨、动力电池断开、飞控仅USB供电。

## 1. 结论及验收边界

| 项目 | 实际证据 | 当前结论 |
|---|---|---|
| USB SSH／sudo | `rk-uav`连接192.168.77.1，jcfh及免密UID0；退出时SSH与USB服务active | 管理链路通过 |
| USB相机直接采集 | RYS USB Camera，VID:PID `0bda:9143`，`/dev/video10`；120帧V4L2采集退出0，短窗口报告约60.39FPS；保存一张1280×720 JPEG | 设备与直接取图通过；60FPS仅是短窗口采集结果 |
| 相机ROS数据 | MJPEG输入1280×720／30FPS，约18秒内收到538帧rgb8及538条CameraInfo，接收约30.03Hz，时间戳递增且消息字节数正确 | ROS采集通过；当前没有相机标定，停止节点的退出健壮性尚未通过 |
| 雷达 | CP2102N `10c4:ea60`，`/dev/ttyUSB0`；实际报告S2M1、Firmware1.02、Hardware18、health OK；复核158帧scan、约10.04Hz | 单模块扫描通过；未做建图、TF、定位或测距精度验收 |
| PX4 USB枚举 | USB报告Auterion／PX4 FMU v6C.x，`3185:0038`；CDC接口无驱动绑定，`/dev/ttyACM*`不存在，`cdc_acm`未注册且模块找不到 | 标准USB串口／MAVROS实机连接受驱动缺项阻塞；不视为飞控损坏 |
| PX4真实遥测 | 临时libusb诊断仅发送GCS心跳后，22秒接收7380条合法MAVLink消息、453086字节；21条心跳、1081条姿态和1081条HIGHRES_IMU，解析错误／丢包计数0 | 实际飞控USB数据及姿态／IMU接收通过；心跳armed=0。该通道是临时诊断，不是正式MAVROS验收 |
| GPS | 常规遥测无GPS_RAW_INT／GPS2_RAW；显式查询GPS_RAW_INT收到COMMAND_ACK(command512,result2)。SYS_STATUS的GPS present／enabled／health位均未置位 | GPS未取得可用数据，定位状态、卫星数及根因均待确认；不能写成“室内无定位但GPS正常” |

本轮没有启动导航、定位、任务或舵机节点，没有解锁、切换模式、发送飞行设定值、修改飞控参数、刷机或重启。相机与雷达同跑时的数据通过，不能扩大为全机性能、YOLO推理或飞行验收。

## 2. 设备身份与接续配置

- 相机当前数据节点`/dev/video10`，`video11`为同一UVC设备的另一节点。稳定身份路径为`/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0`；不要使用RK内建ISP的`/dev/video0`。
- 雷达当前串口`/dev/ttyUSB0`，稳定身份路径为`/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_909394b23877ed11907beefafdf7b791-if00-port0`。本轮驱动串口速率1000000，DenseBoost，目标10Hz；复核最后一帧3585点、2109个有效距离，约0.15–5.508m，是现场采样值，不是精度或量程标定。
- 飞控在USB路径`3-1.4`，相机`3-1.2`，雷达转接器在后续插入时出现于`3-1.3`。第一轮USB列表无雷达，稍后已枚举，不将早期缺失状态延伸为当前未连接。
- GPS接在PX4侧，不能期待RK出现独立GPS USB节点；本轮由飞控MAVLink数据判断。`SYS_STATUS`掩码present=2452127756、enabled=2452127756、health=2184085519，三者与生成头文件定义的GPS位32按位与均为0。

## 3. 相机前序失败与退出异常

首轮检查订阅了`/bench_camera/image_raw`，实际私有话题为`/bench_camera/bench_camera/image_raw`，因此首轮没有收到图像，不能据此认定相机启动失败。后续检查改为按ROS消息类型发现真实话题，MJPEG复核成功。首轮160帧雷达数据有效。

首轮YUYV相机进程在清理阶段收到停止信号后段错误。第一次GDB只捕获了诊断超时产生的SIGINT，没有证明崩溃位置；后续向准确的被调试子进程发送普通SIGINT，得到明确SIGSEGV回溯：`av_parser_close → usb_cam::UsbCam::~UsbCam → UsbCamNode::~UsbCamNode → main`。因此此前“启动后崩溃”的口头判断已更正为“停止时析构崩溃”。

MJPEG复核能够出图，但相机进程停止返回1；雷达与master均返回0。数据JSON中的`passed=true`只代表所列数据断言，不包含节点正常退出。尚未修复系统安装的usb_cam0.3.7，不将切换MJPEG描述为已经解决其生命周期问题。上游[0.3.7驱动源码](https://github.com/ros-drivers/usb_cam/blob/0.3.7/src/usb_cam.cpp)及[节点源码](https://github.com/ros-drivers/usb_cam/blob/0.3.7/nodes/usb_cam_node.cpp)可用于后续核对析构／shutdown路径，本轮不改第三方库。

另有两条独立信息：CameraInfo的K全0，表示本次未加载标定；`focus_auto`不受相机支持，软件颜色转换无硬件加速警告存在。它们不等同于本次图像流失败，也不能据此宣称长期CPU负载合格。

## 4. 飞控诊断方法与GPS边界

仅接收的libusb检查22秒没有数据；随后在同一未绑定内核驱动的CDC设备上，按USB描述符申请接口0／1、设置8N1和DTR，发送255／190身份的GCS心跳后取得真实数据。使用板上已安装的MAVLink生成头文件校验／解码，没有安装Python推理库或USB驱动，没有解绑其它设备、重置USB控制器或修改内核。

GPS专项检查另发送两条`MAV_CMD_REQUEST_MESSAGE=512`，请求消息24／124，不改变遥测发送频率或飞控参数。22秒接收7416条合法消息，姿态／HIGHRES_IMU各1080条，2条COMMAND_ACK；工具只展开第一条同类消息，明确展开的ACK属于GPS_RAW_INT请求，result2为UNSUPPORTED，不为第二条ACK补造结果。两轮均未观察到GPS消息。生成头文件的完整性校验通过并不等于GPS硬件验收。

正式部署仍需为当前`4.4.194`、AArch64、厂商BSP准备匹配的`cdc_acm`支持，不能随便套其它Ubuntu或不同内核模块。当前缺少配套头文件／源码，不能只凭版本字符串认定模块ABI匹配。后续再通过正式MAVROS读取飞控状态，核对GPS端口、模块供电、固件驱动及参数，最后做室外定位验证。Linux驱动实现参考[cdc-acm源文件](https://github.com/torvalds/linux/blob/master/drivers/usb/class/cdc-acm.c)，不将其当前主线版本直接当成本板适配源码。

## 5. 时钟、清理与证据

初始板端显示10月24日02:14左右，本机已为21:12左右。01–04中板端时间未经校正，保留原始输出；21:17:30按电脑UTC时间手动校正后再做ROS检查。RTC仍旧、NTP未同步，未改DNS、Wi-Fi、NTP或RTC配置。当前无线接口DOWN，USB管理链路正常；本轮未尝试无线重连。

临时GDB8.1.1及libbabeltrace1／libdw1从电脑下载Ubuntu官方ARM64包，按板端apt索引的SHA256核对后只解压到临时目录，不安装到系统。初次停止回溯编排使用本内核未提供的`/proc/PID/task/PID/children`失败，产生的GDB与相机子进程随后按准确PID停止；改用ps列子进程后取得回溯。失败输出保留。

所有诊断ROS节点、GDB、临时USB程序与监听均停止；恢复相机原先1280×720 MJPEG／60FPS配置。板端两处临时目录（含GDB与deb）、测试JPEG和归档均在下载并核对后删除，正式catkin与服务保持。没有修改`src/`或安装依赖；源码与正式构建结果沿用前阶段。

主要证据：

- [初始枚举](01_初始设备与进程.txt)、[USB驱动与相机能力](02_USB驱动与相机能力.txt)、[原生采集](03_相机取流与内核驱动.txt)、[校时](05_校时与雷达串口.txt)。
- [首轮数据](首轮数据结果.json)、[最终数据](复核数据结果.json)、[复核及子节点退出](10_ROS相机与雷达复核.txt)、[停止崩溃回溯](15_USB相机停止回溯复核.txt)。
- [仅接收飞控检查](07_PX4_USB只接收诊断.txt)、[GCS心跳后的真实遥测](12_PX4_GCS心跳诊断.txt)、[GPS查询](14_PX4与GPS消息查询.txt)。
- [相机实拍](相机原始帧_1280x720.jpg)、[原始ROS日志与诊断归档](原始ROS日志与诊断.tar.gz)、[收尾／校验](16_收尾状态与归档校验.txt)、[板端清理](17_板端临时文件清理.txt)。
- 复现代码：[传感器采集检查](ros_sensor_probe.sh)、[PX4临时USB诊断](px4_usb_receive_probe.cpp)。脚本中的设备路径为本次实读结果，重新执行前须核对当前设备身份及端口占用；它们不会常驻RK。

归档SHA256：`18c8b1778de776734d12a4a7fb427b35692727a391b882cc8b08c4d639449407`。JPEG SHA256：`3b1fb6017059470814883349c87eb85c879c803060bd7961345e64ced8ed6166`。两者电脑与RK一致。归档排除GDB二进制与deb，保留原始ROS／调试日志；04、08、09、13等过程输出均按实际结果留存，不覆盖最终证据。
