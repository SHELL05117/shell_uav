# 无线地面排查与MPP／RGA相机适配

实际核验：2026-10-28约02:30起，北京时间，具体时间见原始记录；08:00前工程归2026-10-27。SHELL确认拆桨、未起飞，并授权局域网SSH排查与适配。源码分支`codex/wifi-camera-startup-20261027`。本轮没有解锁、OFFBOARD、舵机／电机动作、带载投放或实飞。

## 1. 结论与证据范围

| 项目 | 实际结果 | 边界 |
|---|---|---|
| 原rk_preflight复现 | 真实硬件联合17项通过；存在启动期TF等待与VER版本查询报错 | SHELL那轮原终端完整报错未保存，不声称复现了所有nav2d／IMU故障 |
| ROS 1原生连接 | Computer Use实际设置HTTP Master＋电脑IP；真实地图、JPEG相机和飞控状态同屏 | WS是独立对照，不能替代该项验收 |
| 原Lichtblick失败 | 原失败截图／完整错误未取得；当前同板WS桥数据对照可用 | 未宣称已复现或修复那个客户端当时的具体错误，按SHELL选用的Foxglove原生路线完成交付 |
| 地图 | 原布局/map隐藏；固定／显示参考系选map并启用/map后出现真实2D栅格图 | 无伪造IMU、静态地图或地面真值替代 |
| 相机 | MJPEG→MPP硬解NV12→RGA RGB8；60次同一JPEG处理约71.60fps，RGB MAE均小于1／255 | 为离线重复帧处理能力，不是推理、无线或动态场景帧率 |
| 实时预览 | 原图JPEG上限15fps；原生直接订阅时实际看到颜色正常画面 | 未压缩detect_result仍复现彩色条带；未确诊其客户端／传输根因，不把所有变色归为RK性能 |
| 新入口 | uav check一条禁动联查；正式uav a→uav mission两窗口；统一环境／设备发现／就绪顺序 | mission仅展开参数核对，未运行正式动作 |
| 蜂鸣器 | tune_id=7、battery warning=1、4S约14.22V／8%；预解锁明确因电池失败 | GPS通信8Hz、未定位，但当前告警音不是GPS告警；未校准电池监测 |
| 收尾 | 联查／NSH均退出，三个设备句柄空闲，NPU／USB五服务active | 不替SHELL关机／断电或更改供电 |

## 2. ROS地址与地图显示

SHELL指定继续使用学长的ROS 1原生方法。Computer Use查看到历史连接使用ROS 1，Foxglove默认ROS_HOSTNAME为ALEX。板端`getent hosts ALEX`未得到结果；电脑WLAN为192.168.1.100，RK为192.168.1.102。

实际设置：ROS_MASTER_URI=`http://192.168.1.102:11311`，ROS_HOSTNAME=`192.168.1.100`（电脑地址）。随后在原生ROS 1连接中看到了相机、TF与真实地图。界面同时证实原3D面板的/map隐藏；启用/map、固定与显示参考系均选map、选二维俯视后地图出现。面板虽名“3D”，实际数据仍为nav_msgs/OccupancyGrid二维占据栅格图。

同一RK的`http://127.0.0.1:11311`与LAN地址通常指同一个Master；问题不是自动变成两套Master。原生远程客户端无法访问节点公布的127.0.0.1，且需能够回连客户端声明的地址。板内loopback节点配同板WebSocket桥可以工作，两条路径不能混为一谈。

Computer Use曾因SHELL按Esc停止，解释连接方式后经SHELL明确授权恢复。最终使用ROS 1原生完成验收，保存默认布局；联查停止后的空白／重连提示属于数据源已正常退出，不作为链路运行失败。

界面原始截图：

- [ROS 1连接参数](../../../logs/附件/2026-10-27/Foxglove_ROS1连接参数.png)
- [原布局隐藏/map](../../../logs/附件/2026-10-27/Foxglove_地图未勾选.png)
- [地图与压缩相机](../../../logs/附件/2026-10-27/Foxglove_ROS1地图与压缩相机.png)
- [地图、相机和未解锁飞控同屏](../../../logs/附件/2026-10-27/Foxglove_ROS1地图相机飞控状态.png)

## 3. 原preflight与新启动顺序

[01_原preflight地面复现.json](01_原preflight地面复现.json)和[02_原preflight运行.log](02_原preflight运行.log)保存20秒正式窗口及启停。17项全部通过、无设定值／GPIO、正常释放。该入口的sys_status需要command插件查询版本，受限白名单未启用它，因此反复打印VER服务查询失败；不等于IMU或模型失败。Cartographer及costmap在启动阶段也会等待IMU／TF。

保留该历史核查入口及限制，不将它衔接为正式OFFBOARD入口。新`uav`按真实FCU未解锁、IMU／scan／图像／vision新消息、location READY、地图及map→base_link TF等待就绪，再启动DWA／识别，消除“全开之后先看一屏等待报错、退出又重开ABC”的日常步骤。

`uav a`同时包含完整MAVROS定位、相机、JPEG转发；`uav c`禁动观察，`uav mission --target ...`才显式开任务动作和真实舵机且auto_start=false。正式任务准备仍须检查RC、模型、设定值、飞控预解锁与现场净空。本轮正式mission仅dump参数，没有运行。

统一环境放在`~/.local/share/uav/uav_env.sh`，`.bashrc`先备份再替换已知固定setup／ROS地址行。默认工作空间为HOME/catkin_ws，可由`~/.config/uav/env.sh`覆盖；RK公布IP按当前SSH路由选择，设备优先自动匹配唯一by-id，缺失／多台拒绝猜测。高度保存配置只在显式选择后加载，generic location默认未标定的行为保留。

## 4. MPP／RGA实现与依赖

相机V4L2实查支持MJPEG720p30；YUYV720p仅10fps，不能把格式改成YUYV还宣称保持30fps。RGA是2D色彩／缩放单元，真正的JPEG解码由MPP完成，见[Rockchip MPP](https://github.com/rockchip-linux/mpp)与[RGA说明](https://github.com/rockchip-toybrick/hardware_rockchip_librga)。

本轮实现：MPP旧MJPEG高级任务队列，DRM DMA缓冲、有限等待；RGA旧C接口把NV12转紧密RGB8，显式BT.601 JPEG全范围并使用真实stride。FFmpeg路径同时修正借用mmap内存的packet生命周期、使用实际像素格式／色彩范围和缓存sws上下文。失败帧丢弃，不重新发布时间戳掩盖断流。显式请求mpp_rga而缺依赖时失败，不悄悄退回软件。

依赖在联网电脑获取后传板，本机不训练、不改模型／标签、NPU自启、雷达SDK、内核或驱动。独立安装前缀为`/opt/shell-media`：

| 组件 | 固定来源 | 下载包SHA256 |
|---|---|---|
| MPP 1.0.0标签对应源码 | rockchip-linux/mpp，commit `66f82cdc9aa79f6dbcfe53ba17968b81395b7bc1` | `bf045fbd6fd14b5105e4ec8d3a59f041e2c22d0ab00a954bf33b7cb104018468` |
| Toybrick librga源码 | rockchip-toybrick/hardware_rockchip_librga，核对master为commit `efa7a3ee9ca2b740fa8dc02eb0cee1e57e0d6432` | master源码ZIP `6bb39daa047bbefe2a3c54ec0bec94690417bd31e8264d2d8d75ad944db2c830` |

官方预编译RGA库因GLIBC_2.29加载失败，未采用；实际在板上编译Toybrick源码。CMake3.10兼容调整仅为最低版本声明、Linux空COMMON_LIBS的object链接语句、禁用samples。MPP须HAVE_DRM=ON；首次未启用导致分配器崩溃，回溯后修正。该源码快照缺少H265／VP9／AV1部分解析文件，相应解码器禁用；本轮只验MJPEG。可复现构建辅助为[build_rk_media_runtime.sh](../../../tools/build_rk_media_runtime.sh)，输入为已解压的上述源码，源包保留在本地临时下载目录及板端本次acceptance目录，不进入Git。

RGA启动会报告旧驱动兼容模式；只按本轮实际MJPEG／NV12→RGB测试认定可用，不宣称所有RGA功能兼容。MPP旧VPU探测有不支持的ioctl日志，未出现新增Oops；内核taint实读仍为既有4096。

新增`70-uav-media.rules`仅让既有video组访问rga／vpu_service／mpp_service；未给所有用户开放设备。CMake可选编译MPP／RGA；通用软件路径保留。为诊断分配器崩溃，另离线安装gdb及其libdw1／libbabeltrace1依赖，按apt元数据核对下载MD5，没有系统升级。

## 5. 性能、预览与可验证边界

[media-compare.log](media-compare.log)：同一真实相机JPEG连续60次解码约0.838秒，约71.60fps；R／G／B平均绝对差0.328／0.503／0.346（对比OpenCV软件解码的0–255像素值）。这是颜色／stride／连续任务复用与处理能力测试，不是摄像头采集或无线帧率。

该单帧原始输入保存在[reference.mjpg](reference.mjpg)，供[rk_media_compare.cpp](../../../tools/rk_media_compare.cpp)复核；原preflight的最后相机／标注帧分别保留为[camera-last.jpg](camera-last.jpg)和[output-last.jpg](output-last.jpg)。均为选取的小型验证素材，没有重写像素或补造场景。

[03_ROS1原生联查观测.json](03_ROS1原生联查观测.json)：已有全链20秒订阅观察通过，原图接收19.95Hz、原图JPEG13.78Hz、标注7.61Hz、地图1.01Hz、vision19.71Hz、FCU pose29.05Hz；ROS1原图为1280×720 rgb8 step3840、标注640×360 bgr8 step1920。多订阅／观察进程调度会影响收到的计数，不将这些数值写成实际推理速度或全部生产者频率。readiness六项有效，所有state未解锁且连接，setpoint／GPIO零消息。

原生未压缩detect_result在Foxglove曾显示彩色条带，即使MPP／RGA数值对比已正确；切换`/monitor/camera/raw/compressed`后连续UI观察画面正常。现默认JPEG原图上限15fps／宽960／质量75，标注JPEG另一个话题；只保留最新帧，拒绝超过1秒原图，不堆积历史队列。识别图受检测输出频率影响，不作为原图流畅度标准。未在本轮改检测／模型代码，也未确诊未压缩显示条带的底层原因。

[04_WebSocket对照观测.json](04_WebSocket对照观测.json)保存此前15秒对照，16帧地图、219帧原图JPEG、TF正常、客户端能力空。跨电脑／RK的时间偏差未补偿，文件内camera_age不能作为网络时延指标；原图中继本机新鲜度与跨机网络延迟是两项不同的测量。移动距离、干扰环境、动态场景与实飞无线覆盖均未验收。

## 6. 蜂鸣器与供电

[buzzer-readonly.txt](buzzer-readonly.txt)为停止MAVROS并释放句柄后，经已确认未解锁心跳执行固定只读NSH命令的原始记录。tune_control最新时间戳仅约9毫秒前，tune_id=7；[PX4 v1.13.3消息定义](https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/msg/tune_control.msg)对应低电量慢速告警。

battery_status：connected=true、cell_count=4、voltage_filtered约14.2237V、remaining约0.0803、warning=1；commander明确Check battery、BATT ERR／PREARM ERR。RC18通道、rc_lost=false、rc_failsafe=false。GPS驱动状态OK、8Hz，但fix_type=0／satellites_used=0。当前蜂鸣器有低电量明确证据，不能称GPS告警；这不等于GPS已验收。

未改CBRK_BUZZER（实读0）、GPS、电池、解锁或失效处置参数。SHELL已收到供电核对提醒；电池或稳压电源的实际状态与飞控监测标定仍由现场核对，不用关闭蜂鸣器消除故障。

## 7. 最终核对、回退与旧command

- [final-entry-validation.json](final-entry-validation.json)：五项参数／节点展开检查通过；no_hardware_opened与mission_was_not_launched均true。
- [native-monitor-nodes.txt](native-monitor-nodes.txt)：原生模式仅monitor_jpeg；[optional-bridge-nodes.txt](optional-bridge-nodes.txt)：可选桥不重复JPEG。
- [lifecycle-test.log](lifecycle-test.log)：相机重复shutdown测试通过；[final-validation.log](final-validation.log)为最终语法及参数验证。
- [部署SHA256核对.json](部署SHA256核对.json)：20份本轮源文件／工具本地与RK字节一致；[本地文档核对.json](本地文档核对.json)与[PowerShell语法核对.json](PowerShell语法核对.json)：相对链接、11个Bash块、2个PowerShell块及Python AST检查通过，指南命令只解析未执行。
- [final-device-release.json](final-device-release.json)：最后一次飞控／相机／雷达句柄检查passed，busy／missing／errors均空。原始JSON／终端证据的空白与ANSI控制符按原字节保留；源码及人工文档另做差异空白检查。
- [configure3.log](configure3.log)、[build.log](build.log)：usb_cam与location本轮构建；[new-check-final.log](new-check-final.log)和[ui-check.log](ui-check.log)：真实禁动入口的启动／正常退出记录。
- [旧command只读副本.md](旧command只读副本.md)与[截图](../../../logs/附件/2026-10-27/旧command原文核对.png)：来自SHELL桌面原件，未改原件。location／navigation入口仍存在，但裸location未选已测高度、裸navigation默认禁动且无独立相机；Nano的nvpmodel／jetson_clocks不适用RK。

日常入口与回退见根目录[起飞指南](../../../起飞指南.md)。软件回退可在`~/.config/uav/env.sh`设置`export UAV_CAMERA_BACKEND=ffmpeg`；直接launch可传`mjpeg_backend:=ffmpeg`。硬件后端在原生／桥接两种显示路线下相同。

部署前备份位于板端`~/shell_backups/wifi-repair-20261028/source-before.tar.gz`及bashrc-before，后续统一环境安装也单独备份.bashrc。不自动覆盖回旧源码，不回滚模型／定位／NPU；需要回退时按文件清单选择。本轮源码变更、部署哈希与本地提交见后续核验文件和当天日志。测试进程已退出、设备句柄空闲；未自动关机，动力与实飞状态保持未验收。
