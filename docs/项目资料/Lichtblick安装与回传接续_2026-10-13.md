# Lichtblick 安装与回传接续

日期：2026-10-13。SHELL授权安装Lichtblick回传；本轮已实际安装Windows客户端并准备独立监看配置。仿真建议另见 [仿真测试方案与设备分工](仿真测试方案与设备分工_2026-10-13.md)。

## 1. 已经完成和当前限制

| 事项 | 本轮结果 |
|---|---|
| Windows客户端 | **Lichtblick1.29.1已安装**到 `C:/Users/alexh/AppData/Local/Programs/lichtblick/Lichtblick.exe`；当前用户安装，安装程序退出0 |
| 来源校验 | 官方GitHub发行API与安装包SHA256一致；282,004,508字节；没有使用第三方镜像 |
| 启动 | 程序文件版本1.29.1，卸载注册表版本1.29.1；主界面已打开，3D/Image/Raw面板可见 |
| 项目配置 | 已准备独立只读桥、可选相机预览、启动脚本与布局；静态校验通过，**monitor_layout已在客户端GUI成功导入** |
| Nano连通 | **SHELL确认无人机处于返修状态，暂时无法开机**；本轮没有Nano USB网卡或已登录SSH会话。后续由SHELL按本文操作 |
| Nano上传和启动 | **尚未执行**，未安装或改动Nano系统，未启动ROS/定位/任务/舵机 |
| 真实回传 | **尚未验收**，当前客户端显示No data source；没有把空布局或静态配置称为已收到地图/视频 |

安装证据：[client_installation.json](../验证记录/Lichtblick安装_2026-10-13/client_installation.json)及[GUI导入核验](../验证记录/Lichtblick安装_2026-10-13/gui_verification_2026-10-13.json)。默认布局是从当前客户端GUI实际导出的[模板](../验证记录/Lichtblick安装_2026-10-13/default_layout_export.json)，字段核对官方v1.29.1源码。

官方当前版本与协议依据：[Lichtblick1.29.1发行页](https://github.com/lichtblick-suite/lichtblick/releases/tag/v1.29.1)、[ROS1连接文档](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/frameworks/ros1/)。显示使用Foxglove WebSocket连接，不需要先在Windows安装ROS、WSL或Docker。

## 2. 项目中的安装/启动资料

目录：[监看](../监看/README.md)。

| 文件 | 作用 |
|---|---|
| `bridge_readonly.launch` | 仅启动已有Foxglove Bridge，8765端口；空客户端发布/参数/服务能力和精确话题白名单 |
| `start_monitor_bridge.sh` | 核对当前桥0.6.3、ROS Master、端口和既有桥节点后启动；不安装软件、不杀进程、不启动导航/定位 |
| `camera_preview.launch` | 可选USB相机＋JPEG压缩，5 FPS起步；只感知，不包含舵机、任务、MAVROS设定值或TRT |
| `monitor_layout.json` | 地图/激光/TF/规划俯视＋一路JPEG＋状态字段布局；真实话题/TF与数据流另验收 |
| `validation_2026-10-13.json` | 原补档参数/版本核对、XML/白名单/Bash语法检查；不是运行结果 |

原Nano保存的环境已经包含Foxglove Bridge0.6.3、usb_cam、image_transport和JPEG插件，因此**先核对现机，优先复用已有软件**，无需为此先升级Melodic或下载新ROS2桥。当前安装状态来自导出材料，还没有本轮登录核验。

## 3. Nano连接后完成真实回传的顺序

以下为**待执行步骤**。SHELL开机连接USB后先复查网卡和实际IP；旧USB地址为Nano192.168.55.1、电脑192.168.55.100/24，不假定每次重插都保持接口名称/地址。无线回传则核对双方同网及Nano实际无线IP。

SHELL在[10月12日日志](../../logs/2026-10-12_源码分析、Lichtblick安装与首飞炸机.md)补充：路由器已固定分配Nano无线IP **192.168.1.102**，飞行使用局域网，调试默认USB **192.168.55.1**。对应Lichtblick地址为 `ws://192.168.1.102:8765` 和 `ws://192.168.55.1:8765`；下方USB示例保持原值，切换无线时将 `$nanoAddress` 改为192.168.1.102。该信息来自SHELL记录，不能代替修复后的实际IP/端口验证。

### 3.1 Windows准备与上传

设备修好、开机且USB连接后，在Windows PowerShell核对当前接口：

```powershell
Get-NetAdapter | Select-Object Name,InterfaceDescription,Status
Get-NetIPAddress -AddressFamily IPv4 | Select-Object InterfaceAlias,IPAddress
Test-NetConnection 192.168.55.1 -Port 22
```

如果Nano使用不同的无线IP，把下方变量替换为当前地址；不要直接复用过去的接口索引11或“以太网3”。USB网卡未出现先排查数据线、设备电源和枚举，SSH22不通时不要先调Lichtblick。

确认SSH可达后，创建一个新的监看目录并上传（由SHELL正常输入SSH认证，不需把密码提供给助手）：

```powershell
$nanoAddress = '192.168.55.1'
ssh "jcfh@$nanoAddress" 'mkdir -p /home/jcfh/monitor_display'
scp -r 'D:\project\低空消防\version1018\uav1018\docs\监看\.' "jcfh@${nanoAddress}:/home/jcfh/monitor_display/"

# 查看电脑原稿SHA256，保留用于和Nano副本逐项对照。
Get-FileHash -Algorithm SHA256 -LiteralPath `
  'D:\project\低空消防\version1018\uav1018\docs\监看\bridge_readonly.launch', `
  'D:\project\低空消防\version1018\uav1018\docs\监看\camera_preview.launch', `
  'D:\project\低空消防\version1018\uav1018\docs\监看\start_monitor_bridge.sh'
```

也可用FinalShell已登录SFTP面板上传同一目录；不要覆盖 `/home/jcfh/catkin_ws` 或把Windows安装器复制到Nano。Nano终端核对：

```bash
cd /home/jcfh/monitor_display
sha256sum bridge_readonly.launch camera_preview.launch start_monitor_bridge.sh
```

`bash start_monitor_bridge.sh`不依赖文件可执行位；保持UTF-8/LF，Windows不要将脚本另存成CRLF。

### 3.2 检查ROS并分别启动

1. 登录Nano，读取 `rosversion foxglove_bridge`、当前ROS Master/节点与8765监听情况。已运行桥时查看它的参数和能力，避免重复启动或自动杀掉已有进程。
2. 将监看目录的launch、脚本和说明上传到新的 `/home/jcfh/monitor_display/`，保留原机工作空间和源码；上传后核对字节/哈希。
3. 若已有受控数据源，只启动独立桥；无ROS Master时先另开窗口运行roscore。数据桥不生成地图或相机图像。
4. 没有图像数据且设备空闲时，可单独启动camera_preview；若相机已被占用，只复用已有mission/SSD JPEG，或按下方命令把已有raw另行压缩到preview出口，不再启动第二个usb_cam。
5. Windows确认8765可达，Lichtblick选Foxglove WebSocket，填该Nano实际地址；加载项目布局并检查类型/帧/频率。
6. 先验地图+TF+scan，再验一路JPEG，记录输入频率、桥接带宽和画面延迟；无线链路另测，不用USB结果代替无线验收。

Nano示例命令（**本轮未执行**）：

```bash
# 首先核验现有软件与ROS，不启动完整任务。
source /opt/ros/melodic/setup.bash
rosversion foxglove_bridge
rosnode list
ss -ltn 'sport = :8765'
```

已有ROS Master后，桥窗口：

```bash
bash /home/jcfh/monitor_display/start_monitor_bridge.sh preview
```

脚本默认读取 `/home/jcfh/catkin_ws/devel/setup.bash`。若返修后工作空间路径发生变化，先核对真实setup文件，再通过 `MONITOR_WORKSPACE_SETUP=/实际路径/devel/setup.bash bash /home/jcfh/monitor_display/start_monitor_bridge.sh preview` 指定；版本不再是0.6.3时须重核参数，不能把拒绝启动当作已兼容。

桥启动后，在另一Nano窗口核对以下四项，应全部返回空列表；否则先停止本次桥并检查是否启动了正确配置，再进行客户端验收：

```bash
source /opt/ros/melodic/setup.bash
rosparam get /foxglove_bridge/capabilities
rosparam get /foxglove_bridge/client_topic_whitelist
rosparam get /foxglove_bridge/param_whitelist
rosparam get /foxglove_bridge/service_whitelist
```

这些参数核验不代替实际握手和拒绝能力验证。

若 `rosnode list`明确显示无法连接Master，且没有另一个受控ROS会话，另开Nano窗口先运行：

```bash
source /opt/ros/melodic/setup.bash
roscore
```

保留此窗口，其他窗口连接同一个Master。若Master本已正常，不要重复运行roscore；不为通过脚本检查而启动完整导航任务。

可选相机窗口（设备空闲、无机构动作）：

```bash
source /opt/ros/melodic/setup.bash
roslaunch /home/jcfh/monitor_display/camera_preview.launch
```

若已有usb_cam输出 `/camera/image_raw`、但没有mission/SSD JPEG，不启动camera_preview。先用 `rostopic list` 和 `rostopic hz /camera/image_raw` 核实真实输入；另开窗口仅运行压缩转发，桥仍选preview：

```bash
source /opt/ros/melodic/setup.bash
rosparam set /monitor/camera/image/compressed/format jpeg
rosparam set /monitor/camera/image/compressed/jpeg_quality 50
rosrun image_transport republish raw in:=/camera/image_raw compressed out:=/monitor/camera/image __name:=monitor_image_compressor
```

输入若为其它话题，替换 `in:=` 后的名称。该命令不打开相机设备，但会增加编码负载；原采样率高时先测CPU和带宽，不将这条转发误称为5FPS限流。不要同时运行camera_preview里的同名压缩节点。

Windows连接示例（先确认地址是当前Nano）：

```powershell
Test-NetConnection $nanoAddress -Port 8765
```

Lichtblick连接类型：**Foxglove WebSocket**；示例URL：`ws://192.168.55.1:8765`。

### 3.3 客户端布局与验收

1. 打开Windows开始菜单中的Lichtblick，或运行已安装的 `C:/Users/alexh/AppData/Local/Programs/lichtblick/Lichtblick.exe`。
2. 左侧Layouts选择已导入的 **monitor_layout**；若换电脑或布局丢失，选Import from file，导入 `D:\project\低空消防\version1018\uav1018\docs\监看\monitor_layout.json`。
3. 左上Menu → Open connection → Foxglove WebSocket，填写实际Nano地址。不要选Native ROS1或Rosbridge后再填8765，它们是不同协议。
4. Topics中检查 `/map`、`/scan`、`/tf`、图像的实际类型和更新；3D地图面板Display frame=map，若只是看EGO可改odom；Image默认选 `/monitor/camera/image/compressed`，Raw默认 `/mavros/state`。
5. Nano读取 `rostopic hz /scan`、`rostopic hz /monitor/camera/image/compressed`、`rostopic bw /monitor/camera/image/compressed`；记录数据源、接收率、分辨率和延迟。没有启动MAVROS时Raw的state空白正常；仅相机预览时没有map同样正常。

典型问题按层排查：SSH通但8765不通 → 桥是否启动/地址/监听；连接成功无话题 → ROS Master/白名单/数据源；有map/scan但不显示 → frame_id、TF和时间；有视频但积压 → 先只订阅一路JPEG、降低预览FPS/分辨率并测网络。bridge脚本遇已占端口/现成桥会拒绝启动，应先确认现有会话，不能盲目杀进程。

若受控任务链已经在运行，可用脚本参数 `mission` 或 `ssd`复用其一路JPEG，分别选择 `/mission/video/compressed` 或 `/ssd_detection/debug_image/compressed`。不要为取得这些话题而直接启动默认navigation launch；它会初始化真实舵机并发送飞行设定值。

## 4. 地图与视频分别依赖什么

- `/map`是Cartographer当前输出的二维占用地图，图上定位需要scan、IMU和TF。仅启动桥或相机不会自动生成地图。
- 原location launch包含MAVROS、传感器和向飞控发送外部位姿的location_node。首次回传先检查已有数据源；若需要启动建图，应另安排受控传感器/飞控状态验证，避免把它当完全无写入的显示工具。
- 默认规划是odom系、地图是map系；同时显示需map→odom→base_link→laser_link完整且有时效的TF。Lichtblick使用Display frame字段，不能只照搬旧RViz的字段名。
- 单独相机预览JPEG为`/monitor/camera/image/compressed`；camera_link未必在TF链里，因此先在Image面板看图，不能自动投影到3D地图。它不会运行目标检测。
- 只读桥不提供clientPublish、参数改写和服务能力；显示数据仍必须检查服务器实际握手/日志，文件里写了[]不等于现机已生效。

## 5. 本轮结束时的准确状态

**Windows安装和离线配置准备完成；SHELL已确认Nano返修不能开机，Nano部署、桥连接、地图/图像实时流以及无线性能留待设备恢复后由SHELL按本文完成。** 本轮没有启动任何真实飞行任务、舵机或定位节点，也没有修改原源码和模型。

后续最短路径是连接Nano → 上传独立配置并核哈希 → 启动已有数据源上的只读桥 → 客户端接入并逐层验收。无需等待RTX仿真平台配置完成。
