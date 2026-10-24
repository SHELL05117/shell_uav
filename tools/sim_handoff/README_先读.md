# 1024全链仿真异机交接

本包供SHELL在另一台已配置Docker和WSL的Windows电脑接续。工程日期2026-10-24，实际打包于2026-10-25凌晨。

## 先做什么

1. 将整个ZIP复制到另一台电脑，用ZIP旁的SHA256文件核对压缩包，再完整解压。
2. 阅读本说明，将[接收端提示词](另一台电脑执行提示词.md)全文发给那台电脑的agent，并告诉它解压后的绝对路径。
3. 让接收端agent先核对包内清单和机器环境，再分阶段配置；不要直接运行旧README中的自动任务命令。

## 资源位置

| 路径 | 内容 |
|---|---|
| `project/` | 原项目1024标签的完整已跟踪快照，源码、模型、场景、工具、日志、证据均保持原字节；不带`.git`历史 |
| `project/src/` | 8个ROS包和catkin顶层声明，唯一源码入口 |
| `project/src/gc_2026_sim/` | 室内场地、虚拟传感器、PX4 SITL接入、投放模拟和检查脚本 |
| `project/yolov5training/` | YOLO配置、训练交付脚本／报告、ONNX／TorchScript／RKNN导出模型与清单 |
| `project/docs/源码分析/` | 当前控制、视觉、定位与Gazebo静态分析和已知问题 |
| `project/docs/监看/` | Lichtblick布局、桥接说明和既有验证记录 |
| `inputs/TargetDelection1.1.zip` | 原始180张JPG、180份VOC XML，完整原件；仅作图像验证和素材来源，不要求重新训练 |
| `handoff/` | 本轮新增交接说明、提示词、依赖清单、检查脚本、Docker模板和校验工具 |
| `BASELINE.json` | 1024提交、工程日期、包结构和外部依赖边界 |
| `MANIFEST.json`、`SHA256SUMS.txt` | 包内载荷逐文件SHA256与字节数；这两个清单本身不自包含 |

源码基线：`2f1853ba9b75f9a912541ec2a8df1d1c767d108f`。包不是Git克隆。需要提交接收端改动时，可在`project/`初始化本地Git仓库，以原样快照作首次提交并在新任务分支开发；原工程的历史在发送端保留，不把初始化出的提交冒充原1024提交。

模型相对目录为：

`project/yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/`

其中`models/best.onnx`供PC推理适配，`config/class_map.json`是六类和业务／干扰映射，`config/RK_anchors.txt`是解码依据；RKNN用于RK目标机，不能直接作为x86 PC的检测后端。

## 当前真实状态

- RK端通用构建、相机／雷达／MAVROS接收已通过；NPU与六类图片推理已运行，自启通过一次重启。
- 仿真环境和全链尚未在接收电脑验收。Docker／WSL已装只是起点。
- 仿真入口仍自动包含旧SSD节点。SHELL已放弃SSD／MobileNet，接收端须新增YOLO适配，不能把模型路径直接换成YOLO便称完成。
- 六类模型只是试验基线，有漏检和干扰误框，业务识别未通过。测试失败应如实记录。
- 飞行全程室内，GPS室外定位不是当前仿真前置；TFmini／PCA的实物缺件不阻塞纯软件仿真。

## 环境选择与下载

先读取已有ROS、Gazebo、PX4和容器，再确定组合。兼容起点是Ubuntu18.04／Melodic／Gazebo Classic9／PX4 v1.13.3；已有Noetic环境时先评估Ubuntu20.04／Noetic／Classic11的可用性及源码兼容性，不覆盖已存在的环境。

项目`start_px4_slam.sh`实际默认`ROS_DISTRO=noetic`。若使用Melodic，必须显式设置`ROS_DISTRO=melodic`；先核对PX4 `Tools/setup_gazebo.bash`、`Tools/sitl_gazebo`等路径是否真实存在。新Gazebo／ROS2不能原样替代这套ROS1／Gazebo Classic接口。

本包不是完整离线软件源：Docker基础镜像、ROS／Gazebo软件包、PX4及递归子模块、Cartographer／bridge和PC推理依赖仍需联网获取。具体见[依赖与下载](环境与外部依赖.md)。不含临时固件、RK内核构建缓存、真实凭据、训练检查点或机体运行录制。无需安装RK的NPU上电服务、刷写工具或Jetson CUDA栈来运行PC仿真。

## 完整性与环境检查

在解压后的包根目录，使用Python 3.10+运行：

```text
python handoff/verify_package.py .
```

没有Python时先核对Windows压缩包校验值，之后让接收端agent在WSL／容器内执行同一工具。Windows检查脚本为`handoff/inspect_host.ps1`，Linux／容器检查脚本为`handoff/inspect_linux.sh`；二者只读，不安装软件、启停服务、修改WSL或接入真实设备。

Dockerfile为**未在本机构建的基础编译模板**，只处理通用编译依赖。Gazebo渲染、PX4工具链、Cartographer、bridge和YOLO接入仍按阶段补齐。WSLg／显示socket／OpenGL必须在目标环境实测，CUDA可用不能替代图形验收。

## 分层验收

机器与依赖 → 8包构建／单测 → PX4＋MAVROS控制 → 场景与虚拟相机／雷达 → TF／SLAM／规划 → YOLO图像与消息 → 投放／返航／降落 → 失败分支／全负载与监看。

前三层可先做，YOLO适配后接完整任务。保留每层版本、命令、退出码、日志和未覆盖项；不把Gazebo真值辅助当作独立定位已通过，不把模拟ACK当作物资实际生成。
