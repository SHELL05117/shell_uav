# RK3399ProD：RKNN调用与YOLO路线评估

> **2026-10-23最新决定：** SHELL确认YOLO重新训练，完全放弃SSD／MobileNet作为训练、对照或回退路线，当前不在本机训练。新提供素材实际为6类各30张，含两个干扰类，见[新方案](YOLO重训与RK离线部署方案_2026-10-23.md)。本文旧四类数据、SSD复用及Debian优先内容按当时评估保留，不覆盖最新材料和已验收的Ubuntu18.04环境。

[返回项目目录](../../logs/低空消防无人机工程日志.md)

记录日期：**2026-10-21**。对象为SHELL已购买、预计10月22日到货的 **Toybrick TB-RK3399ProD，6GB（CPU 4GB＋NPU 2GB）＋32GB eMMC**，本文简称“rk板”。采购和到货预期来自SHELL反馈；本轮未接触实物、安装SDK、转换模型或执行板端推理。结论为官方资料和本地源码的离线评估。

## 1. 推荐结论

**现阶段首选YOLOv5s-ReLU，使用RK3399Pro专属官方示例和RKNN-Toolkit 1.7.x建立可工作的基线；随后把YOLOv5n-ReLU作为节省资源的比较项。** 若必须先选一个版本开始学习和准备数据，选择 **YOLOv5**。具体部署模型优先从官方已有的 **YOLOv5s-ReLU、8位非对称量化、CPU后处理** 开始，原因是RK3399Pro专属仓库已经提供这一组合的模型、转换脚本和C++程序，验证路径最明确。[官方专属YOLOv5示例](https://github.com/airockchip/RK3399Pro_npu/tree/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo)

这属于工程优先级判断，**尚不能证明v5s是本项目精度、速度或功耗最优的模型**。v5n-ReLU需要验证或自行训练；v5s-ReLU若在整机并行运行SLAM和规划时资源不足，再比较更小网络和输入尺寸。只有同一数据集、相同输入尺寸和相同计时方法的实测，才能确定最终选择。不引用RK3588/RK3568的FPS来保证本板表现，也不把标称NPU算力换算成飞行任务帧率。

**SDK必须使用第一代RKNN-Toolkit；不要为这块板安装RKNN-Toolkit2作为目标转换器。** 两代工具链不兼容，官方将RK3399Pro明确归入旧工具链，并另设RK3399Pro专属驱动与运行库仓库。[旧工具链说明](https://github.com/rockchip-linux/rknn-toolkit/blob/8e30ee72e772d6d39f2fcaf4047fead450b3ee89/README.md)、[Toolkit2平台边界](https://github.com/airockchip/rknn-toolkit2#support-platform)

## 2. 已核实的支持边界

| 内容 | 官方资料支持的事实 | 对本项目的含义 |
|---|---|---|
| 模型转换 | 旧RKNN-Toolkit支持RK3399Pro，官方仓库发布版为1.7.5 | 优先建立隔离的1.7.5转换环境；若到货固件配套旧版本，先保留厂商可工作的组合，再评估整体升级 |
| 目标机运行 | 专属仓库提供RKNN C API 1.7.5、`librknn_api.so`、驱动固件与示例 | ROS C++节点可封装C API，不要求目标板安装训练环境 |
| NPU通信 | 依赖`npu_transfer_proxy`；不同板可能采用USB3.0或PCIe通信，固件不同 | 先读实际设备类型，不凭板名猜连接方式，不能混用另一接口的NPU固件 |
| YOLOv5示例 | 专属示例要求Toolkit≥1.7.0，仅支持8位非对称量化输出；后处理移至CPU | 初版按示例量化和输出契约实现；“INT8路线”不表示所有张量的数据类型都为有符号INT8 |
| 内存 | 采购规格为4＋2GB | 不能把6GB统一视为Linux/ROS可用内存；NPU专属内存也不能消除主机到NPU的数据传输开销 |

依据：[旧SDK与wheel清单](https://github.com/rockchip-linux/rknn-toolkit/blob/8e30ee72e772d6d39f2fcaf4047fead450b3ee89/README.md)、[RK3399Pro NPU固件与代理说明](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/README.md)、[专属示例README](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/README.md)。内存划分目前按采购反馈记录，到货后需核验。

不能套用针对新一代SoC的三核NPU绑定、`librknnrt.so`部署、Toolkit-Lite2教程或通用`/dev/rknpu`检查来替代本板专属流程。判据是**模型转换器、目标平台、用户态库、NPU固件/驱动、通信代理和实际推理结果组成的完整组合**，不能只看Python包安装成功。

## 3. YOLO候选比较

| 候选 | 证据与适配工作 | 本轮评价 |
|---|---|---|
| **YOLOv5s-ReLU** | RK3399Pro专属官方仓库有模型、转换和C++示例；需换成本项目四类数据训练的模型，并同步类别/anchors/输出头 | **首个NPU基线和当前默认候选** |
| **YOLOv5n-ReLU** | 模型更小是选它比较的理由；ReLU变体仍需按同一分支训练、导出和板端验证，不能声称已有本项目可用权重 | **基线跑通后的优先优化候选**；重点检查小目标和危险品类别漏检 |
| YOLOv3-tiny | 旧Toolkit官方Darknet示例保留tiny参数提示；仍需配置、权重、输出层和anchors对齐 | v5受阻时的备选；不因版本旧自动判定更可靠或更快 |
| YOLOv6n／YOLOv7-tiny | 当前Model Zoo支持表列出RK3399PRO，但须再核对旧工具链导出、C API与量化行为 | 可作后续比较，不增加首轮变量 |
| YOLOv8n／YOLO11n及其他新版本 | 当前Model Zoo表中也列出若干新模型与RK3399PRO；但总安装说明偏向Toolkit2，部分构建/转换入口未列该平台 | 不能直接说“不支持”，也不能把支持表等同本板即装即用；暂不作为首轮恢复路线 |
| 原SSD-MobileNet V1 ONNX | 本地已有模型和标签、CPU/OpenCV实现；原TensorRT后端不可直接迁移 | 留作相同图像的功能/精度比较基线，YOLO路线失败时仍有排查参照 |

支持表只说明存在官方适配线索，不能代替这块板的可运行验证。当前Model Zoo的YOLOv5 `convert.py`提示平台包含rk1808/rv1109/rv1126却未列rk3399pro；CMake切换到RKNPU1的条件也未列rk3399pro。故本项目首轮采用**专属仓库示例**，不用最新通用命令加一个平台参数就宣称支持。[Model Zoo支持表](https://github.com/airockchip/rknn_model_zoo/blob/837db8e992815ff8647eadb709f07067f4bc3a71/README.md)、[当前转换脚本](https://github.com/airockchip/rknn_model_zoo/blob/837db8e992815ff8647eadb709f07067f4bc3a71/examples/yolov5/python/convert.py)、[当前CMake](https://github.com/airockchip/rknn_model_zoo/blob/837db8e992815ff8647eadb709f07067f4bc3a71/examples/yolov5/cpp/CMakeLists.txt)、[旧Toolkit YOLOv3示例](https://github.com/rockchip-linux/rknn-toolkit/blob/8e30ee72e772d6d39f2fcaf4047fead450b3ee89/examples/darknet/yolov3/test.py)

**ReLU与SiLU不能仅通过导出时替换而视为同一个模型。** 官方适配分支说明，移出后处理和部分结构优化可以保持计算结果，而SiLU改为ReLU会改变结果、只应在训练新模型时使用。因此不把现成SiLU权重改激活后直接用于任务，也不把COCO演示模型当成本项目危险品模型。[官方RKNN优化导出说明](https://github.com/airockchip/yolov5/blob/d25a07534c14f44296f9444bab2aa5c601cdaaab/README_rkopt.md)

“YOLOv5”是算法系列；具体工程版本应同时记录官方适配仓库commit、模型配置、激活函数、输入尺寸、导出脚本、opset、后处理和SDK版本。不能只写“用了v5.0/v7.0”，更不能把YOLOv5和后续YOLO版本的发布编号混为一谈。当前官方适配分支说明其基于Ultralytics YOLOv5 v7.0的指定commit；本项目仍以专属示例先验，不强行要求旧示例权重兼容任意最新训练依赖。

## 4. 现有程序可以复用什么

本地导出基线为SSD-MobileNet **V1**，不是旧规划提到的V2。当前模型输入为`[1,3,300,300]`，输出scores/boxes；它与YOLO检测头及后处理不相同。不能把SSD的ONNX改文件名变为YOLO，也不能把TensorRT engine刷入rk板后由NPU执行。详见[总技术指南第5节](../技术文档.md)。

本地导出模型的标签为 5 类（原 `开发工作区/catkin_ws/data/models/real4/labels.txt`，该镜像已随开发工作区移除；现 `src/ssd_detection/models/labels.txt` 为后续扩展的 11 类标签，与导出模型的 5 类输出不一致，不能直接互换）：

| 旧SSD标签索引 | 类别 | 任务区 |
|---|---|---|
| 0 | BACKGROUND | SSD背景类，不作为YOLO目标类 |
| 1 | person_injured | person |
| 2 | hazard_toxic | hazard |
| 3 | hazard_flammable | hazard |
| 4 | hazard_explosive | hazard |

YOLO初版可采用四类、0起始索引，保持这四个名称并建立显式映射；不要将旧SSD的1至4索引直接当YOLO的0至3索引。最终任务类别以SHELL确认的数据集与赛项要求为准。COCO权重没有后三类业务语义，不能由改标签文件生成识别能力。

**SHELL答复后补充：此前四类图片与标注全部保留。** 本轮据此将数据集状态由“是否保留待确认”更新为“SHELL报告已保留”；尚未取得或盘点其存放位置、标注格式、类别顺序、每类数量及训练/验证/测试划分。先检查旧数据是否已符合目标YOLO格式；不符合时在副本中转换，并核验类别索引、框坐标、图片对应关系和异常框，不能默认已经能直接训练。训练脚本和训练权重是否保留仍待确认；本次仅完善文档，没有读取数据、转换标注或训练模型。

建议保留相机输入、检测消息和任务消费接口，把Jetson专属实现替换为新的RKNN后端：

```text
/camera/image_raw（已有usb_cam）
  → 最新帧队列（建议容量1）
  → RGB/缩放或letterbox/按模型约定归一化
  → RKNN C API推理
  → CPU解码/反量化/NMS/还原原图坐标
  → VisionDetectionArray（原始采集时间、frame_id、四类名称、person/hazard、0..1框）
  → 现有MissionVision（需另修新鲜度与帧关联）
```

消息原件：[VisionDetection.msg](../../src/ssd_detection/msg/VisionDetection.msg)、[VisionDetectionArray.msg](../../src/ssd_detection/msg/VisionDetectionArray.msg)；消费者：[mission_vision.cpp](../../src/navigation_2d/src/mission_vision.cpp)。当前消费者按`zone`判定人员/危险品，框按图像宽高换算；新后端应先还原letterbox的缩放和padding，再归一化到**原图**，否则任务对齐会偏移。

新节点继续发布现有消息，可以减少任务逻辑改动，但名称保留`ssd_detection`不代表底层仍是SSD。是否重命名包/话题待实施时确定，不在本轮改源码。旧消息注释中的“两后端精度等价”“置信度排序”等说法不能代替实现核对。

## 5. NPU如何调用：电脑转换，板端推理

### 5.1 三个环境分开准备

| 环境 | 职责 | 建议与限制 |
|---|---|---|
| RTX3060Ti电脑训练环境 | 数据整理、训练/微调、浮点验证 | 与旧转换SDK分开隔离；GPU用于训练，不代表转换一定需要GPU |
| x86_64 Linux转换环境 | RKNN-Toolkit 1.7.5、模型导入、量化、导出 | 官方有Python3.8 Linux x86_64 wheel及Ubuntu20.04/py38依赖文件；这是可评估起点，不是已配置验收。固定依赖并保存镜像/日志，避免把现代PyTorch训练包强装进同一环境 |
| rk板aarch64运行环境 | Linux BSP、NPU代理/固件、`librknn_api.so`、检测节点 | 初版优先厂商Debian10 BSP；不要求安装PyTorch训练环境。C++ API可减少ROS Python版本冲突，但系统/驱动兼容仍需实际验证 |

官方wheel与依赖依据：[1.7.5发布说明](https://github.com/rockchip-linux/rknn-toolkit/blob/8e30ee72e772d6d39f2fcaf4047fead450b3ee89/README.md)、[Ubuntu20.04/py38依赖文件](https://github.com/rockchip-linux/rknn-toolkit/blob/8e30ee72e772d6d39f2fcaf4047fead450b3ee89/packages/requirements-cpu-ubuntu20.04_py38.txt)。本轮未测试安装，包下载/旧依赖可用性、容器支持及准确Python版本在环境实施时记录。

### 5.2 板到货先确认NPU可用

在匹配BSP、尚未启动无人机任务的独立终端中，先做只读识别：

```bash
uname -m
cat /etc/os-release
ps -ef | grep '[n]pu_transfer_proxy'
npu_transfer_proxy devices
```

若工具不在PATH，先定位厂商实际安装位置。保存输出：系统/内核、NPU代理版本或文件哈希、设备ID及USB/PCIe连接类型。随后运行厂商配套的最小模型示例，再核对C API返回的SDK/驱动版本。`devices`能列出NPU只是通信识别成功；还需真实模型运行成功才算推理链路可用。[官方代理和接口区分说明](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/README.md)

如果预装版本先能工作，保留这份版本清单和演示结果；不要先替换系统内核/代理/NPU固件。若要升级到1.7.5，应按官方匹配要求把相应运行库、通信代理、NPU固件作为整体评估；NPU的`boot.img`不是随意取通用rk板Linux镜像代替，USB/PCIe固件路径也必须与实物一致。

### 5.3 第一轮先跑官方示例

采用[RK3399Pro专属YOLOv5目录](https://github.com/airockchip/RK3399Pro_npu/tree/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo)，先验证其`yolov5s_relu_out_opt.rknn`和`bus.bmp`。电脑可按配套脚本交叉编译，但应匹配目标系统ABI和aarch64工具链；也可在目标机独立临时目录评估原生构建。均不将旧Nano的build/devel复制过来。

须先修正或核对官方示例中的几个不一致之处，再执行：

1. README的安装/运行示例误写`rknn_ssd_demo_Linux`，而CMake和脚本实际产物是`rknn_yolov5_demo_Linux`；按实际输出操作。
2. 配套`pytorch2rknn.py`默认`platform="rk1808"`，本项目必须按目标明确设为`rk3399pro`并保存转换配置，不能默认转换其他SoC模型。
3. 脚本实际调用`load_pytorch`，输入为`[3,640,640]`；注释却写“Load ONNX”。区分TorchScript/普通训练checkpoint/ONNX，不能把任意`.pt`直接塞给导入器。
4. 示例预编译模型便于首验；自定义模型应先导出普通RKNN验证。预编译进一步绑定驱动，不能把不匹配错误当“YOLO算法不能用”。

依据：[CMake实际安装名](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/CMakeLists.txt)、[交叉编译脚本](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/build_linux.sh)、[转换脚本](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/convert_rknn_demo/yolov5/pytorch2rknn.py)、[运行脚本](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/run_demo.sh)。上述修正针对后续示例副本，本轮没有改下载仓库或本地项目。

### 5.4 任务模型训练、转换和检查

1. **盘点已保留的四类图片/标注，核验或按需转换为YOLO格式并建立固定验证集。** 先确认位置、原格式、类别顺序、每类数量及既有划分，保留原件并在副本上按需转换、抽查叠框；记录拍摄设备、场地、目标尺寸/距离、光照、模糊、遮挡及负样本；按拍摄批次或视频划分训练/验证/测试，避免相邻帧泄漏。
2. **冻结可导出的ReLU训练分支。** 先用少量样本验证“训练权重→浮点导出→旧Toolkit导入”能贯通，再投入完整训练；保存commit、配置、类别顺序及权重哈希。
3. **按RKNN适配导出。** 官方适配分支提供`--rknpu`导出并产生`RK_anchors.txt`；保留检测头输出，将不利量化的后处理移到CPU。选择ONNX或配套TorchScript导入路线时，以1.7.5能实际读取、输出可比较为准；不强制使用当前训练工具默认opset。
4. **配置量化并导出RKNN。** 使用旧`rknn.api.RKNN`：`config(target_platform=['rk3399pro'], ...) → load_onnx/load_pytorch → build(do_quantization=True,dataset=...) → export_rknn → release`。每步检查返回值。校准图片应来自实际场景并覆盖四类、背景和光照；官方bus一图只够演示，不足以作为项目校准集。
5. **逐层定位精度变化。** 对比PyTorch浮点、导出浮点、RKNN模拟/板端量化输出；先核对RGB/BGR、缩放、padding、0..1或其他归一化契约，再比较分类与框差异。不能沿用旧SSD的[-1,1]预处理给YOLO。
6. **板端C++后处理和消息映射。** 将anchors、stride、类别数/输出通道数、阈值、量化zero point/scale与模型匹配。同步修改官方示例中80类常量，不能只换labels；保存调试叠框及原始数值。

导出依据：[官方RKNN优化分支](https://github.com/airockchip/yolov5/blob/d25a07534c14f44296f9444bab2aa5c601cdaaab/README_rkopt.md)。具体命令在取得实际训练权重与能工作的转换环境后固化；本轮给的是可核验实施顺序，不是假定模型已经转换成功。

### 5.5 板端C API调用顺序

```text
加载.rknn字节
  → rknn_init
  → rknn_query：SDK/driver、输入输出数量、属性/量化参数
每帧：
  → 预处理并准备符合模型约定的CPU缓冲区
  → rknn_inputs_set
  → rknn_run
  → rknn_outputs_get
  → CPU解码、反量化/NMS与坐标还原
  → rknn_outputs_release
结束：
  → rknn_destroy
```

以[本板1.7.5 C API头文件](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/librknn_api/include/rknn_api.h)和[专属C++实现](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/examples/c_demos/rknn_yolov5_demo/src/main.cc)为调用依据；完整手册入口为[Rockchip RK3399Pro RKNN API V1.7.5中文版](https://github.com/airockchip/RK3399Pro_npu/blob/8858114555f1700e41e185b14ec3d266342f8059/rknn-api/doc/Rockchip_RK3399Pro_User_Guide_RKNN_API_V1.7.5_CN.pdf)。本轮直接核读README、转换/C++源码及头文件；PDF作为后续完整接口手册入口，不声称已逐页审阅。

这是普通CPU内存输入的C API路线，不能照抄Jetson中要求CUDA设备内存的接口契约。是否使用RGA、零拷贝或输出优化作为第二阶段性能工作；初版先把图像与结果逐帧核对。

**特别注意异步结果。** 官方示例初始化包含`RKNN_FLAG_ASYNC_MASK`，头文件说明可能取得上一帧输出。无人机任务初版建议使用同步模式，随后再评估异步；若保留异步，必须用返回的`frame_id`对应采集时间、相机朝向和原始图像，不能把上一帧框盖上“当前时间”发给控制逻辑。该项是根据官方API与本项目任务消费者作出的适配建议，并非实测故障定位。

## 6. 验收矩阵：决定最终是否使用v5s

| 层级 | 试验 | 应保存的结果 | 当前状态 |
|---|---|---|---|
| BSP/NPU | 配套最小模型、代理设备与版本查询 | BSP/内核、CPU架构、SDK/驱动/固件/代理及哈希、返回码 | 待板到货 |
| 官方YOLO | 固定bus图、配套模型、重复运行 | 框/类别/置信度、叠框、首次加载和稳定推理耗时、错误日志 | 未执行 |
| 四类浮点 | 同一固定测试集比较原SSD与候选YOLO | 分类别precision/recall、漏检/误检、框位置与阈值；样本数量和划分 | SHELL报告旧图片/标注全部保留；位置、格式、类别顺序、每类数量及划分待盘点，YOLO转换未执行 |
| 量化一致性 | 浮点、模拟及板端量化对比 | 分类别精度变化、预处理/后处理一致性、少量原始张量核对 | 未执行 |
| 分辨率/模型 | v5s-ReLU与v5n-ReLU；建议先比较320/416，必要时640 | 小目标召回、端到端P50/P95/P99延迟、CPU/内存、温度及连续运行情况 | 比较建议，输入尺寸未锁定 |
| ROS接口 | 固定图回放、无目标、遮挡、断相机、推理失败 | header采集时间、帧号、框/类别/zone、失败与空检测分开、结果年龄 | 未实施 |
| 整机负载 | 同时SLAM、EGO、任务通信和监看 | 控制/定位间隔、检测结果年龄、丢帧、温度、资源与负载下的异常 | 未实施 |
| 任务行为 | 先离线/仿真，再台架检查视觉等待与失效 | 旧结果不会重复累积为稳定帧；超时/无目标/后端失败有明确状态 | 未实施 |

输入尺寸是待比较的试验点，不是性能保证；改变尺寸通常需要重新导出/转换并核对检测头，不能只改相机分辨率。先用官方640示例验证链路，再使用符合训练与部署约定的候选尺寸。计时必须包含采集排队、预处理、传输/NPU、取回和CPU后处理；`rknn_run`耗时不能直接称作端到端延迟或稳定任务FPS。

最终选型顺序建议为：**类别/小目标召回达标 → 框定位和置信度可靠 → 结果足够新 → SLAM/规划并行仍稳定 → 再比较功耗和帧率**。具体阈值要结合SHELL的目标尺寸、相机视角、飞行速度和任务允许等待时间确定，本轮不编造验收数值。原识别停滞、撞墙和返航失败仍需独立排查，不能因YOLO或新板上线自动宣布解决。

## 7. 版本与资料留存

首次成功后至少记录：板型/版本、BSP镜像来源与哈希、系统/内核、NPU连接方式、固件/代理/运行库版本及哈希、Toolkit wheel版本/哈希、Python/依赖、训练与导出仓库commit、模型配置/类别/输入尺寸、训练/导出/RKNN文件哈希、校准集清单、后处理参数及结果。不要仅保存一个`.rknn`，也不要将预编译模型等同于可跨驱动通用模型。

本轮2026-10-21在线核查到的版本定位如下，后续下载仍需自行校验并记录实际文件；网页主分支可能继续变化。

| 仓库 | 本轮定位commit | 用途 |
|---|---|---|
| rockchip-linux/rknn-toolkit | `8e30ee72e772d6d39f2fcaf4047fead450b3ee89` | 第一代1.7.5工具链、wheel与旧示例 |
| airockchip/RK3399Pro_npu | `8858114555f1700e41e185b14ec3d266342f8059` | 本板固件、代理、C API和YOLOv5示例，首选依据 |
| airockchip/yolov5 | `d25a07534c14f44296f9444bab2aa5c601cdaaab` | RKNN优化导出与ReLU训练提示 |
| airockchip/rknn_model_zoo | `837db8e992815ff8647eadb709f07067f4bc3a71` | 较新模型支持线索，须逐个核对旧平台适配 |

## 8. 等SHELL回来后补充的事项

- 已知旧四类图片与标注全部保留；接续需提供存放位置、标注格式、类别顺序、每类数量及既有划分，并确认训练脚本和训练权重是否保留。先盘点和转换标注；官方演示模型仍可用于独立验证NPU。
- rk板到货时预装的系统、内核、NPU SDK/驱动版本及官方演示是否可运行？本轮暂不要求先升级。
- 实际相机型号、分辨率/帧率、任务高度与最小目标尺寸是什么？这些决定输入尺寸和精度验收条件。
- 最近现场部署的识别/任务源码是否与本地导出一致？当前新鲜度与错误状态建议来自本地基线，需与现场版本核对。

本文件已完成资料评估和实施/验收路线整理；以上缺口不阻碍先准备官方SDK和数据检查，但尚不构成系统、模型或飞行验收。
