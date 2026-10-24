# README_交回说明（YOLO 首轮训练，接收端 → SHELL）

- 交接包：`YOLO异机训练交接包_2026-10-24`（校验通过：13 个文件 SHA256 全部一致；外层 ZIP `90f3af6f…0015` 与 `.sha256` 一致）
- 工作机：Windows 11 + i5-12400F + 16 GB + **RTX 3060 Ti 8 GB**，无 RK 板、无相机、无无人机
- 工程日期：2026-10-24（Asia/Shanghai）；真实时间戳见 `logs/`
- 本包内所有结论都来自本机实际运行；没有 RK 的部分一律写“待验证”

## 1. 状态总表（不要用一句“训练完成”覆盖全部阶段）

| 阶段 | 状态 | 说明 |
|---|---|---|
| 交接包完整性与素材核查 | **已完成** | 180/180 图片全像素解码、XML 配对/几何/类别/尺寸全查、无重复图、框回画抽查通过 |
| 派生数据与划分 | **已完成** | VOC→YOLO，六类 108/36/36；**类内按采集顺序**划分，非采集组独立（已注明） |
| 训练环境 | **已完成** | 独立 Python 3.10.11 + venv，torch 2.5.1+cu121，GPU 可用 |
| 正式训练（第一轮） | **已完成（试验级）** | YOLOv5s-ReLU 6 类 640，200 epoch；val mAP@0.5 **0.902**（最好）/0.880（最后），mAP@0.5:0.95 0.777/0.761 |
| 冻结测试集评估 | **已完成（只评一次）** | mAP@0.5 **0.722** / mAP@0.5:0.95 **0.634**；但固定阈值下类间混淆严重，见 `docs/evaluation.md` |
| 浮点导出（ONNX/TorchScript） | **已完成** | opset 12，3 个 sigmoid 头；与 PyTorch 头最大差 3e-4、解码框差 0.0004 px |
| RKNN 转换（第一代工具链） | **已完成** | rknn-toolkit **1.7.5**，target `rk3399pro`，8bit 非对称 u8，`.rknn` 7.17 MB |
| 量化精度对照 | **未完成（本机不可能）** | 1.7.5 起 toolkit 取消 PC 模拟器，报 “Simulator is not supported…”，必须上板；比对脚本已交付 |
| RK 目标机验收（加载/延迟/ROS/相机） | **待验证** | 无硬件，未做；清单见 `docs/rk_deployment_checklist.md` |
| 第二轮训练（旋转增强） | **未完成** | 配方与脚本已交付；本轮因单轮耗时从 9 s 升到 90 s，主动中止在 epoch 13 |

## 2. 交付物目录

```
deliverables/
  README_交回说明.md          ← 本文件
  SHA256SUMS.txt             ← 包内每个文件的 SHA256
  docs/
    environment.md           环境、版本、网络限制、源码 commit
    data_pipeline.md         数据核查、坐标约定、划分策略与限制
    training.md              训练配方、种子、耗时、初始化权重限制、第二轮尝试
    evaluation.md            测试集指标、混淆表、业务错误单列、阈值敏感性
    export_quantization.md   ONNX/TorchScript/RKNN 转换与对拍、量化局限
    integration_notes.md     输入/输出/解码/NMS/letterbox 还原/白名单（给 C++ 用）
    rk_deployment_checklist.md  RK 离线部署清单与板端待办
    env_summary.json / pip_freeze_train.txt / pip_freeze_rknn.txt
  config/                    class_map.json、data.yaml、hyp_rot_finetune.yaml、RK_anchors.txt、upstream_sources.json
  scripts/                   01…09 全部脚本 + 4 个可重跑 bat（训练/恢复/导出用）
  models/                    best.pt、best.onnx、best.torchscript、sixclass_yolov5s_relu_rk3399pro.rknn、model_manifest.json
  metrics/                   selected_exp1_warm_coco_silu/（曲线、opt/hyp）、test_valpy/（标准评估）、test_eval/、val_eval/
  reports/                   audit_pre_conversion.json、split_manifest.json、calib_manifest.json、overlays/、rknn_convert_report.json
  logs/                      训练/评估/pip 日志
```

未随包交付（体积大或与本机绑定）：原始 ZIP（866 MB）、派生图片与标签、两个 venv、完整训练缓存。
它们的位置和哈希在 `docs/` 与 `reports/split_manifest.json` 里可追溯；需要时可按下文命令重建。

## 3. 关键哈希（便于核对）

| 文件 | SHA256 |
|---|---|
| 原始素材 `TargetDelection1.1.zip` | `6a9d4537ce546ae5bb0c429f565edaf3f0e44c6e0bd628b7faf5a061c9666b41` |
| 源码 `airockchip/yolov5` @ `d25a075…`  ZIP | `4b398f847229d99bd87b3cbef63b585dabe7280fcaa2bfdf3b398d278ea3cf7e` |
| 初始化权重 `yolov5s.pt`（SiLU COCO） | `8b3b748c1e592ddd8868022e8732fde20025197328490623cc16c6f24d0782ee` |
| `best.pt` | `007d6088f936fbf12d6c39b6cea15c15a04c9bf1f84ccb3455198f70369e4d6e` |
| `best.onnx` | `42f4c1fba3b5aa9716961a5ad87dccb8c6bb67c376c6a03003e3c6b82b297933` |
| `best.torchscript` | `af8ad5ae8041bf9a825e5e1af98751ba3949e10390974dc1adde5e614d4d32cb` |
| `.rknn` | `f922b48bdd34e0ed47b94a8c3975a6b12b58989c8cf4c5816f66cc3a00b6eab1` |

全部文件级哈希见 `SHA256SUMS.txt`。

## 4. 可复现命令（Windows，路径按本机绝对路径写）

```bat
:: 1) 校验交接包
env\python310\python.exe YOLO异机训练交接包_2026-10-24\verify_package.py

:: 2) 素材核查 + 派生数据（含回画审查图）
env\python310\python.exe work\scripts\01_extract_audit.py
env\python310\python.exe work\scripts\02_build_dataset.py

:: 3) 量化校准集（只用训练集）
env\python310\python.exe work\scripts\03_make_calib.py

:: 4) 训练（交付模型，200 epoch）
work\scripts\run_exp1_warm.bat

:: 5) 冻结测试集评估（标准 YOLOv5）
env\venv-train\Scripts\python.exe work\repos\yolov5-...\val.py --task test ^
   --weights work\runs\exp1_warm_coco_silu\weights\best.pt ^
   --data work\dataset\sixclass\data.yaml --img 640 --conf-thres 0.001 --iou-thres 0.6

:: 6) 导出 ONNX/TorchScript（--rknpu，生成 RK_anchors.txt）
env\venv-train\Scripts\python.exe work\repos\yolov5-...\export.py --weights best.pt ^
   --include onnx torchscript --rknpu --img 640 --batch 1 --device cpu --opset 12

:: 7) RKNN 转换（独立环境 env\venv-rknn）
env\venv-rknn\Scripts\python.exe work\scripts\07_rknn_convert.py --onnx best.onnx ^
   --calib work\dataset\calib\dataset.txt --out work\rknn
```

## 5. 必须提醒下游的事实

1. **初始化权重不是 ReLU 预训练权重**：用的是 COCO 的 SiLU 权重做迁移初始化，只迁移形状匹配层。不要对下游声称“已验证的 YOLOv5s-ReLU 预训练模型”。
2. **数据只有 180 张、每类 30 张、单一场景族、每张只有 1 个目标**，测试集与训练集同场景；指标偏乐观，不能作为飞行验收。
3. **固定阈值不可用**：conf 0.25 时测试集 `hazard_toxic` 召回为 0/6，而 AP@0.5 有 0.767——置信度校准很差，上线前必须用更大标定集做按类阈值。
4. **干扰类误触发 5/12**（测试集，conf 0.25）——业务上“干扰项不得触发任务”目前**不满足**。
5. 板端一切未验证：驱动/代理/固件、模型加载、NPU 延迟、量化后精度、ROS 集成、相机与并行负载。
6. 量化模型 `.rknn` 只证明**转换链跑通**，不证明精度；量化前后对比必须在板上做（脚本已给）。

## 6. 建议的下一步

1. 先解决数据：补采不同背景/高度/机载视角/多目标/空场景，尤其三个三角形危险标志的更多样本；对干扰样本完整标注。
2. 用更大的验证集标定按类阈值并定义可接受的误触发/漏检，再决定是否进入 ROS 集成。
3. 第二轮训练用 `config/hyp_rot_finetune.yaml`（旋转增强）重跑，先查清单轮 90 s 的原因（残留 dataloader worker、磁盘缓存、增强开销）。
4. 拿到 RK 板后按 `docs/rk_deployment_checklist.md` 逐项验收；用 `scripts/06_infer_onnx.py`/`08_rknn_compare.py` 做板端与 PC 端逐框对拍。
