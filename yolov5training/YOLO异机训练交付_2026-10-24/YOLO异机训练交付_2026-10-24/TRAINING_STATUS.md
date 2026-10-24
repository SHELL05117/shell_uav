# TRAINING_STATUS — 接收端（D:\project\yolotraining）YOLO 首轮训练记录

工程日期按 Asia/Shanghai、次日 08:00 前归前一天归 2026-10-24；真实时间戳见各节与 `logs/`。
交接包 `YOLO异机训练交接包_2026-10-24/` 视为只读输入；派生数据、环境、日志、交付物在本工作目录的
`work/`、`env/`、`logs/`、`deliverables/`。

## 0. 最终状态

| 阶段 | 状态 | 摘要 |
|---|---|---|
| 交接包校验 | 完成 | `verify_package.py` 13/13 文件 SHA256 通过；外层 ZIP 与 `.sha256` 一致 |
| 素材解码与标注核查 | 完成 | 180/180 全像素解码；配对/尺寸/几何/类别全查；无重复图；6 张拼版 + 6 张放大回画审查 |
| 派生数据与划分 | 完成 | 六类 108/36/36（类内按采集顺序，非采集组独立，已注明） |
| 训练环境 | 完成 | Python 3.10.11 + venv，torch 2.5.1+cu121，RTX 3060 Ti 可用 |
| 正式训练 | 完成（试验级） | YOLOv5s-ReLU 6 类 640，200 epoch；val mAP@0.5 0.902（best 185）/0.880（last） |
| 冻结测试集评估 | 完成（一次） | mAP@0.5 0.722 / mAP@0.5:0.95 0.634；固定阈值下混淆严重 |
| 浮点导出 | 完成 | ONNX opset12（3 个 sigmoid 头）+ TorchScript；对拍 head 3e-4、解码框 0.0004 px |
| RKNN 转换 | 完成 | rknn-toolkit 1.7.5、rk3399pro、8bit 非对称 u8；`.rknn` 7.17 MB |
| 量化精度对照 | 未完成 | 1.7.5 取消 PC 模拟器，必须上板；比对脚本已交付 |
| RK 目标机验收 | 待验证 | 无硬件 |
| 第二轮（旋转增强） | 未完成 | 单轮耗时异常（9 s → 90 s），主动中止在 epoch 13；配方与脚本已交付 |

## 1. 时间线（真实时间）

| 时间 | 事件 |
|---|---|
| 2026-10-24 19:57 | 会话开始，工作区权限问题（DSH 沙箱无法在 `D:\project\yolotraining` 授予写权限），诊断后由用户切到完全权限 |
| 20:02 | 交接包校验通过 |
| 20:04–20:06 | 素材全量解码/标注核查（`logs/stage1_audit.log`） |
| 20:06 | 派生数据集与回画审查图（`logs/stage2_build.log`） |
| 20:03–20:50 | 训练环境：Python 3.10.11 → venv → torch（PyPI 的 CPU 轮子踩坑 → NJU 镜像 CUDA 轮子 2.33 GB）→ 依赖 → sympy 版本冲突修复 |
| 20:27 | 3 epoch 冒烟训练通过（数据、六类头、GPU、checkpoint） |
| 20:31–20:33 | 导出 ONNX/TorchScript + RK_anchors.txt（冒烟权重） |
| 20:38–21:00 | 下载 rknn-toolkit 1.7.5 打包（678 MB，多镜像试速）并解出 Windows 轮子 |
| 20:53–21:00 | 建立 Python 3.6 转换环境（rknn 轮子 + tf 1.14 + torch 1.10-cpu + onnx 1.10） |
| 20:55–21:15 | `exp1_warm_coco_silu` 正式训练 200 epoch（0.482 h） |
| 21:06–21:09 | RKNN 转换成功（`.rknn` 7.17 MB） |
| 21:10 | 测试集标准评估（val.py）→ mAP@0.5 0.722 |
| 21:11 | 任务级混淆/失败分析（`work/reports/eval_exp1_test`） |
| 21:13–21:20 | `exp2_scratch_relu` 从零消融（epoch 39 因并发评估内存不足中断 → mAP50 0.003） |
| 21:15–21:45 | `exp3_rot_finetune` 旋转增强微调（epoch 13 主动中止；单轮约 90 s） |
| 21:46–22:05 | 验证集阈值敏感性分析、交付物整理、SHA256 清单 |

## 2. 关键决策与依据

1. **坐标约定**：XML 具 labelImg 指纹（`verified="yes"`、相对 path、pose/truncated/difficult、整数坐标，无框贴边）。按 **0 基连续边界**换算，不做统一 −1；1 px 口径差异对归一化值影响 < 3e-4，已写入 split_manifest。
2. **划分**：每类只有一段连续采集（无 EXIF 时间），采用类内按采集顺序 18/6/6；明确标注 `group_independent: false`，因此指标是试验级。
3. **初始化权重**：搜不到可核验的 YOLOv5s-ReLU `.pt`（airockchip 仓库无 release 资产），改用 COCO **SiLU** 权重做形状匹配迁移，并跑同配置从零消融作对照；文档中不声称“匹配 ReLU 预训练”。
4. **先验证部署链**：在长训练之前先用 3 epoch 权重点通 ONNX 导出与解码；再在正式模型上重做导出/转换。
5. **量化对照**：toolkit 1.7.5 明确不支持 simulator（"not supported after version 1.7.3"），不降级工具链（项目要求锁定 1.7.5），如实记录并交付可上板运行的比对脚本。
6. **第二轮训练**：观察到单轮耗时从 9 s 涨到 90 s（数据增强 + 多 worker/残留进程），为避免交付半成品，停止在 epoch 13，保留配方与脚本供下一轮。

## 3. 实测问题与处理

| 问题 | 处理 |
|---|---|
| DSH 沙箱无法在此目录 grantWrite（目录属主 Administrators、无 WRITE_DAC） | 按沙箱 ACL 诊断流程检查后无法在受限权限修复；用户切完全权限，文件内容/属主未改 |
| `pypi.org` 不可达 | pip 改清华镜像；torch CUDA 轮子从南京大学 PyTorch 镜像取 |
| PyPI 的 Windows torch 是 CPU-only（203 MB） | 识别后改用 cu121 轮子（2.33 GB），并在日志中保留证据 |
| `pkg_resources` 缺失 / sympy 版本冲突导致 YOLOv5 触发 pip 自动安装并卡在 pypi | 固定 `setuptools==69.5.1`、`sympy==1.13.1`，并设 `YOLOv5_AUTOINSTALL=false` |
| Ultralytics 首次运行要下载 Arial.ttf，`ultralytics.com` 返回 308 | 把系统 `arial.ttf` 复制到 `%APPDATA%\Ultralytics\` |
| PowerShell 传空参数 `--weights ''` 被吞掉 | 用 .bat 承载训练命令（同时作为可复现记录） |
| Pillow 12 移除 `FreeTypeFont.getsize`，YOLOv5 画图报错 | 训练环境固定 `Pillow==9.5.0` |
| RKNN `load_onnx` 传 crop 参数 + `mean_values` 报错 | 不裁剪 + 用 1.7.x 的 `channel_mean_value="0 0 0 255"` |
| toolkit 构造缺 tensorflow / torch | 转换环境补 tf 1.14.0、torch 1.10.0+cpu（不动训练环境） |
| 对拍时 TF32 造成 3.6e-2 差异 | 对拍固定 CPU fp32，差异降到 3e-4 |
| exp2 从零消融 DataLoader OOM（并发跑评估） | 记录为截断消融，epoch 39 与 exp1 同期对比 |

## 4. 结论与限制（给下一个接手的人）

- 训练与导出/转换链路**已经在本机跑通**；模型在验证集上 mAP@0.5 约 0.88–0.90。
- 但**测试集上按固定阈值使用**时，三个三角形危险标志互相混淆（有毒召回 0/6 @0.25，AP 仍有 0.767 → 置信度校准差），
  干扰类有 5/12 被误判成业务类。**当前不满足“干扰项不得触发任务”的业务要求**，只能作为首轮试验基线。
- 数据是主要瓶颈：每类 18 张训练图、单一场景、单目标、同场景划分。
- 无 RK 硬件：驱动/代理/固件、板端延迟、量化精度、ROS/相机全部待验证。

## 5. 交付物

`deliverables/`（96 个文件，91 MB，含 `SHA256SUMS.txt`）：
README_交回说明、docs/（环境、数据、训练、评估、导出量化、集成、RK 部署清单）、config/、scripts/、
models/（best.pt / best.onnx / best.torchscript / .rknn / model_manifest.json）、metrics/、reports/、logs/。
