# 训练记录

## 1. 结论

- 完成了一轮 **YOLOv5s-ReLU、6 类、640 输入、200 epoch** 的正式训练（`exp1_warm_coco_silu`），验证集
  **mAP@0.5 = 0.902、mAP@0.5:0.95 = 0.777**（最好 epoch 185；最后一个 epoch 0.880 / 0.761）。
- 冻结测试集只评估了一次：**mAP@0.5 = 0.722、mAP@0.5:0.95 = 0.634**，但**按固定置信度阈值使用时，类别混淆严重**
  （有毒/易燃两类尤其差，见 `evaluation.md`）。因此这轮结果只能算**试验结果**，不能当作可交付的检测精度。
- 没有可用的、来源可核验的 YOLOv5s-ReLU 预训练权重；本轮用 COCO 的 **SiLU** 权重做初始化（只迁移形状匹配层），
  这一限制在下面单独说明，并用同配置的“从零训练”做了对照（epoch 39 时 mAP50 0.003）。

## 2. 配方（exp1，交付模型）

| 项 | 值 |
|---|---|
| 结构 | `models/yolov5s.yaml`，`nc=6`，57 个 Conv 层全部 `nn.ReLU`（构建后统计） |
| 参数量 / 计算量 | 7.04 M / 16.0 GFLOPs（640） |
| 初始化 | `yolov5s.pt`（ultralytics v7.0，COCO 80 类，SiLU），`Transferred N/N` 见日志 |
| 数据 | `work/dataset/sixclass`，train 108 / val 36 / test 36（每类 18/6/6） |
| 输入 | 640×640 letterbox，RGB，`/255` |
| 超参 | `data/hyps/hyp.scratch-low.yaml`（mosaic 1.0、HSV、scale 0.5、degrees 0、fliplr 0.5） |
| 优化 | SGD，lr0 0.01，cos/linear 衰减到 0.0001，warmup 3 epoch，batch 16，weight_decay 5e-4 |
| 轮数 / 早停 | 200（上限），patience 50（未触发） |
| 种子 | 42（`--seed 42`） |
| 设备 | RTX 3060 Ti 8 GB，`--device 0`，AMP 开启（自检通过） |
| 实际耗时 | 0.482 h（约 8.7 s/epoch，含验证） |
| 复现命令 | `scripts/run_exp1_warm.bat` |

训练日志：`logs/exp1_warm_train.log`；曲线与混淆矩阵：`metrics/selected_exp1_warm_coco_silu/`；
`opt.yaml` 与 `hyp.yaml` 已随包交付（含实际生效的全部参数）。

## 3. 初始化权重的限制（必须知道）

- 项目的首选是“来源可核验的同架构 YOLOv5s-ReLU 预训练权重”。经检索，**airockchip/yolov5 仓库没有发布任何 release 资产**，
  rknn_model_zoo 提供的是 ONNX 而不是 `.pt`，因此本轮**没有**匹配的 ReLU 预训练权重。
- 替代做法：用 ultralytics v7.0 的 `yolov5s.pt`（SiLU，COCO）初始化，**只按形状迁移**到 ReLU 网络；检测头因类别数不同而重新初始化。
  这是迁移学习初始化，**不等于**“已验证的 ReLU 预训练权重”，激活函数差异会带来特征分布偏差，随后由 200 epoch 微调吸收。
- 对照：`exp2_scratch_relu`（同数据、同超参、随机初始化，seed 42）在 **epoch 39** 时 val mAP@0.5 = **0.003**（mAP50-95 = 0.0009），
  同期 exp1 已在 0.66 以上。该对照因并发评估导致 DataLoader 内存不足中断，未跑满 200 epoch，仅作**同轮次**参考。
  它说明：**180 张图从零训练不可行**，预训练初始化是必要的，而当前只能借助非匹配激活的 COCO 权重。

## 4. 第二轮尝试（未完成，留作下一步）

- 动机：交付模型在 0.25 阈值下把三角标志互相混淆（有毒→干扰/易爆、易燃→干扰/易爆），而照片里标志的**平面旋转**变化很大，第一轮 `degrees=0`。
- 做法：`scripts/run_exp3_rot.bat` + `config/hyp_rot_finetune.yaml`（degrees 15、scale 0.6、translate 0.15、mosaic 0.8、lr0 0.005，
  从 exp1 的 `best.pt` 全参数热启动，`Transferred 349/349`）。
- 实际执行到 epoch 13（val mAP50 0.886 / mAP50-95 0.627），**单轮耗时约 90 s**（第一轮约 9 s，疑似数据增强与多 worker/残留进程导致吞吐下降），
  按当前机器状态跑满 120 epoch 需要约 3 小时，故**主动中止**。已交付可直接重跑的 bat 与 hyp，不把它算作已完成成果。

## 5. 复现要点

1. 环境见 `docs/environment.md`（Python 3.10.11 + torch 2.5.1+cu121，独立 venv）。
2. 数据见 `docs/data_pipeline.md`（`scripts/01`、`02` 重建标签与划分；`split_manifest.json` 记录每文件 SHA256）。
3. 训练：`scripts/run_exp1_warm.bat`（Windows 批处理，路径按本机写成绝对路径，换机需改 `ROOT`）。
4. 注意：本机 `pypi.org` 不可达，pip 用清华镜像；torch 的 CUDA 轮子需从 `mirrors.nju.edu.cn/pytorch` 获取，
   若用 PyPI 的 `torch` 会拿到 CPU-only 轮子（本机第一次就踩到，日志保留在 `logs/pip_torch_install.log`）。
