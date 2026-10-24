# 集成说明：输入/输出、后处理与业务白名单

本文件是给 C++（RK3399Pro 第一代 RKNN C API）与 ROS 侧实现用的接口约定。数值来自本次实际导出与验证，不是示意。

## 1. 模型输入

| 项 | 值 |
|---|---|
| 形状 | `1 × 3 × 640 × 640`，float32（RKNN 量化模型内部为 8bit 非对称 u8 输入） |
| 通道顺序 | **RGB**（导出时 `reorder_channel="0 1 2"`，不做 BGR 交换） |
| 数值范围 | `pixel / 255.0`，均值 0，标准差 255（toolkit `channel_mean_value="0 0 0 255"`） |
| 预处理 | letterbox：等比缩放到 640 长边，剩余区域用灰色 `(114,114,114)` 填充并居中 |

letterbox 的关键量（必须与训练一致）：

```
r  = min(640 / W, 640 / H)
nw = round(W * r), nh = round(H * r)        # 取自 PIL 实现，见 scripts/06_infer_onnx.py
canvas = 640x640, 灰 114
dx = (640 - nw) // 2,  dy = (640 - nh) // 2
```

画回原图坐标（推理后）：

```
x_orig = (x_letterbox - dx) / r
y_orig = (y_letterbox - dy) / r
并按 [0, W] / [0, H] 裁剪
```

## 2. 模型输出（`export.py --rknpu` 的约定）

三个张量，每个都**已经过 sigmoid**（RK 分支在 Detect 里对每个 head 做 `torch.sigmoid` 后直接输出，不含 NMS、不含网格解码）：

| 序号 | 形状（640 输入） | 对应 stride |
|---|---|---|
| 0 | `1 × 33 × 80 × 80` | 8 |
| 1 | `1 × 33 × 40 × 40` | 16 |
| 2 | `1 × 33 × 20 × 20` | 32 |

通道布局：`3 个 anchor × (x, y, w, h, obj, c0..c5)` = `3 × 11 = 33`。
ONNX 里的输出名是 `output0 / 286 / 288`，**不要按名字取，按空间尺寸（80/40/20）排序**。

anchor（像素单位，来自导出时生成的 `RK_anchors.txt`）：

```
level0 (stride 8) : [10,13] [16,30] [33,23]
level1 (stride 16): [30,61] [62,45] [59,119]
level2 (stride 32): [116,90] [156,198] [373,326]
```

## 3. 解码（与 `scripts/06_infer_onnx.py` 完全一致）

对每个 level：

```
grid_x, grid_y = 特征图上像素索引 (0..79 / 0..39 / 0..19)
score = obj * max(cls)                # sigmoid 已在模型内完成
if score < conf_thres: skip
cx = (sx * 2 + grid_x) * stride       # 注意不是 YOLOv3 的 "-0.5"
cy = (sy * 2 + grid_y) * stride
w  = (sw * 2)^2 * anchor_w
h  = (sh * 2)^2 * anchor_h
box = [cx - w/2, cy - h/2, cx + w/2, cy + h/2]   # letterbox 坐标系
```

NMS：**按类别分别做**（不同类别互不抑制）。验证时使用的阈值：`conf 0.25`、`IoU 0.45`（仅报告用工作点，不是业务阈值）。实现方式与 YOLOv5 `non_max_suppression` 相同：按置信度降序，逐类别的 IoU 阈值抑制。

## 4. 六类与业务白名单

| ID | 名称 | 业务 |
|---:|---|---|
| 0 | person_marker | person |
| 1 | hazard_toxic | hazard |
| 2 | hazard_flammable | hazard |
| 3 | hazard_explosive | hazard |
| 4 | distractor_1 | **忽略，禁止触发任务** |
| 5 | distractor_2 | **忽略，禁止触发任务** |

业务适配层只能用 ID 0–3 映射到任务；**不得**把“不是 person 的所有框”当作 hazard，也不得把干扰类映射成危险品。类别顺序固定，不能重排。

## 5. 与其他实现的数值一致性（已实测）

| 比较 | 结果 |
|---|---|
| ONNX 三个 head vs PyTorch `Conv+sigmoid`（同一 640 输入，CPU fp32） | 最大绝对差 `3.0e-4` |
| 用同一解码函数解 ONNX 与 PyTorch head，最高分候选框 | 坐标最大差 `0.0004 px`，置信度差 `3e-6` |
| 两侧各自 NMS 后逐框匹配 IoU | 均值 0.73、最小 0.48（差异来自两套 NMS 对近似重复框的取舍，不是坐标误差） |

结论：导出链与解码公式可以放心移植；C++ 侧要复刻的是**按类别 NMS**，否则边界场景会出现与 Python 参考实现不同的框集合。

## 6. ROS 侧约定（计划，供目标工程实现）

- 话题建议中性名 `/vision/detections`，消息含：采集 header（采集时间，不是推理完成时间）、原图宽高、`class_id/class_name`、置信度、原图归一化 `xyxy`。
- 只保留**最新帧**结果；不得把旧结果累积成“稳定识别”。
- 无目标、推理失败、相机断开要分别报告，不能都用“空检测”表示。
- 无 RK 实物时，只能做离线图片与回放验证，不能宣称相机/飞行验收通过。
