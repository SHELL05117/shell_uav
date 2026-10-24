# RK 离线部署清单（TB-RK3399ProD / 第一代 RKNN）

本文件只列**电脑侧已准备**与**目标机待验证**两类事实。没有 RK 实物，所有硬件项一律标“未验证”，不写猜测结论。

## 1. 目标机（来自交接包，未在本机核实）

| 项 | 交接包记录 | 本机状态 |
|---|---|---|
| 板卡 | Toybrick TB-RK3399ProD | 未接硬件 |
| OS / 内核 | Ubuntu 18.04 ARM64 / 厂商内核 4.4.194 | 未验证 |
| ROS | ROS1 Melodic | 未验证 |
| 内存 | CPU 侧约 4 GB | 未验证 |
| NPU 软件栈 | 交接包只读盘点未找到 `npu_transfer_proxy`、rknn 运行库、相关 systemd 单元或进程 | **未取得可用 NPU 栈证据**（不等于硬件损坏） |
| 模型加载 / 延迟 / 温度 | — | 未测 |

## 2. 电脑侧已准备好的运行材料

| 材料 | 位置 | 说明 |
|---|---|---|
| 量化模型 | `models/sixclass_yolov5s_relu_rk3399pro.rknn`（7.17 MB，SHA256 `f922b48b…`） | rknn-toolkit 1.7.5，`target_platform="rk3399pro"`，8bit 非对称 u8 |
| 浮点对照模型 | `models/best.onnx`、`models/best.torchscript` | 用于板端不一致时定位问题 |
| anchors | `config/RK_anchors.txt` | 9 个像素 anchor，按 stride 展开 |
| 类别表 | `config/class_map.json` | 六个 ID 与业务语义，顺序不可改 |
| 解码/后处理约定 | `docs/integration_notes.md` | 含网格、anchor、letterbox 还原、按类 NMS |
| 参考实现 | `scripts/06_infer_onnx.py` | PC 端浮点推理，可与板端逐框比对 |
| 量化对照脚本 | `scripts/08_rknn_compare.py` | 板端或可模拟环境下跑浮点 vs 量化 |

## 3. 板端仍需获取的组件（电脑下载→SSH 传入，不让板联网）

| 组件 | 用途 | 匹配要求 | 状态 |
|---|---|---|---|
| `librknn_runtime.so` | NPU 推理运行时 | 必须与 1.7.5 模型及板端 NPU 驱动版本匹配；**不能**用 RKNN-Toolkit2 / RK3588 的库 | 待目标机确认现有版本 |
| `librknn_api.so` + `rknn_api.h` | C API 头文件与库 | 同上 | 待确认 |
| `npu_transfer_proxy` | RK3399Pro 的 NPU 通信代理（交接包盘点未发现） | 需与固件/驱动配套 | 待确认 |
| NPU 固件/驱动 | 板级配套 | **先核对现有 BSP**，不盲目刷写 | 未验证 |
| `rknn_yolov5_demo` 源码 | 官方 C++ 后处理示例 | RK3399Pro 专属仓库固定 commit | 电脑侧已有固定 URL，未编译到板端 |

每个文件传入前都应记录来源 URL、版本、架构、SHA256；传入后核对再运行。

## 4. 板端集成改动点（必须做，官方 demo 是 80 类）

1. 类别数：`OBJ_CLASS_NUM` 80 → 6；输出通道 `3×(5+80)` → `3×(5+6)=33`。
2. anchors：换成 `RK_anchors.txt` 中的 9 组值（`rknn_yolov5_demo` 用 `BOX_THRESH`/anchor 数组，注意单位是像素）。
3. 输出顺序：按空间尺寸 80/40/20 取，不能按名字（ONNX 里叫 `output0/286/288`）。
4. 置信度：模型输出**已含 sigmoid**，不要再 sigmoid 一次。
5. NMS：按类别做，阈值与 PC 侧报告一致（0.25 / 0.45 仅作对照工作点）。
6. letterbox 还原：按 `docs/integration_notes.md` 的 `r/dx/dy` 公式，别把 640 坐标当原图坐标。
7. 业务白名单：只允许 ID 0–3 触发任务；ID 4/5 必须显式忽略。
8. 输入张量类型：用 `rknn_query(RKNN_QUERY_INPUT_ATTR)` 确认是 `UINT8`（NHWC）还是 `FLOAT32`；本模型配置了 `channel_mean_value="0 0 0 255"`，预期喂 **uint8 RGB NHWC**。

## 5. 板端验收清单（全部未做）

- [ ] 运行库/代理/固件版本核对，模型可加载
- [ ] 单帧推理成功，输出三张量形状为 80/40/20 × 33
- [ ] 板端框与 PC 端浮点框在同一图片上逐框一致（IoU、类别）
- [ ] 量化模型的逐类 P/R、干扰类误触发率（业务关键项）
- [ ] 端到端延迟、结果年龄、帧率、内存、温度
- [ ] 连续运行稳定性、异常（相机断流、推理失败）处理
- [ ] 与 SLAM/规划并行时的资源与温度

以上任何一项未做，都不能称为“部署完成”。
