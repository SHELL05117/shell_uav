# 导出与量化记录（ONNX / TorchScript / RKNN）

## 1. 导出命令（在训练环境中）

```
cd work\repos\yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab
python export.py --weights <best.pt> --include onnx torchscript --rknpu --img 640 --batch 1 --device cpu --opset 12
```

`--rknpu` 的作用（本仓库 `export.py` 实现）：

- 生成 `RK_anchors.txt`（按 stride 展开后的像素 anchor）；
- 把 Detect 头切成 3 个独立输出，**保留 sigmoid、去掉后处理**（RK demo 的 CPU 后处理需要这种格式）；
- 对首层 6×6/stride2 卷积做等价替换（2×2 stride2 + 3×3 stride1，权重按相位切片），实测输出一致。

## 2. 产物与哈希（第一轮模型 exp1）

| 文件 | 大小 | SHA256 |
|---|---:|---|
| `best.pt`（PyTorch checkpoint，含 EMA） | 13.76 MB | `007d6088f936fbf12d6c39b6cea15c15a04c9bf1f84ccb3455198f70369e4d6e` |
| `best.onnx`（opset 12） | 26.83 MB | `42f4c1fba3b5aa9716961a5ad87dccb8c6bb67c376c6a03003e3c6b82b297933` |
| `best.torchscript` | 27.10 MB | `af8ad5ae8041bf9a825e5e1af98751ba3949e10390974dc1adde5e614d4d32cb` |
| `sixclass_yolov5s_relu_rk3399pro.rknn` | 7.17 MB | `f922b48bdd34e0ed47b94a8c3975a6b12b58989c8cf4c5816f66cc3a00b6eab1` |

ONNX 的输入输出：

```
IN   images  [1, 3, 640, 640]
OUT  output0 [1, 33, 80, 80]
OUT  286     [1, 33, 40, 40]
OUT  288     [1, 33, 20, 20]
opset 12
```

## 3. 浮点一致性验证（已做）

脚本 `scripts/06_infer_onnx.py --parity-ckpt <best.pt>`，同一批 640×640 letterbox 输入：

| 比较项 | 结果 |
|---|---|
| ONNX 三头 vs PyTorch `Conv + sigmoid` | 最大绝对差 `3.0e-4` |
| 同一解码函数处理两侧 head 的最高分候选 | 框坐标最大差 `0.0004 px`，置信度差 `3e-6` |
| 两侧各自 NMS 后匹配 | IoU 均值 `0.73`、最小 `0.48`（差异源于 NMS 实现取舍，见 `integration_notes.md`） |

说明：并行跑 CUDA 时曾观察到 `3.6e-2` 的 head 差异，原因是 Ampere 默认 TF32；改为 CPU fp32 后一致到 `3e-4`。对拍一律用 CPU。

## 4. RKNN 转换（第一代工具链，独立环境）

环境：`env\venv-rknn`（Python 3.6.8 + rknn-toolkit 1.7.5 Windows wheel），与训练环境完全分开。

```
python scripts\07_rknn_convert.py ^
  --onnx best.onnx ^
  --calib work\dataset\calib\dataset.txt ^
  --out work\rknn --name sixclass_yolov5s_relu_rk3399pro
```

关键配置（`rknn-toolkit 1.7.x` API）：

```python
rknn.config(channel_mean_value="0 0 0 255",   # 输入 /255，均值 0
            reorder_channel="0 1 2",          # RGB 不换通道
            target_platform="rk3399pro",      # 第一代工具链的目标平台
            quantized_dtype="asymmetric_quantized-u8",
            quantized_algorithm="normal",
            optimization_level=3, batch_size=1)
rknn.load_onnx(model="best.onnx")             # 输入已是固定 1x3x640x640，不做 crop
rknn.build(do_quantization=True, dataset="calib/dataset.txt")
rknn.export_rknn("sixclass_yolov5s_relu_rk3399pro.rknn")
```

- 校准集：48 张（每类 8 张），只能用训练集生成，测试/验证集不参与。
- 结果：`build` 与 `export_rknn` 均返回 0；toolkit 自带的浮点/量化一致性检查（在量化数据集上）三个输出都是 `cosine = 1.0, euclidean = 0.0`。
- 过程中出现过 `found outlier value, this may affect quantization accuracy` 警告；属于提示信息，未阻止转换。

### 遇到并解决的两个问题

1. 初次 `load_onnx` 传入 `inputs/input_size_list/outputs` 做裁剪，配合 `mean_values` 导致
   `The len of mean_values ... is wrong, expect 1` 且 `Calc node Conv ... output shape fail`。
   → 输入尺寸本来就固定，改为不做裁剪，并使用 1.7.x 的 `channel_mean_value`，转换成功。
2. `RKNN()` 构造时先后缺 `tensorflow`、`torch`（toolkit 内部模块 import）。
   → 在转换环境补 `tensorflow==1.14.0` 与 `torch==1.10.0+cpu`，不动训练环境。

## 5. 量化精度对照：本机无法完成，需目标板

`rknn.init_runtime(target=None)` 返回：

```
E Simulator is not supported after version 1.7.3. Please use development board for RKNN model evaluation.
```

即 rknn-toolkit 1.7.5 起**取消了 PC 端模拟器**，量化的逐类精度只能：
- 在 RK3399Pro 实板上用 `rknn_yolov5_demo`（或自写 C++ 后处理）跑测试集；或
- 在 Linux x86 环境安装更早的 toolkit（≤1.7.3）做模拟——本轮**没有**这样做，因为项目要求锁定 1.7.5 组合。

已交付 `scripts/08_rknn_compare.py`：在能连板/能模拟的环境里直接跑 `--onnx ... --rknn ...`，输出
`work/rknn/quant_vs_float.json`（逐图 top-1 类别一致性、IoU、置信度差、各类检出数）。脚本本身已写好并在本机执行到
`init_runtime` 一步才停止，说明其余流程（读模型、解码、NMS、比对）可用。

## 6. 目标板侧仍需完成

- 用 `rknn_yolov5_demo`（RK3399Pro 专属示例）替换为六类模型：改 class 数（6）、anchors、后处理里的类别循环、输出节点名/顺序；
- 板端实测单帧延迟、内存、温度与连续运行稳定性；
- 用真实/回放图像核对板端框与 PC 端浮点框一致（本文件的解码约定是基准）；
- 量化模型的逐类精度、干扰类误触发率必须在板上复测，不能把 PC 上的浮点指标当作量化指标。
