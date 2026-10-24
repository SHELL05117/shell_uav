# 环境与版本记录（接收端训练电脑）

本文件记录实际使用的环境，不含猜测值。所有路径均相对于交回包根目录或注明绝对路径。

## 1. 硬件与操作系统

| 项 | 实测值 | 获取方式 |
|---|---|---|
| 操作系统 | Microsoft Windows 11 Pro，版本 10.0.26200 | `Get-CimInstance Win32_OperatingSystem` |
| CPU | 12th Gen Intel Core i5-12400F，6 核 | `Win32_Processor` |
| 内存 | 15.8 GB | `Win32_OperatingSystem.TotalVisibleMemorySize` |
| GPU | NVIDIA GeForce RTX 3060 Ti，8192 MiB | `nvidia-smi` / `torch.cuda.get_device_properties` |
| 显卡驱动 | 616.92（CUDA UMD 13.4） | `nvidia-smi` |
| 计算能力 | sm_86 | `torch.cuda.get_device_capability` |
| 磁盘 | C: 约 82 GB 可用，D: 约 200 GB 可用（训练期间） | `Get-PSDrive` |
| cuDNN | 9.1.0（torch 内部） | `torch.backends.cudnn.version()` |

机器身份说明：本机是**接收交接包的训练电脑**，没有 RK 板、没有相机、没有无人机。

## 2. 训练环境（与转换环境分离）

| 项 | 值 |
|---|---|
| 安装方式 | 官方 Python 3.10.11 安装包，per-user、`PrependPath=0`，装在 `env\python310` |
| 安装包来源 | `https://mirrors.tuna.tsinghua.edu.cn/python/3.10.11/python-3.10.11-amd64.exe` |
| 安装包 SHA256 | `d8dede5005564b408ba50317108b765ed9c3c510342a598f9fd42681cbe0648b` |
| 虚拟环境 | `env\venv-train`（仅本工程使用，不替换系统 Python） |
| PyTorch | `2.5.1+cu121`（Windows CUDA 轮子，来源 `https://mirrors.nju.edu.cn/pytorch/whl/cu121/`） |
| TorchVision | `0.20.1+cu121` |
| CUDA 运行时 | 12.1（随 torch 轮子，驱动侧 616.92 向后兼容） |
| 包清单 | `docs/pip_freeze_train.txt` |

关键说明：本机 **pypi.org 不可达**，pip 使用清华镜像 `https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple`；Windows 上 PyPI 的 `torch` 轮子是 CPU-only（203 MB），不能用于训练，CUDA 轮子（2.33 GB）单独从 NJU 的 PyTorch 轮子镜像取得。这些都在 `docs/` 的日志里有记录。

## 3. 转换环境（第一代 RKNN，独立锁定）

| 项 | 值 |
|---|---|
| Python | 3.6.8（`env\python36` + `env\venv-rknn`） |
| RKNN-Toolkit | `rknn-toolkit 1.7.5`，官方 Windows 轮子 `rknn_toolkit-1.7.5-cp36-cp36m-win_amd64.whl` |
| 轮子 MD5 | `af96a8ab4cffa1037d41deb155e3d92e`（与官方 `packages.md5sum` 一致） |
| 轮子来源 | `rockchip-linux/rknn-toolkit` release `v1.7.5` 的 `rknn-toolkit-v1.7.5-packages.zip`（678.1 MB，经镜像下载） |
| TensorFlow | 1.14.0（toolkit 启动时导入，缺它 `RKNN()` 构造即失败） |
| Torch（转换侧） | 1.10.0+cpu（toolkit 导入时用到，不参与训练） |
| ONNX / ONNX Runtime | 1.10.0 / 1.9.0（toolkit 1.7.5 的固定依赖） |
| NumPy | 1.19.5 |
| 目标平台字符串 | `rk3399pro` |
| 包清单 | `docs/pip_freeze_rknn.txt` |

两个环境完全分开：转换环境没有安装训练用的 torch 2.5.1，训练环境没有安装 rknn-toolkit。

## 4. 训练源码与初始化权重

| 项 | 值 |
|---|---|
| 仓库 | `https://github.com/airockchip/yolov5`（RK 优化分支，基于 ultralytics/yolov5 v7.0） |
| 固定 commit | `d25a07534c14f44296f9444bab2aa5c601cdaaab` |
| 获取方式 | `https://codeload.github.com/airockchip/yolov5/zip/<commit>`（本机无 git 可执行文件，用固定 commit 的源码包） |
| 源码包 SHA256 | `4b398f847229d99bd87b3cbef63b585dabe7280fcaa2bfdf3b398d278ea3cf7e` |
| 激活函数证据 | `models/common.py` 中 `Conv.__init__` 使用 `nn.ReLU()`；构建后统计 57 个 `Conv` 层全部为 `ReLU`，无 `SiLU` |
| 初始化权重 | ultralytics/yolov5 release `v7.0` 的 `yolov5s.pt`，SHA256 `8b3b748c1e592ddd8868022e8732fde20025197328490623cc16c6f24d0782ee` |
| 初始化权重的限制 | 该权重是 **SiLU** 激活的 COCO 80 类模型；只按张量形状迁移匹配层到 ReLU 6 类网络，属于迁移学习初始化，**不是**已验证的 YOLOv5s-ReLU 预训练权重（见 `training.md` 的说明与消融对比） |

## 5. 网络与镜像（本机实测）

| 目标 | 结果 |
|---|---|
| github.com / codeload / api.github.com / raw.githubusercontent | 可达 |
| pypi.org | **不可达**（连接超时） |
| mirrors.tuna.tsinghua.edu.cn（PyPI、Python 安装包） | 可达 |
| mirrors.nju.edu.cn/pytorch（PyTorch 轮子） | 可达，约 45–60 MB/s |
| download.pytorch.org | 可达但约 1 MB/s，未使用 |
| registry-1.docker.io | 不可达，Docker 引擎未运行 |
| WSL | 有 WSL2，仅 `docker-desktop` 发行版，无 Ubuntu |

因此本轮转换在 **Windows 原生 Python 3.6 + 官方 1.7.5 Windows 轮子**里完成，没有走 Docker/WSL。
