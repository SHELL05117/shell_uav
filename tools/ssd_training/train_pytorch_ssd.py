#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Train the detector with PyTorch and export an ONNX that detectNet accepts.

WHAT THIS RUNS, AND WHY IT IS ONE SCRIPT
----------------------------------------
The model no longer goes frozen graph -> UFF -> .plan.  It goes

    data/voc_task/  --train_ssd.py-->  .pth  --onnx_export.py-->  .onnx
                                                             |
                                        check_onnx_contract.py  <- the gate
                                                             |
                                        copy 2 files to the Jetson

All three steps are somebody else's script: `train_ssd.py` and `onnx_export.py`
live in jetson-inference's own training submodule
(`python/training/detection`, which is `dusty-nv/pytorch-detection` -> the `ssd`
submodule -> `dusty-nv/pytorch-ssd`).  They are used **unmodified** on purpose:
their ONNX export names the tensors `input_0` / `scores` / `boxes` and their
`forward(is_test=True)` emits softmaxed scores plus decoded corner-form boxes,
which is exactly the contract detectNet's `postProcessSSD_ONNX` assumes.  Patching
them to fit some other exporter is how you end up with a model that loads fine
and reports nothing.

So this script does the three things that are easy to get wrong and that no
upstream script checks:

  1. **It uses the interpreter you ran it with** (`sys.executable`), so the conda
     environment that has torch is the one that trains.  Calling `python3` here
     instead would silently train in whatever that happens to be.
  2. **It checks the pretrain checkpoint exists before starting.**  The default
     is downloaded from an nvidia.box.com URL that has been dead for a while; when
     it 404s, `train_ssd.py` does not say so and training starts from random
     weights, which produces a model that "trains fine" and detects nothing.
  3. **It checks the label order that came out.**  `train_ssd.py` writes
     `BACKGROUND` + whatever `<voc root>/labels.txt` held, and that file becomes
     the class id -> name mapping on both sides.  If that root file was ever
     reordered, every class id in the model silently means a different sign, and
     the symptom is a confident detector that reports the wrong class.  The only
     cheap defence is comparing it against `labels.py` after the fact.

Usage
-----
    # 1. lay out the data (once; see prepare_pytorch_voc.py)
    python3 tools/ssd_training/prepare_pytorch_voc.py --out data/voc_task

    # 2. train + export + gate.  Run with YOUR conda python:
    python tools/ssd_training/train_pytorch_ssd.py \
        --data data/voc_task --model-dir data/models/task --epochs 100

    # see the commands without running anything
    python3 tools/ssd_training/train_pytorch_ssd.py --dry-run

Exit status
-----------
0  trained, exported, and the contract gate passed
1  something was wrong *before* training started (or the gate failed afterwards)
2  `train_ssd.py` / `onnx_export.py` itself returned non-zero
"""

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import labels as L                    # noqa: E402  (path set up above)

# Where jetson-inference's training scripts usually are.  Probed in order; the
# walker in find_train_dir() also goes a few levels down from each of these, so
# the exact submodule depth does not have to be right here.
# NOTE the `ssd` suffix on the repo-local ones: the scripts live in
# python/training/detection/**ssd**/, one level deeper than it looks.
TRAIN_DIR_CANDIDATES = [
    os.environ.get("JETSON_INFERENCE_DIR", ""),
    os.path.join(ROOT, "jetson-inference", "python", "training", "detection", "ssd"),
    os.path.join(ROOT, "jetson-inference", "python", "training", "detection"),
    os.path.join(ROOT, "..", "jetson-inference", "python", "training", "detection", "ssd"),
    os.path.join(ROOT, "..", "jetson-inference", "python", "training", "detection"),
    "/home/jcfh/jetson-inference/python/training/detection/ssd",
    "/home/jcfh/catkin_ws/src/jetson-inference/python/training/detection/ssd",
]

# jetson-inference ships this default inside its own tree; it is written into
# train_ssd.py as a relative path.
DEFAULT_PRETRAINED = "models/mobilenet-v1-ssd-mp-0_675.pth"

# 产物文件名。下游只认这一个名字：verify_exported() 查它、check_onnx_contract.py
# 拿它当门禁输入、detectNet 的 --model 默认也是它。上游 onnx_export.py 自己会
# 按 `--net` 拼出 `mb1-ssd.onnx`，所以必须显式 --output 覆盖。
ONNX_NAME = "ssd-mobilenet.onnx"

# Where the base model actually is, as of 2026-10-07.
#
# train_ssd.py's own fallback is `wget https://nvidia.box.com/shared/static/
# djf5w54rjvpqocsiztzaandq1m3avr7c.pth`, and that URL is **dead** - the NVIDIA
# forums have years of "re-download the base model", "EOFError: Ran out of input"
# and "corrupt" reports against it.  The old Google mirror
# (storage.googleapis.com/models-hao/...) is gone too: it now answers
# 403 AccessDenied "(or it may not exist)".
#
# What still works is the upstream fork's Google Drive folder, which the
# pytorch-ssd README points at.  Verified reachable: the folder listing returns
# 200 and still contains all four checkpoints.
PRETRAINED_DRIVE_FOLDER = "https://drive.google.com/drive/folders/1pKn-RifvJGWiOx0ZCRLtCXM5GT5lAluu"
PRETRAINED_DRIVE_IDS = {
    "mobilenet-v1-ssd-mp-0_675.pth": "1pSPLnWGGNs3kV_YSxr4vsmSvDCLpUsEr",
    "mb2-ssd-lite-mp-0_686.pth": None,
    "vgg16-ssd-mp-0_7726.pth": None,
}

# torch 2.6 flipped torch.onnx.export's default to the dynamo exporter, which
# needs onnxscript and emits a different graph.  2.4.1 is the version that
# produced the .onnx this project has actually run on a Jetson.
KNOWN_GOOD_TORCH = "2.4.1"


class Failure(Exception):
    """Something to report and stop on, as opposed to crash on."""


class UpstreamFailure(Failure):
    """train_ssd.py / onnx_export.py 自己返回了非零 —— 它们已经打过原因了。"""


def probe_export_deps():
    """-> (ok, problems). Checks what the ONNX **export** step needs.

    Separate from probe_torch because the failure mode is different and worse:
    `onnx` is only touched by `onnx_export.py`, at the very end of the run, so a
    missing one costs you the whole training first.  (I asserted for a while that
    torch.onnx.export did not need the `onnx` package.  It does - torch 2.4.1's
    TorchScript exporter path calls onnx_proto_utils._add_onnxscript_fn, which
    imports it.  Found by actually running the export.)
    """
    try:
        import onnx  # noqa: F401
    except ImportError:
        return False, [
            "缺 onnx 包 —— 训练能跑完，但 `onnx_export.py` 会在**最后一步**失败，"
            "那时候几个小时的 GPU 时间已经花掉了。\n"
            "  证据（2026-10-07 实测）：torch 2.4.1 导出时报\n"
            "    torch.onnx.errors.OnnxExporterError: Module onnx is not installed!\n"
            "  它是被 torch 的 TorchScript 导出路径间接 import 的"
            "（torch/onnx/utils.py 的 _export -> _add_onnxscript_fn），"
            "不是 onnx_export.py 自己要的。\n"
            "  【修法】在这个环境里 `pip install onnx`（纯 CPU 包，很小）"]
    return True, []


def probe_nvidia_smi():
    """Ask nvidia-smi for the card name, independently of torch.

    Worth the detour: "this machine has no GPU" and "the GPU is right there but
    this torch is the CPU build" call for completely different fixes, and
    `torch.cuda.is_available()` being False cannot tell them apart.
    """
    try:
        import subprocess
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, timeout=15)
    except (OSError, ImportError):
        return None
    if result.returncode != 0:
        return None
    names = [line.strip() for line in result.stdout.decode("utf-8", "replace").splitlines()
             if line.strip()]
    return names[0] if names else None


def probe_torch(smoke=False):
    """-> (version_string, problems, info) about the torch in *this* interpreter.

    Every problem carries its own 【修法】.  An earlier version of this script
    ended with one generic hint -- "add --no-pretrained if you really mean to
    train from random weights" -- which was actively misleading, because neither
    "wrong exporter default" nor "no GPU" is fixed by that flag.
    """
    problems = []
    notes = []
    info = {"notes": notes}
    try:
        import torch
    except ImportError:
        return None, ["这个解释器里没有 torch。\n"
                      "  【修法】用装了 torch 的那个 python 跑本脚本（脚本用的是 "
                      "sys.executable，不会换解释器）：\n"
                      "    conda activate <你的环境>\n"
                      "    python tools/ssd_training/train_pytorch_ssd.py --dry-run"], {}
    version = torch.__version__
    parts = version.split("+")[0].split(".")
    info["build"] = version.split("+")[1] if "+" in version else "?"
    try:
        major, minor = int(parts[0]), int(parts[1])
    except (IndexError, ValueError):
        return version, ["读不出 torch 版本号: %r\n  【修法】换 torch。" % version], info

    cuda = bool(getattr(torch, "cuda", None) and torch.cuda.is_available())
    info["cuda"] = cuda
    if cuda:
        try:
            info["gpu"] = torch.cuda.get_device_name(0)
        except Exception:  # noqa: BLE001
            info["gpu"] = "?"

    if (major, minor) >= (2, 6):
        message = (
            "torch %s 的 `torch.onnx.export` 默认走 **dynamo 导出器**（要 onnxscript，"
            "而且它常把常量表达式编成 `com.microsoft::*` 的 contrib 算子 —— "
            "TensorRT 的 nvonnxparser 不认那些，engine 会在 Jetson 上建不出来）。\n"
            "  本项目**实测在 Jetson 上跑通**的那份 .onnx 是 torch **%s** 导出的"
            "（producer 里写着 pytorch %s，opset 17）。\n"
            "  【修法】在这个环境里换回已知可用的版本（Python 3.12 有 cp312 wheel）：\n"
            "    pip install torch==%s torchvision==0.19.1 \\\n"
            "        --index-url https://download.pytorch.org/whl/cu121"
            % (version, KNOWN_GOOD_TORCH, KNOWN_GOOD_TORCH, KNOWN_GOOD_TORCH))
        if smoke:
            notes.append(message +
                         "\n  [烟测模式] 不拦你，但记住这一版的产物**不是能上飞机的"
                         ".onnx** —— 它只用来验证数据与训练链能跑通。"
                         "真正要导出时把 torch 换回来。")
        else:
            problems.append(message)
    if not cuda:
        card = probe_nvidia_smi()
        message = ("torch.cuda.is_available() 是 False ⇒ 会用 **CPU** 训练。"
                   "SSD-MobileNet 3600 张 300×300 在 CPU 上跑 100 epoch 是**数小时到数天**"
                   "（GPU 上是几十分钟），而且 batch 只能很小。")
        if card:
            message += ("\n  卡是有的：**%s**（nvidia-smi 查到了）。所以问题不是没显卡，"
                        "而是这个环境里装的是 **CPU 版 torch**（build=%s）。\n"
                        "  【修法】把它换成 CUDA 版：\n"
                        "    pip install torch==%s torchvision==0.19.1 \\\n"
                        "        --index-url https://download.pytorch.org/whl/cu121\n"
                        "  （驱动要 ≥ 525 才带得动 cu121；nvidia-smi 那行 CUDA Version "
                        "只要 ≥ 12.1 就够。）\n"
                        "  装完先确认：\n"
                        "    python -c \"import torch;print(torch.__version__, "
                        "torch.cuda.is_available(), torch.cuda.get_device_name(0))\""
                        % (card, info.get("build", "?"), KNOWN_GOOD_TORCH))
        else:
            message += ("\n  而且 nvidia-smi 也查不到卡 ⇒ 这台机器**确实没有 NVIDIA 显卡**。\n"
                        "  【修法】三条路：① 换一台有 GPU 的机器跑这个脚本；"
                        "② 在 Jetson 上训（Nano 有 GPU，但 JetPack 4.5 的 py3.6 只能装 "
                        "torch 1.8，导出的 onnx 是 opset 9）；③ 加 --allow-cpu 用 CPU "
                        "硬跑（数小时到数天，且 batch 要调小）")
        if smoke:
            notes.append(message + "\n  [烟测模式] 已加 --allow-cpu，按你说的继续。")
        else:
            problems.append(message)
    return version, problems, info


def _has_scripts(path):
    return (os.path.isfile(os.path.join(path, "train_ssd.py")) and
            os.path.isfile(os.path.join(path, "onnx_export.py")))


def find_train_dir(explicit):
    """Locate the directory that actually holds train_ssd.py + onnx_export.py.

    The obvious guess is wrong, and guessing wrong is the whole problem here.
    jetson-inference's training tree is **two** submodule levels deep:

        python/training/detection/          <- submodule: dusty-nv/pytorch-detection
            .gitmodules  ->  [submodule "ssd"] url = .../pytorch-ssd
            ssd/                             <- submodule: dusty-nv/pytorch-ssd
                train_ssd.py
                onnx_export.py

    So `python/training/detection` contains a `.gitmodules` and one empty-looking
    directory, and neither script is in it.  Rather than encode that path (which
    is one rename away from breaking again), walk down a few levels and accept
    whichever directory really has both scripts.
    """
    roots = []
    if explicit:
        roots.append(explicit)
    else:
        for candidate in TRAIN_DIR_CANDIDATES:
            if candidate:
                roots.append(candidate)

    seen_near_misses = []
    for root in roots:
        root = os.path.abspath(root)
        if not os.path.isdir(root):
            continue
        if _has_scripts(root):
            return root
        # depth-limited walk: the scripts are at most a couple of levels down,
        # and we skip noisy directories on the way
        skip = {".git", "build", "docs", "data", "models", "__pycache__"}
        for current, dirs, _files in os.walk(root):
            depth = os.path.relpath(current, root).count(os.sep)
            if depth >= 3:
                dirs[:] = []
                continue
            dirs[:] = [d for d in dirs if d not in skip]
            if _has_scripts(current):
                return current
            if os.path.isfile(os.path.join(current, ".gitmodules")):
                seen_near_misses.append(current)

    hint = ""
    if seen_near_misses:
        hint = ("\n注意：%s 下有 .gitmodules 但自己没有那两个脚本 —— "
                "submodule 还要再往下一层：\n%s\n"
                "在那一层里找。对本仓库就是 "
                "jetson-inference/python/training/detection/ssd/。"
                % (seen_near_misses[0],
                   "\n".join("    %s" % p for p in seen_near_misses)))
    elif any(os.path.isdir(r) for r in roots):
        hint = ("\n试过的目录存在，但里面没有 train_ssd.py —— "
                "clone 时多半漏了 --recursive。")

    raise Failure(
        "找不到 jetson-inference 的训练脚本（要 train_ssd.py 与 onnx_export.py）。\n"
        "  试过的位置：\n%s\n"
        "  用 --train-dir <...>/python/training/detection/ssd 指定，"
        "或设环境变量 JETSON_INFERENCE_DIR。%s\n"
        "  那个目录是 submodule：\n"
        "    git clone --recursive https://github.com/dusty-nv/jetson-inference\n"
        "  已经 clone 过但仍报这个错，就补一次（不需要重新 clone）：\n"
        "    cd <jetson-inference> && git submodule update --init --recursive"
        % ("\n".join("    %s" % r for r in roots if r), hint))


def check_dataset(path):
    """Verify the layout prepare_pytorch_voc.py produced, before spending GPU hours."""
    if not os.path.isdir(path):
        raise Failure("数据集目录不存在: %s\n先跑 prepare_pytorch_voc.py。" % path)
    annotations = os.path.join(path, "Annotations")
    images = os.path.join(path, "JPEGImages")
    trainval = os.path.join(path, "ImageSets", "Main", "trainval.txt")
    test = os.path.join(path, "ImageSets", "Main", "test.txt")
    labels = os.path.join(path, "labels.txt")

    for required in (annotations, images):
        if not os.path.isdir(required):
            raise Failure("%s 不存在。这个目录是 prepare_pytorch_voc.py 产的，"
                          "不要手工拼。" % required)
    for required in (trainval, test, labels):
        if not os.path.isfile(required):
            raise Failure("%s 不存在。VOCDataset 在这两个 ImageSets 文件都找不到时"
                          "只会抛 IOError，而缺 labels.txt 时它**不报错**，"
                          "改用 20 类 VOC 默认表 —— 那会让全部类名错位。"
                          % required)

    n_train = len([l for l in open(trainval, encoding="utf-8").read().split("\n") if l.strip()])
    n_test = len([l for l in open(test, encoding="utf-8").read().split("\n") if l.strip()])
    root_labels = [l.strip() for l in open(labels, encoding="utf-8").read().split("\n")
                   if l.strip()]

    if n_train == 0:
        raise Failure("trainval.txt 是空的 ⇒ 训练集为空。")
    if n_test == 0:
        print("警告: test.txt 是空的（验证集为空）。能训练，但训完没法在本地比较"
              "哪个 epoch 更好 —— 那就只能靠飞出去看。", file=sys.stderr)

    # labels.py 是类别体系的唯一真相源（README 坑清单第 5 条）。root labels.txt
    # 决定类 id，顺序错了模型不会报错，只是每个类号都换了含义。
    if root_labels != list(L.CLASS_NAMES):
        raise Failure(
            "%s 的内容与 labels.py 的 CLASS_NAMES 不一致。\n"
            "  这个文件的**顺序**就是类 id，VOCDataset 自己 prepend 一个 "
            "BACKGROUND，train_ssd.py 再把同样的顺序写进模型目录的 labels.txt。\n"
            "  文件里: %s\n  labels.py: %s\n"
            "  改法：用 labels.py 的顺序重新生成（prepare_pytorch_voc.py 会做），"
            "不要手工改。" % (labels, root_labels, list(L.CLASS_NAMES)))

    print("数据集 %s" % path)
    print("  trainval.txt %d 张 / test.txt %d 张" % (n_train, n_test))
    print("  labels.txt   %d 行（不含 BACKGROUND，VOCDataset 会自己加）"
          % len(root_labels))
    print("  类别顺序与 labels.py 一致 ✓")
    return n_train, n_test


def check_pretrained_file(path):
    """Load the checkpoint and confirm it is a whole SSD state_dict.

    Worth doing because the documented failure mode for this file is a **truncated
    download**: the NVIDIA forum is full of "EOFError: Ran out of input" and
    "re-download the base model" for exactly this URL.  A short file does not fail
    here, it fails 20 minutes into training inside DataLoader.

    Returns None when there is nothing to complain about, **or** when torch is not
    importable - a missing torch is already reported separately and must not turn
    into a traceback from here.
    """
    try:
        import torch
    except ImportError:
        return None
    try:
        state = torch.load(path, map_location="cpu")
    except Exception as error:  # noqa: BLE001
        return "torch.load 失败：%s（这就是论坛上那些 EOFError 的同一件事：文件下了一半）" % error
    if not isinstance(state, dict):
        return "顶层不是 dict，而是 %s" % type(state).__name__
    keys = list(state.keys())
    # a whole SSD checkpoint has both a backbone and per-head tensors; a truncated
    # or foreign file will be missing one of them
    has_backbone = any(k.startswith("base_net.") for k in keys)
    has_heads = any(k.startswith("classification_headers.") or
                    k.startswith("regression_headers.") for k in keys)
    if not has_backbone:
        return ("里面没有 base_net.* 开头的键（%d 个键，第一个是 %r）—— "
                "这不是 pytorch-ssd 的 checkpoint" % (len(keys), keys[0] if keys else None))
    if not has_heads:
        return ("里面没有 classification_headers.* / regression_headers.* —— "
                "骨干在而检测头没了，文件不完整" )
    return None


def fetch_pretrained(path):
    """Try to download the base model with gdown.  -> True/False (False = explain)."""
    name = os.path.basename(path)
    drive_id = PRETRAINED_DRIVE_IDS.get(name)
    if drive_id is None:
        return False
    import shutil
    import subprocess
    if shutil.which("gdown") is None:
        return False
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print("正在用 gdown 下载 %s ..." % name)
    result = subprocess.run(["gdown", drive_id, "-O", path])
    return result.returncode == 0 and os.path.isfile(path)


def guess_net_from_checkpoint(path):
    """Which architecture a checkpoint belongs to, judged by its filename.

    Only used to catch a mismatch early.  `init_from_pretrained_ssd()` strips the
    detection heads and loads everything else with `load_state_dict()`, so a
    checkpoint from a different backbone fails deep inside with a wall of
    "size mismatch for base_net.0.0.weight ..." - true, but it reads like a
    corrupted download rather than "you passed --net mb1-ssd with an mb2 file".
    """
    name = os.path.basename(path).lower()
    if "mb2" in name or "mobilenet-v2" in name or "mobilenetv2" in name:
        return "mb2-ssd-lite"
    if "mb1" in name or "mobilenet-v1" in name or "mobilenetv1" in name \
            or "ssd-mobilenet" in name:
        return "mb1-ssd"
    if "vgg16" in name:
        return "vgg16-ssd"
    return None


def check_net_matches_pretrained(net, pretrained):
    """-> (problems, notes).  A mismatch is fatal; a risky-but-legal net is a note."""
    if pretrained is None:
        return [], []
    implied = guess_net_from_checkpoint(pretrained)
    if implied is None:
        return [], []
    problems = []
    notes = []
    if implied != net:
        problems.append(
            "--net=%s 与权重 %s 不是同一个架构（文件名看是 %s）。\n"
            "  init_from_pretrained_ssd() 会剥掉检测头、但骨干是按 load_state_dict "
            "整个灌进去的，形状对不上会在几十层下面炸成一片 size mismatch —— "
            "看起来像文件坏了，其实只是选错了。\n"
            "  两条出路：把 --net 改成 %s，或者 --pretrained-ssd 换成 %s 那份。"
            % (net, os.path.basename(pretrained), implied, implied,
               "mb2-ssd-lite-mp-0_686.pth" if implied == "mb2-ssd-lite"
               else "mobilenet-v1-ssd-mp-0_675.pth"))
    if net == "mb2-ssd-lite":
        notes.append(
            "mb2-ssd-lite 的 onnx 导出**带着 ReLU6**：onnx_export.py 里是 "
            "create_mobilenetv2_ssd_lite(len(class_names), is_test=True)，"
            "而 onnx_compatible 的默认值是 False ⇒ 图里是 ReLU6 而不是 ReLU。\n"
            "  上游 README 当年因此说它 'not ONNX-compatible'，那句话在这个 fork 里"
            "仍然成立。\n"
            "  较新 torch 会把 ReLU6 展成 Clip(0,6)（TensorRT 能解析），所以**可能**"
            "没事 —— 但本机没有 torch、验不了，而 mb1-ssd 那条是你实机已经跑通的"
            "（零风险）。所以除非你想追 mb2 的速度，否则用 mb1-ssd。\n"
            "  真要试：照训，导完跑 check_onnx_contract.py —— 它会把 Relu6 / Clip "
            "这类算子是否 TRT 7.1 能解析直接告诉你。")
    return problems, notes


def resolve_pretrained(train_dir, explicit):
    """-> (path_or_None, problems).  None means 'train from random weights'."""
    path = explicit if explicit else os.path.join(train_dir, DEFAULT_PRETRAINED)
    path = os.path.normpath(path)          # DEFAULT_PRETRAINED 里是 '/'
    if os.path.isfile(path):
        return path, []
    problems = []
    if explicit:
        problems.append("指定的 --pretrained-ssd 不存在: %s" % path)
    else:
        problems.append(
            "预训练权重不在 %s。\n"
            "  train_ssd.py 会去 wget 一个 nvidia.box.com 的地址，那个链接**已经失效**\n"
            "  （NVIDIA 论坛上大量 \"EOFError: Ran out of input\" / \"re-download the base\n"
            "  model\" 就是它；旧的 storage.googleapis.com/models-hao 镜像现在返回\n"
            "  403 AccessDenied）。而 wget 失败它**不报错**，于是训练从随机权重开始\n"
            "  —— 模型会正常下降，然后什么都检不出来。这是最容易浪费一整夜的一种失败。\n"
            "  还能用的出处（上游 fork 的 Google Drive，2026-10-07 实测 HTTP 200，\n"
            "  四个权重文件都在）：\n"
            "    %s\n"
            "  拿它（两条任选）：\n"
            "    pip install gdown\n"
            "    gdown 1pSPLnWGGNs3kV_YSxr4vsmSvDCLpUsEr -O \"%s\"\n"
            "  或者直接浏览器打开那个文件夹下载，再放到上面那个路径。\n"
            "  也可以让本脚本替你下：加 --fetch-pretrained。"
            % (path, PRETRAINED_DRIVE_FOLDER, path))
    return None, problems


def run(step, argv):
    print("\n" + "=" * 72)
    print(">> %s" % step)
    print("   %s" % " ".join(argv))
    print("=" * 72)
    sys.stdout.flush()
    result = subprocess.run(argv)
    if result.returncode != 0:
        raise UpstreamFailure("%s 返回非零（%d）。上面的输出是它的原话。"
                              % (step, result.returncode))


def verify_exported(model_dir, expect_classes):
    """Run our own gate on what the export produced.  This is the verdict."""
    onnx = os.path.join(model_dir, ONNX_NAME)
    labels = os.path.join(model_dir, "labels.txt")
    if not os.path.isfile(onnx):
        raise Failure("onnx_export.py 说它成功了，但 %s 不存在。" % onnx)
    if not os.path.isfile(labels):
        raise Failure("%s 不存在。train_ssd.py 会把 labels.txt 写进 --checkpoint-"
                      "folder，缺了它 detectNet 没有类名可用。" % labels)

    produced = [l.strip() for l in open(labels, encoding="utf-8").read().split("\n")
                if l.strip()]
    expected = ["BACKGROUND"] + list(L.CLASS_NAMES)
    if produced != expected:
        print("\n" + "!" * 72, file=sys.stderr)
        print("警告: %s 与期望不一致。" % labels, file=sys.stderr)
        print("  期望: %s" % expected, file=sys.stderr)
        print("  实际: %s" % produced, file=sys.stderr)
        print("类 id 就是行号，所以这两串顺序不同意味着模型里的每个类号都指向"
              "另一个标志 —— 而它不会报错。", file=sys.stderr)
        print("!" * 72 + "\n", file=sys.stderr)

    gate = os.path.join(HERE, "check_onnx_contract.py")
    print("\n" + "=" * 72)
    print(">> 门禁：check_onnx_contract.py")
    print("   这一步才是判据。onnx_export.py 返回 0 只说明它写出了文件，")
    print("   不说明 detectNet 读得懂它。")
    print("=" * 72)
    result = subprocess.run([sys.executable, gate, "--onnx", onnx,
                             "--labels", labels, "--confidence", "0.5"])
    if result.returncode != 0:
        raise Failure("契约门禁没过（退出码 %d）。**不要把这个 .onnx 拷到飞机上** ——"
                      "那上面三条静默失效里的任意一条都会让它跑起来却什么都不报。"
                      "门禁的输出在上面。" % result.returncode)

    print("\n" + "=" * 72)
    print("下一步：把**两个**文件拷到 Jetson")
    print("=" * 72)
    print("  %s" % onnx)
    print("  %s" % labels)
    print("")
    print("运行时读的是**包内**的 models/ 目录：把这两个文件覆盖到")
    print("  <ws>/src/ssd_detection/models/ssd-mobilenet.onnx")
    print("  <ws>/src/ssd_detection/models/labels.txt")
    print("（launch 默认 $(find ssd_detection)/models/...，仿真与实机同一路径）")
    print("")
    print("之后（以 ~/catkin_ws/src/ssd_detection/models/ 为例）：")
    print("  # 1) 先离线看一眼框（不启动 ROS）。第一次建 engine，Nano 上几分钟。")
    print("  rosrun ssd_detection ssd_detection_node \\")
    print("      --model ~/catkin_ws/src/ssd_detection/models/ssd-mobilenet.onnx \\")
    print("      --labels ~/catkin_ws/src/ssd_detection/models/labels.txt \\")
    print("      --dir <几张图> --out /tmp/ssd_preview")
    print("")
    print("  # 2) 装进任务链（与仿真侧同一个模型文件）")
    print("  # 真机：use_usb_cam:=true 同时把图像源（GStreamer 直采）和检测实现")
    print("  #      （C++ detectNet）切过去；仿真只用 vision_backend:=ssd")
    print("  roslaunch navigation_2d navigation_2d.launch use_usb_cam:=true \\")
    print("      vision_backend:=ssd")
    return onnx, labels


def main():
    parser = argparse.ArgumentParser(
        description="train_ssd.py -> onnx_export.py -> check_onnx_contract.py")
    parser.add_argument("--data", default="data/voc_task",
                        help="prepare_pytorch_voc.py 产出的目录（默认 data/voc_task）")
    parser.add_argument("--model-dir", default="data/models/task",
                        help="checkpoint 与 onnx 的输出目录（默认 data/models/task）")
    parser.add_argument("--train-dir", default="",
                        help="jetson-inference 的 python/training/detection；"
                             "默认按 $JETSON_INFERENCE_DIR 与几个常见位置找")
    parser.add_argument("--net", default=None,
                        help="网络。不给就按 --pretrained-ssd 的文件名推断，"
                             "再不行默认 mb1-ssd（= SSD-Mobilenet，与 detectNet "
                             "的 onnx_export.py 默认一致，也是本项目实机跑通过的那个）")
    parser.add_argument("--resolution", type=int, default=300)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=16,
                        help="Nano 上训不动；这张卡是给 PC 用的。显存不够就减半")
    parser.add_argument("--lr", type=float, default=0.01,
                        help="SGD 初始学习率（pytorch-ssd 的默认值）")
    parser.add_argument("--workers", type=int, default=None,
                        help="DataLoader 进程数；不给则 Windows 上 0、其它 2。"
                             "Windows 上 >0 会反复启动 python 进程")
    parser.add_argument("--pretrained-ssd", default="",
                        help="预训练权重 .pth；不给则用 jetson-inference 自带的")
    parser.add_argument("--allow-cpu", action="store_true",
                        help="烟测模式：用 CPU 训练，并且不拦 torch 版本的"
                             "导出器问题。**产物不是能上飞机的 .onnx**，只用来看"
                             "数据与训练链能不能跑通（建议再配 --epochs 2）")
    parser.add_argument("--no-pretrained", action="store_true",
                        help="明确从随机权重开始（会先警告）")
    parser.add_argument("--fetch-pretrained", action="store_true",
                        help="缺权重时用 gdown 自动拉（需要 pip install gdown）。"
                             "拉完会 torch.load 一次并确认它是个完整的 SSD "
                             "checkpoint —— 下半截文件是这个权重最常见的故障，"
                             "而它要 20 分钟后才在 DataLoader 里炸")
    parser.add_argument("--freeze-base-net", action="store_true",
                        help="冻结骨干，只训头与 extras。样本少时有用")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印将要执行的命令")
    args = parser.parse_args()

    if args.workers is None:
        args.workers = 0 if os.name == "nt" else 2

    # ---- 前置检查：全部在花 GPU 时间之前 --------------------------------
    problems = []
    torch_version, torch_problems, torch_info = probe_torch(args.allow_cpu)
    problems.extend(torch_problems)
    # 导出依赖也要在开工前查：它只在最后一步被 import，缺了等于白训。
    _export_ok, export_problems = probe_export_deps()
    problems.extend(export_problems)

    try:
        train_dir = find_train_dir(args.train_dir)
        n_train, n_test = check_dataset(args.data)
    except Failure as error:
        print("前置检查失败:\n  %s" % error, file=sys.stderr)
        return 1

    pretrained, pretrained_problems = resolve_pretrained(train_dir,
                                                         args.pretrained_ssd)
    if pretrained is None and args.fetch_pretrained and not args.no_pretrained:
        print("\n--fetch-pretrained：试着把基础权重拉下来 ...")
        if fetch_pretrained(os.path.join(train_dir, DEFAULT_PRETRAINED)):
            pretrained, pretrained_problems = resolve_pretrained(train_dir, "")
            print("下载完成。")
        else:
            print("没能自动下载（gdown 没装？），按上面的提示手动来。",
                  file=sys.stderr)
    if pretrained is not None:
        broken = check_pretrained_file(pretrained)
        if broken:
            problems.append("预训练权重有问题：%s" % broken)
    if args.no_pretrained:
        print("\n按你的要求从随机权重开始。样本量小的话这基本学不出可用的检测器。",
              file=sys.stderr)
        pretrained = None
        pretrained_problems = []
    else:
        problems.extend(pretrained_problems)

    # --net 与权重必须同一个架构。默认让权重说话（它的文件名比我们的默认可信），
    # 但显式给了又冲突就直接拦住，别让 load_state_dict 在几十层下面炸。
    net = args.net or (guess_net_from_checkpoint(pretrained) if pretrained else None) \
        or "mb1-ssd"
    if args.net is None and guess_net_from_checkpoint(pretrained):
        print("按权重文件名自动选定 --net %s"
              % guess_net_from_checkpoint(pretrained))
    net_problems, net_notes = check_net_matches_pretrained(net, pretrained)
    problems.extend(net_problems)

    for note in list(torch_info.get("notes", [])) + list(net_notes):
        print("\n注意：%s" % note, file=sys.stderr)


    model_dir = os.path.abspath(args.model_dir)
    python = sys.executable

    train_cmd = [python, os.path.join(train_dir, "train_ssd.py"),
                 "--dataset-type", "voc",
                 # 上游的参数名是 `--datasets`（带 s，nargs='+'）。写 `--dataset`
                 # 会撞上 argparse 的前缀缩写规则：--dataset 同时是 --dataset-type
                 # 和 --datasets 的前缀 ⇒ "ambiguous option"，训练一秒都跑不了。
                 "--datasets", os.path.abspath(args.data),
                 "--net", net,
                 "--resolution", str(args.resolution),
                 "--batch-size", str(args.batch_size),
                 "--num-epochs", str(args.epochs),
                 "--lr", str(args.lr),
                 "--workers", str(args.workers),
                 "--checkpoint-folder", model_dir]
    if pretrained:
        train_cmd += ["--pretrained-ssd", pretrained]
    if args.freeze_base_net:
        train_cmd.append("--freeze-base-net")

    export_cmd = [python, os.path.join(train_dir, "onnx_export.py"),
                  "--net", net,
                  "--model-dir", model_dir,
                  "--resolution", str(args.resolution),
                  # onnx_export.py 的输出名默认是 `args.net + '.onnx'`，
                  # 也就是 mb1-ssd.onnx；而 verify_exported() 和 detectNet
                  # 都只认 ssd-mobilenet.onnx（mb1-ssd 是 ssd-mobilenet 的别名）。
                  # 不显式给 --output 的话，训练跑完会在第 2/3 步找不到文件。
                  "--output", ONNX_NAME]

    print("python          : %s" % python)
    print("torch           : %s  (build=%s, cuda=%s%s)"
          % (torch_version, torch_info.get("build", "?"),
             torch_info.get("cuda"),
             (", " + torch_info["gpu"]) if torch_info.get("gpu") else ""))
    print("train scripts   : %s" % train_dir)
    print("dataset         : %s (%d train / %d val)" % (args.data, n_train, n_test))
    print("model-dir       : %s" % model_dir)
    print("net             : %s%s" % (net, "  (auto)" if not args.net else ""))
    print("classes         : %d (+1 背景 = scores 的最后一维应为 %d)"
          % (len(L.CLASS_NAMES), len(L.CLASS_NAMES) + 1))
    print("pretrained      : %s" % (pretrained or "(无 —— 随机权重)"))

    if problems:
        print("\n" + "!" * 72, file=sys.stderr)
        print("开工前有几件事要你自己决定（每条都带自己的修法）：", file=sys.stderr)
        for item in problems:
            print("  * %s" % item, file=sys.stderr)
        print("!" * 72, file=sys.stderr)
        print("\n每一条的【修法】就是对应那件事的解法 —— 没有一个全局开关能同时关掉"
              "它们。逐条处理完再跑。\n"
              "  * 缺权重      → 放好文件，或 --fetch-pretrained，或真的"
              "从随机权重起步就加 --no-pretrained\n"
              "  * torch 版本  → 换回已知可用的版本（命令在上面的【修法】里）\n"
              "  * 没有 GPU    → 装 CUDA 版 torch，或加 --allow-cpu（会慢到数小时到数天）\n"
              "  * 缺 onnx     → pip install onnx（导出那一步才 import，缺了等于白训）\n",
              file=sys.stderr)
        return 1

    if args.dry_run:
        print("\n--dry-run，将要执行：")
        print("  " + " \\\n    ".join(train_cmd))
        print("  " + " \\\n    ".join(export_cmd))
        print("  %s --onnx %s --labels %s"
              % (os.path.join(HERE, "check_onnx_contract.py"),
                 os.path.join(model_dir, ONNX_NAME),
                 os.path.join(model_dir, "labels.txt")))
        return 0

    try:
        os.makedirs(model_dir, exist_ok=True)
        run("1/3 训练（这一步会很久）", train_cmd)
        run("2/3 导出 ONNX", export_cmd)
        verify_exported(model_dir, len(L.CLASS_NAMES) + 1)
    except UpstreamFailure as error:
        print("\n上游脚本失败: %s" % error, file=sys.stderr)
        return 2
    except Failure as error:
        print("\n失败: %s" % error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
