"""Stage 7: convert the RK-style ONNX to a first-generation .rknn (RK3399Pro).

Environment: env/venv-rknn (Python 3.6.8 + rknn-toolkit 1.7.5 Windows wheel,
separate from the training environment on purpose).

The ONNX must come from `export.py --rknpu` (three sigmoid-activated heads, no
post-processing).  Quantisation is 8-bit asymmetric (u8) as the RK3399Pro demo
expects; calibration images are letterboxed 640x640 crops that never include the
frozen test split (see 03_make_calib.py).

Usage (inside env\\venv-rknn):
  python 07_rknn_convert.py --onnx best.onnx --calib work/dataset/calib/dataset.txt --out work/rknn
"""
from __future__ import print_function

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True, type=Path)
    ap.add_argument("--calib", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "work" / "rknn")
    ap.add_argument("--name", default="sixclass_yolov5s_relu_rk3399pro")
    ap.add_argument("--no-quant", action="store_true")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    from rknn.api import RKNN  # imported here so the script fails clearly outside the converter env

    log = {"onnx": str(args.onnx), "onnx_sha256": None, "toolkit": None, "steps": [], "ok": False, "error": None}
    import hashlib

    log["onnx_sha256"] = hashlib.sha256(args.onnx.read_bytes()).hexdigest()

    rknn = RKNN(verbose=False)
    try:
        import rknn as rknn_pkg
        log["toolkit"] = getattr(rknn_pkg, "__version__", "unknown")
    except Exception:
        pass

    t0 = time.time()
    ret = rknn.config(
        channel_mean_value="0 0 0 255",   # 1.7.x API: mean r g b and scale (model input is x/255)
        reorder_channel="0 1 2",          # input is RGB; no channel swap inside the NPU
        target_platform="rk3399pro",      # first-generation toolkit, TB-RK3399ProD
        quantized_dtype="asymmetric_quantized-u8",
        quantized_algorithm="normal",
        optimization_level=3,
        batch_size=1,
    )
    log["steps"].append({"step": "config", "ret": ret})
    if ret != 0:
        raise RuntimeError("rknn.config failed: %s" % ret)

    import onnx

    m = onnx.load(str(args.onnx))
    in_names = [i.name for i in m.graph.input]
    out_names = [o.name for o in m.graph.output]
    log["inputs"] = in_names
    log["outputs"] = out_names
    insz = [[d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim]]
    # no cropping: the ONNX input is already fixed at 1x3x640x640
    ret = rknn.load_onnx(model=str(args.onnx))
    log["steps"].append({"step": "load_onnx", "ret": ret, "input_size": insz})
    if ret != 0:
        raise RuntimeError(f"rknn.load_onnx failed: {ret}")

    ret = rknn.build(do_quantization=not args.no_quant, dataset=str(args.calib))
    log["steps"].append({"step": "build", "ret": ret, "quantized": not args.no_quant})
    if ret != 0:
        raise RuntimeError(f"rknn.build failed: {ret}")

    rknn_path = args.out / f"{args.name}.rknn"
    ret = rknn.export_rknn(str(rknn_path))
    log["steps"].append({"step": "export_rknn", "ret": ret, "path": str(rknn_path)})
    if ret != 0:
        raise RuntimeError(f"rknn.export_rknn failed: {ret}")

    log["ok"] = True
    log["elapsed_s"] = round(time.time() - t0, 1)
    log["rknn_bytes"] = rknn_path.stat().st_size
    log["rknn_sha256"] = hashlib.sha256(rknn_path.read_bytes()).hexdigest()
    (args.out / "convert_report.json").write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(log, ensure_ascii=False, indent=1))
    rknn.release()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # keep a machine-readable failure record
        import json as _json
        import traceback

        out = ROOT / "work" / "rknn"
        out.mkdir(parents=True, exist_ok=True)
        (out / "convert_error.json").write_text(
            _json.dumps({"ok": False, "error": repr(exc), "traceback": traceback.format_exc()}, ensure_ascii=False, indent=1),
            encoding="utf-8",
        )
        print("CONVERT FAILED:", repr(exc), file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
