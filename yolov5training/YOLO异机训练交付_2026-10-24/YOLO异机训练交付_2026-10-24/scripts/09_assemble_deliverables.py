"""Stage 9: assemble the return package (deliverables/) and its SHA256 manifest.

Copies only the artefacts that belong to SHELL: configs, scripts, metrics, models,
selected reports/logs.  The 866 MB source archive, the derived images/labels and the
virtual environments are NOT copied; the manifest records their hashes elsewhere.

Usage: python 09_assemble_deliverables.py [--best-exp exp3_rot_finetune]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PKG = next(p for p in ROOT.iterdir() if p.is_dir() and p.name.startswith("YOLO")
           and (p / "config" / "class_map.json").exists())
WORK = ROOT / "work"
DELIV = ROOT / "deliverables"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()
    shutil.copy2(src, dst)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--best-exp", default="exp3_rot_finetune")
    ap.add_argument("--run2-exp", default="exp1_warm_coco_silu")
    args = ap.parse_args()

    best_run = WORK / "runs" / args.best_exp
    run2 = WORK / "runs" / args.run2_exp
    if not (best_run / "weights" / "best.pt").exists():
        raise SystemExit(f"missing checkpoint: {best_run}")

    # ---- config ---------------------------------------------------------
    copy(PKG / "config" / "class_map.json", DELIV / "config" / "class_map.json")
    copy(WORK / "dataset" / "sixclass" / "data.yaml", DELIV / "config" / "data.yaml")
    copy(WORK / "hyp" / "hyp_rot_finetune.yaml", DELIV / "config" / "hyp_rot_finetune.yaml")
    copy(PKG / "config" / "upstream_sources.json", DELIV / "config" / "upstream_sources.json")
    copy(WORK / "repos" / "yolov5-d25a07534c14f44296f9444bab2aa5c601cdaaab" / "RK_anchors.txt",
         DELIV / "config" / "RK_anchors.txt")

    # ---- scripts --------------------------------------------------------
    for py in sorted((WORK / "scripts").glob("*.py")):
        copy(py, DELIV / "scripts" / py.name)
    for bat in sorted((WORK / "scripts").glob("*.bat")):
        copy(bat, DELIV / "scripts" / bat.name)

    # ---- models ---------------------------------------------------------
    for name in ("best.pt", "best.onnx", "best.torchscript"):
        src = best_run / "weights" / name
        if src.exists():
            copy(src, DELIV / "models" / name)
    rknn = WORK / "rknn" / "sixclass_yolov5s_relu_rk3399pro.rknn"
    if rknn.exists():
        copy(rknn, DELIV / "models" / rknn.name)
    if (WORK / "rknn" / "convert_report.json").exists():
        copy(WORK / "rknn" / "convert_report.json", DELIV / "reports" / "rknn_convert_report.json")

    # ---- metrics --------------------------------------------------------
    for run, tag in ((best_run, "selected"), (run2, "ablation")):
        if not run.exists():
            continue
        out = DELIV / "metrics" / ("%s_%s" % (tag, run.name))
        for name in ("results.csv", "results.png", "confusion_matrix.png", "confusion_matrix.csv",
                     "PR_curve.png", "P_curve.png", "R_curve.png", "F1_curve.png",
                     "labels.jpg", "labels_correlogram.jpg", "opt.yaml", "hyp.yaml",
                     "train_batch0.jpg", "val_batch0_pred.jpg", "val_batch0_labels.jpg"):
            src = run / name
            if src.exists():
                copy(src, out / name)

    eval_dir = WORK / "reports" / "eval_exp1_test"
    for name in ("metrics_per_class.json", "confusion_matrix.csv", "per_image_predictions.json",
                 "task_failures.json", "detections.csv", "test_predictions_sheet.jpg"):
        src = eval_dir / name
        if src.exists():
            copy(src, DELIV / "metrics" / "test_eval" / name)
    # YOLOv5 val.py run on the frozen test split (standard per-class P/R/mAP + curve plots)
    valpy = WORK / "runs" / "eval_exp1_test"
    if valpy.exists():
        for f in sorted(valpy.iterdir()):
            if f.is_file() and f.suffix.lower() in (".png", ".csv", ".json", ".jpg"):
                copy(f, DELIV / "metrics" / "test_valpy" / f.name)
    # validation-split task analysis (used to choose the reported working point)
    for name in ("metrics_per_class.json", "confusion_matrix.csv", "per_image_predictions.json",
                 "task_failures.json", "detections.csv"):
        src = WORK / "reports" / "eval_exp1_val" / name
        if src.exists():
            copy(src, DELIV / "metrics" / "val_eval" / name)

    # ---- reports --------------------------------------------------------
    copy(WORK / "reports" / "audit_pre_conversion.json", DELIV / "reports" / "audit_pre_conversion.json")
    copy(WORK / "dataset" / "sixclass" / "split_manifest.json", DELIV / "reports" / "split_manifest.json")
    copy(WORK / "dataset" / "calib" / "calib_manifest.json", DELIV / "reports" / "calib_manifest.json")
    for name in ("summary.json",):
        src = WORK / "reports" / "onnx_infer_exp1snap" / name
        if src.exists():
            copy(src, DELIV / "reports" / "onnx_parity_summary.json")
    ov = WORK / "reports" / "overlays"
    if ov.exists():
        for img in sorted(ov.glob("*.jpg")):
            copy(img, DELIV / "reports" / "overlays" / img.name)

    # ---- environment ----------------------------------------------------
    envdir = WORK / "reports" / "env"
    for name in ("env_summary.json", "pip_freeze_train.txt", "pip_freeze_rknn.txt"):
        src = envdir / name
        if src.exists():
            copy(src, DELIV / "docs" / name)

    # ---- working record -------------------------------------------------
    copy(ROOT / "TRAINING_STATUS.md", DELIV / "TRAINING_STATUS.md")

    # ---- logs -----------------------------------------------------------
    logs = ROOT / "logs"
    for name in ("smoke_train.log", "exp1_warm_train.log", "exp2_scratch_train.log",
                 "exp3_rot_train.log", "val_exp1_test.log", "stage1_audit.log", "stage2_build.log",
                 "pip_torch_install.log", "pip_yolov5_requirements.log"):
        src = logs / name
        if src.exists():
            copy(src, DELIV / "logs" / name)

    # ---- selected-model info -------------------------------------------
    opt = best_run / "opt.yaml"
    info = {
        "selected_experiment": args.best_exp,
        "rerun_experiment": args.run2_exp,
        "weights": {},
    }
    for name in ("best.pt", "best.onnx", "best.torchscript"):
        p = best_run / "weights" / name
        if p.exists():
            info["weights"][name] = {"bytes": p.stat().st_size, "sha256": sha256(p)}
    if rknn.exists():
        info["weights"][rknn.name] = {"bytes": rknn.stat().st_size, "sha256": sha256(rknn)}
    if opt.exists():
        info["opt_yaml"] = opt.read_text(encoding="utf-8", errors="replace")
    (DELIV / "models" / "model_manifest.json").write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")

    # ---- SHA256SUMS -----------------------------------------------------
    lines = []
    for p in sorted(DELIV.rglob("*")):
        if p.is_file() and p.name != "SHA256SUMS.txt":
            lines.append(f"{sha256(p)}  {p.relative_to(DELIV).as_posix()}")
    (DELIV / "SHA256SUMS.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    total = sum(p.stat().st_size for p in DELIV.rglob("*") if p.is_file())
    print(json.dumps({"files": len(lines), "total_mb": round(total / 1e6, 1), "root": str(DELIV)}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
