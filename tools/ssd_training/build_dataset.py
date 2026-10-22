#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build the detector's training dataset: gate, render, merge, report.

WHERE THIS SITS IN THE WHOLE JOB
--------------------------------
  1.  build the dataset                          <- this script (no TF, no GPU)
  2.  lay it out for pytorch-ssd                 <- prepare_pytorch_voc.py, this machine
  3.  train, export ONNX, gate it               <- train_pytorch_ssd.py, a machine with PyTorch
  4.  copy ssd-mobilenet.onnx + labels.txt       <- Jetson, then fly it

Steps 3 and 4 used to be a TensorFlow 1.15 container plus frozen graph -> UFF ->
.plan.  That chain is gone: the model now stops at ONNX and jetson-inference's
detectNet reads it directly, so there is no .plan, no graphsurgeon, and no
TensorFlow on the aircraft.  What survives from the old chain is
`voc_split.pick_split` - still the one place the train/val split is
decided, so both routes keep training on the same images in the same folds.

This script is step 1 only, because it is the half that runs on this machine.
It runs the pieces in the one order that is safe:

    gate  ->  render the synthetic half  ->  convert + append the real half
          ->  report the split and the class balance

The gate runs FIRST and aborts the build if it fails.  Merging two halves that
disagree about what a box covers trains the regressor on the average of two
wrong boxes; the centre stays right, so the drop still lands and nothing
downstream ever notices.  That is a failure you cannot detect after the fact,
only prevent before it.

THE TWO HALVES DO DIFFERENT JOBS
--------------------------------
  synthetic  the nine GB 2894 warning signs, exact geometry, exact scale,
             flat lighting
  real       4 of the ten classes, photographed: ochre print, thicker black
             rule, specular sheen, more oblique perspective

Six classes exist ONLY in the synthetic half (1-06, 1-27, 1-28, 1-42, 1-43 and
1-04), so training on the photographs alone silently produces a detector that
can never fire them.  Both halves go in the same directory on purpose; the
`real_` prefix is how you tell them apart afterwards.

THE SYNTHETIC HALF DOES NOT RENDER THE PERSON CARD (default since 2026-10-07)
----------------------------------------------------------------------------
`--synth-zones hazard` is the default.  Reasons:

  * the person class already has real photographs (30 of the 120), so it is
    not one of the classes that depends on synthetic samples;
  * the person artwork is hand-drawn - a synthetic portrait is a much weaker
    stand-in for a photographed one than a synthetic GB 2894 pictogram is,
    because the pictogram is a flat two-colour print either way;
  * and a synthetic portrait that is *almost* right teaches the classifier a
    boundary that the real card does not sit on.

Use `--synth-zones both` to go back to rendering person frames as well.

Usage
-----
    python3 tools/ssd_training/build_dataset.py --out data/task_ds
    python3 tools/ssd_training/build_dataset.py --out data/task_ds --synth-count 8000
    python3 tools/ssd_training/build_dataset.py --out data/task_ds --real-only
    python3 tools/ssd_training/build_dataset.py --out data/task_ds --synth-zones both
"""

import argparse
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import labels as L  # noqa: E402


def run(label, argv):
    print("\n== %s" % label)
    print("   %s" % " ".join(argv))
    result = subprocess.run(argv)
    if result.returncode != 0:
        raise SystemExit("\n%s failed (exit %d) - stopping before anything is "
                         "merged." % (label, result.returncode))
    return result.returncode


def check_textures():
    """Every texture labels.py names must exist, or the renderer draws nothing."""
    missing = [n for n in L.CLASS_NAMES
               if not os.path.exists(L.texture_path(n))]
    if missing:
        raise SystemExit(
            "missing texture(s) for %s.\n"
            "Run: python3 tools/make_task_markers.py" % ", ".join(missing))
    print("textures   : %d present" % len(L.CLASS_NAMES))


def count_prefix(directory, pattern, prefix):
    return len([p for p in glob.glob(os.path.join(directory, pattern))
                if os.path.basename(p).startswith(prefix)])


def main():
    ap = argparse.ArgumentParser(
        description="Build the synthetic + real training dataset.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True,
                    help="dataset root; images/ and annotations/ go inside")
    ap.add_argument("--synth-count", type=int, default=4000,
                    help="synthetic frames to render (default 4000)")
    ap.add_argument("--synth-zones", default="hazard",
                    choices=["hazard", "person", "both"],
                    help="which zones the synthetic half renders.  Default "
                         "'hazard' = warning signs only, no person card: the "
                         "person class is carried by the real photographs.  "
                         "'both' restores the old mixed set.")
    ap.add_argument("--seed", type=int, default=20261006)
    ap.add_argument("--val-fraction", type=float, default=0.12)
    ap.add_argument("--real-src", default=L.REAL_DATASET_DIR)
    ap.add_argument("--synth-only", action="store_true")
    ap.add_argument("--real-only", action="store_true",
                    help="append the photographs to an existing set, render "
                         "nothing")
    ap.add_argument("--skip-gate", action="store_true",
                    help="do not run check_box_convention.py first.  Only for "
                         "re-running after nothing about the box changed.")
    args = ap.parse_args()

    python = sys.executable
    out = os.path.abspath(args.out)
    print("dataset    : %s" % out)
    print("python     : %s" % python)

    check_textures()

    if not args.skip_gate:
        run("box-convention gate", [python,
                                    os.path.join(HERE, "check_box_convention.py")])

    if not args.real_only:
        run("render the synthetic half", [
            python, os.path.join(HERE, "gen_synth_dataset.py"),
            "--out", out, "--count", str(args.synth_count),
            "--zones", args.synth_zones,
            "--seed", str(args.seed)])
        if args.synth_zones == "hazard":
            print("   note: zones=hazard, so every 'person_injured' box in "
                  "the merged set\n         comes from a real photograph.  "
                  "There will be no synthetic person samples.")

    if not args.synth_only:
        if not os.path.isdir(os.path.join(args.real_src, "Annotations")):
            print("\n== real half SKIPPED: no Annotations/ under %s"
                  % args.real_src)
            print("   the set is not on this machine, or it moved.  Everything "
                  "below\n   is still valid, but six classes will have only "
                  "synthetic samples.")
        else:
            run("convert + append the real photographs", [
                python, os.path.join(HERE, "prepare_real_dataset.py"),
                "--src", args.real_src, "--out", out, "--prefix", "real_",
                # Zoom each photograph until its box covers the fraction of the
                # frame the aircraft actually sees.  Without this the real half
                # arrives at 0.76x of the deployment scale.
                "--fit-box-fraction",
                "%.3f" % L.deployment_box_fraction("hazard_flammable")])

    label_map = os.path.join(out, "label_map.pbtxt")
    if not os.path.exists(label_map):
        with open(label_map, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(L.label_map_pbtxt())
        print("\nwrote %s" % label_map)

    images = os.path.join(out, "images")
    annotations = os.path.join(out, "annotations")
    n_synth = count_prefix(images, "*.jpg", "synth_")
    n_real = count_prefix(images, "*.jpg", "real_")
    print("\n== what is in the dataset now")
    print("   %-26s %6d" % ("synthetic frames", n_synth))
    print("   %-26s %6d" % ("photographs", n_real))
    print("   %-26s %6d" % ("total", n_synth + n_real))
    if n_synth == 0 or n_real == 0:
        print("   NOTE: only one half is present.  See the README section on why "
              "both\n         are needed before training.")

    # The split report needs only the XMLs, so it runs here rather than in a
    # training environment - and it is the only way to get the real per-class
    # counts instead of guessing them.  Its output is captured rather than
    # inherited so the numbers can be lifted into the commands printed below.
    report = subprocess.run(
        [python, os.path.join(HERE, "voc_split.py"),
         "--images", images, "--annotations", annotations,
         "--label-map", label_map,
         "--val-fraction", str(args.val_fraction), "--seed", str(args.seed)],
        capture_output=True, encoding="utf-8", errors="replace")
    if report.stdout:
        sys.stdout.write(report.stdout)
    if report.returncode != 0:
        if report.stderr:
            sys.stderr.write(report.stderr)
        raise SystemExit("\nthe split report failed - do not train on this set.")

    val_examples = None
    for line in (report.stdout or "").splitlines():
        if line.strip().startswith("val images ="):
            val_examples = line.split("=")[-1].strip()
    if val_examples is None:
        raise SystemExit(
            "\nthe split report did not print the val image count, so the "
            "commands below cannot be filled in.  Do not guess it.")

    with open(os.path.join(out, "dataset_info.txt"), "w",
              encoding="utf-8", newline="\n") as fh:
        fh.write("built by tools/ssd_training/build_dataset.py\n")
        fh.write("synthetic frames : %d (seed %d)\n" % (n_synth, args.seed))
        fh.write("synthetic zones  : %s%s\n"
                 % (args.synth_zones,
                    "  <- person class is real-photographs only"
                    if args.synth_zones == "hazard" else ""))
        fh.write("photographs      : %d (%s)\n" % (n_real, args.real_src))
        fh.write("real zoom        : --fit-box-fraction %.3f\n"
                 % L.deployment_box_fraction("hazard_flammable"))
        fh.write("val fraction     : %.2f\n" % args.val_fraction)
        fh.write("classes          : %d\n" % L.NUM_CLASSES)
        for i, name in enumerate(L.CLASS_NAMES, start=1):
            fh.write("  %2d %s\n" % (i, name))
    print("\nwrote %s" % os.path.join(out, "dataset_info.txt"))

    print("""
=============================================================
 STEP 2 - lay the data out for pytorch-ssd (this machine)
=============================================================
   python3 tools/ssd_training/prepare_pytorch_voc.py --out data/voc_task

   It reuses THIS script's split function (voc_split.pick_split), so the
   PyTorch route and the old TF route train on the same images in the same
   folds.  It also writes data/models/task/labels.txt - the 11-line file
   (BACKGROUND + %(nc)d classes) that both detection nodes point at.

=============================================================
 STEP 3 - train + export + gate (a machine with PyTorch)
=============================================================
   # use YOUR conda python; the script trains with sys.executable
   python tools/ssd_training/train_pytorch_ssd.py \\
       --data data/voc_task --model-dir data/models/task --epochs 100

   It runs jetson-inference's own train_ssd.py and onnx_export.py unmodified,
   then runs check_onnx_contract.py on what came out.  That gate is the verdict:
   onnx_export.py returning 0 only means it wrote a file, not that detectNet
   can read it.

=============================================================
 STEP 4 - fly it
=============================================================
   Copy TWO files into the package the runtime reads:
   src/ssd_detection/models/ssd-mobilenet.onnx and
   src/ssd_detection/models/labels.txt.  Nothing else - no .plan, no .uff, no
   graphsurgeon, and no TensorFlow on the aircraft.  The launch defaults are
   $(find ssd_detection)/models/..., so simulation and aircraft read the same
   path, and catkin installs the models/ directory with the package.

   # look at the boxes first, without ROS (the first run builds the engine)
   rosrun ssd_detection ssd_detection_node \\
       --model src/ssd_detection/models/ssd-mobilenet.onnx \\
       --labels src/ssd_detection/models/labels.txt --dir <some images> --out /tmp/ssd_preview

   # then into the mission (the same .onnx the simulator reads)
   # 没有后端开关了：检测器由 navigation_2d.launch 无条件 include。
   # 真机：use_usb_cam:=true 同时把图像源（GStreamer 直采）和检测实现
   #      （C++ detectNet）切过去；仿真不加这个参数，走 Gazebo 相机 + Python。
   roslaunch navigation_2d navigation_2d.launch use_usb_cam:=true
=============================================================
""" % {"out": args.out.replace("\\", "/"), "vf": args.val_fraction,
       "nc": L.NUM_CLASSES, "ve": val_examples})
    return 0


if __name__ == "__main__":
    sys.exit(main())
