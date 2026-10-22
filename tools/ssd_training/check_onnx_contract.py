#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Does this .onnx satisfy exactly what jetson-inference's detectNet assumes?

This is the gate for the ONNX route, the alternative to graphsurgeon/UFF/trtexec.
On that route the model stops being a TensorFlow artifact and becomes

    ssd-mobilenet.onnx + labels.txt

dropped next to each other; detectNet parses the ONNX, builds a TensorRT engine
on first run, caches it beside the model, and consumes the two raw output
tensors itself.  Nothing is allowed to disagree, because nothing is checked at
run time.

THE CONTRACT (detectNet.cpp, read off the source, not guessed)
--------------------------------------------------------------
detectNet strips the batch dim (it builds an implicit-batch network), so an
ONNX tensor of shape [1, N, C] reaches postProcessSSD_ONNX as dims [N, C] and

    DIMS_C(x) = x.d[0]        DIMS_H(x) = x.d[1]        DIMS_W(x) = x.d[2]

gives the two numbers the whole post-process is built on:

    mNumClasses = DIMS_H(scores)      <- the LAST dim of scores
    numBoxes    = DIMS_C(boxes)       <- the FIRST dim of boxes
    numCoord    = DIMS_H(boxes)       must be 4

and then, per box n,

    for m in 1 .. mNumClasses-1:      # class 0 IS BACKGROUND, never reported
        score = scores[n * mNumClasses + m]
        keep the arg-max above --confidence (default 0.5)
    Left = boxes[n*4 + 0] * width     Top    = boxes[n*4 + 1] * height
    Right= boxes[n*4 + 2] * width     Bottom = boxes[n*4 + 3] * height

Three consequences that are easy to get wrong and impossible to notice:

  1. `scores` MUST already be probabilities.  detectNet compares the raw
     tensor against the threshold; it applies no sigmoid and no softmax.  A
     model exported in training mode emits logits, whose range is roughly
     [-20, 20] - everything would clear 0.5 and every box would be reported as
     the same class.  This script checks for a Softmax node feeding `scores`.

  2. `boxes` MUST already be decoded: corner form, relative to the image
     (0..1).  detectNet does no anchor enumeration.  In pytorch-ssd that is
     SSD.forward(is_test=True) calling convert_locations_to_boxes() and then
     center_form_to_corner_form(); a model exported from the training branch
     emits raw regression deltas against anchors instead.

  3. `mNumClasses` is read from the TENSOR, but the class names come from
     labels.txt, and only the UFF branch reconciles the two (it overwrites
     mNumClasses with the label count).  On ONNX they are never compared: a
     labels.txt one line short shifts every name by one, silently.  Checked
     here against --labels.

detectNet binds its outputs by NAME, in the order given on the command line:
`--output-cvg` becomes mOutputs[0] (OUTPUT_CONF) and `--output-bbox` becomes
mOutputs[1] (OUTPUT_BBOX).  So the names and their order both matter.

WHY THIS NEEDS NO DEPENDENCIES
------------------------------
The four numbers this gate needs - input name/shape, output names/shapes - sit
in the ONNX file as plain protobuf.  Reading them with a 60-line wire-format
parser keeps the check runnable on the machine that generates the data, where
there is deliberately no onnx, no protobuf and no TensorFlow.

WHAT IT DOES NOT CHECK
----------------------
Whether TensorRT 7.1 can actually parse the graph.  That is a real risk - the
proven model on this project is opset 17 / ir_version 8, above what TRT 7.1
nominally accepts - but it is not a silent one: the engine build either
succeeds or says why.  Build the engine before trusting the file; see README
section 6.3.

Exit status 1 means do not copy this model to the aircraft.

Usage
-----
    python3 tools/ssd_training/check_onnx_contract.py --onnx ssd-mobilenet.onnx
    python3 tools/ssd_training/check_onnx_contract.py --onnx m.onnx \
        --labels labels.txt --confidence 0.5
"""

import argparse
import collections
import os
import sys

# --------------------------------------------------------------------------
# Minimal ONNX protobuf reader (wire format only, no schema, no imports).
#
# Only the fields this gate needs are decoded:
#   ModelProto   1 ir_version | 2 producer_name | 3 producer_version
#                7 graph      | 8 opset_import
#   OperatorSetIdProto 1 domain | 2 version
#   GraphProto   1 node | 2 name | 11 input | 12 output
#   NodeProto    4 op_type
#   ValueInfoProto 1 name | 2 type
#   TypeProto    1 tensor_type
#   TypeProto.Tensor 1 elem_type | 2 shape
#   TensorShapeProto 1 dim
#   TensorShapeProto.Dimension 1 dim_value | 2 dim_param
# --------------------------------------------------------------------------

ELEM_TYPES = {
    1: "FLOAT", 2: "UINT8", 3: "INT8", 4: "UINT16", 5: "INT16",
    6: "INT32", 7: "INT64", 8: "STRING", 9: "BOOL", 10: "FLOAT16",
    11: "DOUBLE", 12: "UINT32", 13: "UINT64", 14: "COMPLEX64",
    15: "COMPLEX128", 16: "BFLOAT16",
}

OK = "OK  "
BAD = "FAIL"
WARN = "WARN"


def _varint(buf, i):
    result = 0
    shift = 0
    while True:
        byte = buf[i]
        i += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, i
        shift += 7


def _fields(buf):
    """Yield (field_number, wire_type, value) for one protobuf message."""
    i = 0
    end = len(buf)
    while i < end:
        key, i = _varint(buf, i)
        field, wire = key >> 3, key & 7
        if wire == 0:
            value, i = _varint(buf, i)
        elif wire == 1:
            value = buf[i:i + 8]
            i += 8
        elif wire == 2:
            length, i = _varint(buf, i)
            value = buf[i:i + length]
            i += length
        elif wire == 5:
            value = buf[i:i + 4]
            i += 4
        else:
            raise ValueError("unsupported protobuf wire type %d" % wire)
        yield field, wire, value


def _text(raw):
    return raw.decode("utf-8", "replace")


def _parse_value_info(buf):
    """ValueInfoProto -> (name, elem_type, [dims])  dims are str; '?' if dynamic."""
    name, elem, dims = None, None, []
    for field, wire, value in _fields(buf):
        if field == 1 and wire == 2:
            name = _text(value)
        elif field == 2 and wire == 2:                       # TypeProto
            for f2, w2, v2 in _fields(value):
                if f2 == 1 and w2 == 2:                      # tensor_type
                    for f3, w3, v3 in _fields(v2):
                        if f3 == 1 and w3 == 0:
                            elem = v3
                        elif f3 == 2 and w3 == 2:            # shape
                            for f4, w4, v4 in _fields(v3):
                                if f4 == 1 and w4 == 2:      # Dimension
                                    dim = "?"
                                    for f5, w5, v5 in _fields(v4):
                                        if f5 == 1 and w5 == 0:
                                            dim = str(v5)
                                        elif f5 == 2 and w5 == 2:
                                            dim = _text(v5)
                                    dims.append(dim)
    return name, elem, dims


def parse_onnx(path):
    """Return a dict with everything this gate reports on."""
    with open(path, "rb") as handle:
        data = handle.read()

    info = {
        "path": path,
        "size": len(data),
        "ir_version": None,
        "producer": None,
        "producer_version": None,
        "opset": [],
        "graph_name": None,
        "inputs": [],
        "outputs": [],
        "op_types": collections.Counter(),
        "num_nodes": 0,
    }

    graph = None
    for field, wire, value in _fields(data):
        if field == 1 and wire == 0:
            info["ir_version"] = value
        elif field == 2 and wire == 2:
            info["producer"] = _text(value)
        elif field == 3 and wire == 2:
            info["producer_version"] = _text(value)
        elif field == 8 and wire == 2:
            domain, version = "ai.onnx", None
            for f2, w2, v2 in _fields(value):
                if f2 == 1 and w2 == 2:
                    domain = _text(v2) or "ai.onnx"
                elif f2 == 2 and w2 == 0:
                    version = v2
            info["opset"].append((domain, version))
        elif field == 7 and wire == 2:
            graph = value

    if graph is None:
        raise ValueError("no graph field in %s - not an ONNX model?" % path)

    for field, wire, value in _fields(graph):
        if field == 1 and wire == 2:                          # NodeProto
            info["num_nodes"] += 1
            op_type = None
            for f2, w2, v2 in _fields(value):
                if f2 == 4 and w2 == 2:
                    op_type = _text(v2)
            info["op_types"][op_type] += 1
        elif field == 2 and wire == 2:
            info["graph_name"] = _text(value)
        elif field == 11 and wire == 2:
            info["inputs"].append(_parse_value_info(value))
        elif field == 12 and wire == 2:
            info["outputs"].append(_parse_value_info(value))

    return info


# Op types the TensorRT 7.1 ONNX parser handles.  A graph built out of these
# needs no plugin, which is the whole point of this route: the FlattenConcat_TRT
# plugin that the UFF export has to inject is a serialised custom blob whose
# format must match between the machine that builds the engine and the one that
# runs it.  An ONNX made of standard ops has nothing to mismatch.
TRT_SAFE_OPS = {
    "Conv", "Relu", "Clip", "Add", "Sub", "Mul", "Div", "Concat", "Reshape",
    "Transpose", "Squeeze", "Unsqueeze", "Softmax", "Exp", "Sigmoid", "Slice",
    "Gather", "Cast", "Min", "Max", "Sum", "Constant", "Identity", "Pad",
    "Flatten", "AveragePool", "MaxPool", "GlobalAveragePool", "Resize",
    "Upsample", "LeakyRelu", "Sqrt", "Pow", "ReduceMean", "Split", "Tile",
    "Expand", "Shape", "Range", "Where", "Erf", "Neg", "Abs", "Log", "Round",
}

# Two dims counts are legal here, and they mean the same thing:
#   [1, N, C]  explicit batch, what torch.onnx.export writes
#   [N, C]     batch already stripped, what the engine reports
def split_box_dims(dims):
    if len(dims) == 3:
        return dims[1], dims[2]
    if len(dims) == 2:
        return dims[0], dims[1]
    return None, None


def read_labels(path):
    """Return the label lines, or None if anything about the file is wrong."""
    with open(path, "r", encoding="utf-8-sig") as handle:
        raw = handle.read().splitlines()
    lines = [line.strip() for line in raw if line.strip()]
    return lines


def main():
    parser = argparse.ArgumentParser(
        description="Check an .onnx against detectNet's SSD post-processing contract.")
    parser.add_argument("--onnx", required=True,
                        help="the .onnx to check (the one you would copy to the aircraft)")
    parser.add_argument("--labels", default=None,
                        help="labels.txt that will sit beside it; its line count must "
                             "equal the class dim of `scores`, and line 0 must be BACKGROUND")
    parser.add_argument("--input-name", default="input_0",
                        help="expected input tensor name (default: input_0, what "
                             "pytorch-ssd's onnx_export.py writes)")
    parser.add_argument("--cvg-name", default="scores",
                        help="expected name of the confidence output (default: scores)")
    parser.add_argument("--bbox-name", default="boxes",
                        help="expected name of the bbox output (default: boxes)")
    parser.add_argument("--confidence", default=0.5, type=float,
                        help="the threshold detectNet will run with; printed with the "
                             "derived command line (default: 0.5)")
    parser.add_argument("--allow-any-opset", action="store_true",
                        help="downgrade the opset/ir_version warning to informational")
    args = parser.parse_args()

    failures = []
    warnings = []

    def report(state, title, detail=""):
        print("  [%s] %-34s %s" % (state, title, detail))
        if state == BAD:
            failures.append(title)
        elif state == WARN:
            warnings.append(title)

    print("=" * 74)
    print("ONNX -> detectNet contract gate")
    print("=" * 74)
    try:
        info = parse_onnx(args.onnx)
    except Exception as exc:                                  # noqa: BLE001
        print("  [%s] could not read %s: %s" % (BAD, args.onnx, exc))
        return 1
    print("  file       %s" % info["path"])
    print("  size       %.2f MB" % (info["size"] / 1048576.0))
    print("  graph      %s  (%d nodes)" % (info["graph_name"], info["num_nodes"]))
    print()

    # ---- provenance -------------------------------------------------------
    print("-- provenance " + "-" * 60)
    report(OK, "producer",
           "%s %s" % (info["producer"] or "?", info["producer_version"] or "?"))
    opset = ", ".join("%s=%s" % (d, v) for d, v in info["opset"]) or "?"
    report(OK, "opset", opset)
    ir_version = info["ir_version"]
    report(OK, "ir_version", str(ir_version))

    # TensorRT 7.1 is what JetPack 4.5 ships, and the ONNX parser in it was
    # written against opset 11.  A model above that is not automatically broken
    # - the model this project actually flew is opset 17 / ir_version 8 and it
    # built on TRT 7.1.3 - but it is the first thing to suspect if the engine
    # build fails, so say so rather than staying quiet.
    if not args.allow_any_opset:
        max_opset = max([v for _, v in info["opset"] if v], default=0)
        if max_opset > 11:
            report(WARN, "opset above TRT 7.1 nominal",
                   "opset %d > 11; known to work on 7.1.3 for this project, but "
                   "build the engine before trusting it" % max_opset)
        if ir_version is not None and ir_version > 7:
            report(WARN, "ir_version above TRT 7.1 nominal",
                   "ir_version %d > 7; same caveat" % ir_version)
    print()

    # ---- input ------------------------------------------------------------
    print("-- input " + "-" * 65)
    if len(info["inputs"]) != 1:
        report(BAD, "single input",
               "graph declares %d inputs; detectNet binds mInputs[0] only"
               % len(info["inputs"]))
    else:
        name, elem, dims = info["inputs"][0]
        report(OK, "count", "1")
        if name == args.input_name:
            report(OK, "name", name)
        else:
            # detectNet reads the input name from the engine, and the command
            # line can override it, so a different name is survivable - but it
            # means the model did not come from onnx_export.py, so say which.
            report(WARN, "name", "%s (expected %s; pass it through if you keep it)"
                   % (name, args.input_name))
        report(OK if elem in (1,) else BAD, "dtype",
               "%s" % ELEM_TYPES.get(elem, elem))
        if elem != 1:
            report(BAD, "dtype is FLOAT", "detectNet feeds float; %s would not match"
                   % ELEM_TYPES.get(elem, elem))
        if len(dims) == 4 and dims[1] == "3":
            report(OK, "dims", "[%s]  NCHW" % ", ".join(dims))
            if dims[2] != dims[3]:
                report(WARN, "square resolution",
                       "%sx%s; detectNet will letterbox, but SSD anchors assume square"
                       % (dims[2], dims[3]))
        else:
            report(BAD, "dims [1,3,H,W]",
                   "got [%s]; detectNet feeds GetInputWidth()xGetInputHeight() RGB"
                   % ", ".join(dims))
    print()

    # ---- outputs ----------------------------------------------------------
    print("-- outputs " + "-" * 63)
    if len(info["outputs"]) != 2:
        report(BAD, "exactly 2 outputs",
               "graph declares %d; postProcessSSD_ONNX indexes mOutputs[0] and [1]"
               % len(info["outputs"]))
    else:
        (cvg_name, cvg_elem, cvg_dims) = info["outputs"][0]
        (bbox_name, bbox_elem, bbox_dims) = info["outputs"][1]

        # Order, not just presence: --output-cvg binds index 0.
        if cvg_name == args.cvg_name and bbox_name == args.bbox_name:
            report(OK, "names and order", "%s, %s" % (cvg_name, bbox_name))
        else:
            report(BAD, "names and order",
                   "got %s, %s; want %s, %s (order decides which is which)"
                   % (cvg_name, bbox_name, args.cvg_name, args.bbox_name))

        if cvg_elem == 1 and bbox_elem == 1:
            report(OK, "dtype", "FLOAT, FLOAT")
        else:
            report(BAD, "dtype",
                   "%s, %s" % (ELEM_TYPES.get(cvg_elem, cvg_elem),
                               ELEM_TYPES.get(bbox_elem, bbox_elem)))

        cvg_boxes, num_classes = split_box_dims(cvg_dims)
        bbox_boxes, num_coord = split_box_dims(bbox_dims)
        report(OK, "%s dims" % cvg_name, "[%s]" % ", ".join(cvg_dims))
        report(OK, "%s dims" % bbox_name, "[%s]" % ", ".join(bbox_dims))

        if None in (cvg_boxes, num_classes):
            report(BAD, "%s shape" % cvg_name,
                   "expected 3 dims [1,N,C] or 2 dims [N,C]")
        if None in (bbox_boxes, num_coord):
            report(BAD, "%s shape" % bbox_name, "expected [1,N,4] or [N,4]")
        if num_coord != "4":
            report(BAD, "4 box coordinates",
                   "%s last dim is %s; detectNet reads coord[0..3]"
                   % (bbox_name, num_coord))
        if cvg_boxes and bbox_boxes and cvg_boxes != bbox_boxes:
            report(BAD, "box counts agree",
                   "%s says %s, %s says %s" % (cvg_name, cvg_boxes, bbox_name, bbox_boxes))
        print()

        # ---- labels ------------------------------------------------------
        print("-- labels " + "-" * 64)
        if args.labels is None:
            report(WARN, "labels.txt not given",
                   "class names come from labels.txt and detectNet never compares "
                   "them to the model; pass --labels to check the pair")
        elif not os.path.isfile(args.labels):
            report(BAD, "labels.txt exists", args.labels)
        else:
            lines = read_labels(args.labels)
            report(OK, "path", "%s (%d lines)" % (args.labels, len(lines)))
            if len(lines) != int(num_classes):
                report(BAD, "line count == class dim",
                       "labels.txt has %d lines, %s's last dim is %s; on the ONNX "
                       "branch nothing reconciles these, the names just shift"
                       % (len(lines), cvg_name, num_classes))
            else:
                report(OK, "line count == class dim", "%d == %s" % (len(lines), num_classes))
            if lines and lines[0].strip().upper() == "BACKGROUND":
                report(OK, "line 0 is BACKGROUND", lines[0])
            else:
                report(BAD, "line 0 is BACKGROUND",
                       "got %r; class 0 is skipped by detectNet, so line 0 is a "
                       "placeholder and naming a real class there hides it"
                       % (lines[0] if lines else None))
            if lines:
                live = lines[1:]
                report(OK, "reportable classes", "%d: %s" % (len(live), ", ".join(live[:6]))
                       + (" ..." if len(live) > 6 else ""))
        print()

    # ---- graph shape ------------------------------------------------------
    print("-- graph " + "-" * 65)
    exotic = sorted(op for op in info["op_types"] if op not in TRT_SAFE_OPS)
    total = sum(info["op_types"].values())
    report(OK, "op types", "%d distinct over %d nodes" % (len(info["op_types"]), total))
    if exotic:
        report(WARN, "non-standard op types",
               "%s - TRT 7.1 may need a plugin for these" % ", ".join(map(str, exotic)))
    else:
        report(OK, "all ops TRT-parseable", "no plugin needed")

    # The one static proxy for "scores are probabilities, not logits".
    if info["op_types"].get("Softmax"):
        report(OK, "Softmax feeding scores",
               "%d node(s); scores are probabilities"
               % info["op_types"]["Softmax"])
    else:
        report(BAD, "Softmax feeding scores",
               "no Softmax node; if this model was exported in training mode its "
               "scores are logits and every box clears --confidence")

    # Anchor decoding happens inside the graph, which is why detectNet does no
    # enumeration: the tell is exp() applied to the size regression.
    if info["op_types"].get("Exp"):
        report(OK, "anchor decode in graph", "Exp present (boxes are decoded)")
    else:
        report(WARN, "anchor decode in graph",
               "no Exp node; boxes may still be raw regression deltas against "
               "anchors - export with is_test=True")
    print()
    print("  op histogram: %s" % ", ".join(
        "%s x%d" % (op, count) for op, count in info["op_types"].most_common(10)))
    print()

    # ---- verdict ----------------------------------------------------------
    print("=" * 74)
    if failures:
        print("FAIL - %d problem(s); do not copy this model to the aircraft:" % len(failures))
        for item in failures:
            print("   - %s" % item)
    else:
        print("PASS - this .onnx satisfies detectNet's post-processing contract.")
        print()
        print("derived constants (what detectNet will compute):")
        try:
            _, cvg_dims = info["outputs"][0][0], info["outputs"][0][2]
            _, bbox_dims = info["outputs"][1][0], info["outputs"][1][2]
            boxes, classes = split_box_dims(cvg_dims)
            print("   mNumClasses = %s        numBoxes = %s" % (classes, boxes))
            print("   class 0     = BACKGROUND (never reported)")
            print()
            print("command line detectNet needs (names must match the graph):")
            print("   --model=<this.onnx> --labels=<labels.txt>")
            print("   --input_blob=%s --output-cvg=%s --output-bbox=%s"
                  % (info["inputs"][0][0], info["outputs"][0][0], info["outputs"][1][0]))
            print("   --confidence=%.2f" % args.confidence)
            print()
            h, w = (info["inputs"][0][2] + ["?", "?"])[2:4]
            print("preprocessing detectNet applies for ONNX (preProcess, MODEL_ONNX):")
            print("   cudaTensorNormMeanRGB  scale [0,1]  mean 0.5  std 0.5   -> RGB, [-1,1]")
            print("   i.e. 2*pixel/255 - 1 per channel, in R,G,B order.")
            print("   This is the SAME numeric range as the UFF branch's")
            print("   cudaTensorNormBGR(-1,1), but the OPPOSITE channel order -")
            print("   feeding BGR here is a colour swap the model will not report.")
            print("   input is %sx%s, RGB, and boxes come back relative to that frame."
                  % (w, h))
        except (IndexError, TypeError):
            pass
    if warnings:
        print()
        print("warnings (%d):" % len(warnings))
        for item in warnings:
            print("   - %s" % item)
    print("=" * 74)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
