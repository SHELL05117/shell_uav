# -*- coding: utf-8 -*-
"""adapt_external_onnx.py —— 把"外来 ONNX"适配成 detectNet 契约的图。

背景（2026-10-07 实测）：队友用 tf2onnx 转出来的 ssd_mobilenet_v1_4class.onnx
训练得很好（120 张实拍 119 张 IoU>0.3、mean IoU 0.953），但和 detectNet 的
ONNX 分支有四处不兼容，任何一处都足以让它"engine 正常、永远零检出"：

  1. 预处理：图要 **0..255 原始值**，而 detectNet 固定做 (x/255-0.5)/0.5
     （cudaTensorNormMeanRGB，改不了）⇒ 在图入口补 Mul(127.5)+Add(127.5)，
     把 detectNet 喂进来的 [-1,1] 还原回 [0,255]。
  2. 输出顺序：外来图 0=boxes、1=scores，detectNet 按**顺序**绑 0=scores /
     1=boxes（jetson_detector.cpp 顶注第 2 条）⇒ 输出重排成 scores 在前。
  3. 输出名：Identity:0 / Identity_1:0 ⇒ 改名 scores / boxes。
  4. 类槽：外来图 class 0 是真实类（人员），而 detectNet 的
     postProcessSSD_ONNX **跳过 class 0（当背景）** ⇒ 在 scores 最前面拼一列
     全 0 的背景列（[1,N,C] -> [1,N,C+1]），原类号整体 +1，人员从 0 变 1。

用法（boxes-index 用 probe 结果填；本模型实测 0=boxes）：
  python adapt_external_onnx.py --onnx in.onnx --out out.onnx --boxes-index 0

适配前必须先用实拍标注图投票确认输出顺序（本仓库 2026-10-07 的探针做法），
两个输出形状常常完全相同，静态分不出来。

改完的图用 tools/ssd_training/check_onnx_contract.py 过门禁，再拿
ssd_detection_node.py --image 离线跑几张图确认，才能拷上机。
"""
from __future__ import print_function

import argparse

import numpy as np
import onnx
from onnx import helper, TensorProto


def adapt(model_path, out_path, boxes_index, denorm):
    model = onnx.load(model_path)
    graph = model.graph

    inputs = [vi.name for vi in graph.input]
    if len(inputs) != 1:
        raise SystemExit("expect exactly 1 graph input, got %r" % (inputs,))
    old_input = inputs[0]

    outputs = [vi.name for vi in graph.output]
    if len(outputs) != 2:
        raise SystemExit("expect exactly 2 graph outputs, got %r" % (outputs,))
    scores_index = 1 - boxes_index
    old_boxes_name = outputs[boxes_index]
    old_scores_name = outputs[scores_index]

    # ---- 形状信息（沿用原输出的形状；scores 的类维后面会 +1）-------------
    def shape_of(vi):
        return [d.dim_value if d.dim_value else d.dim_param for d in
                vi.type.tensor_type.shape.dim]

    boxes_vi = graph.output[boxes_index]
    scores_vi = graph.output[scores_index]
    boxes_shape = shape_of(boxes_vi)
    scores_shape = shape_of(scores_vi)
    if len(scores_shape) != 3 or len(boxes_shape) != 3:
        raise SystemExit("expect [1,N,C]/[1,N,4], got %r / %r"
                         % (scores_shape, boxes_shape))
    num_anchors = scores_shape[1]
    num_classes = scores_shape[2]

    # ---- 1) 入口反归一化：input_0 -> Mul(127.5) -> Add(127.5) -> 原图入口 --
    if denorm:
        for node in graph.node:
            for i, name in enumerate(node.input):
                if name == old_input:
                    node.input[i] = "adapt/denorm_out"
        mul_const = onnx.numpy_helper.from_array(
            np.full((1, 1, 1, 1), 127.5, dtype=np.float32), name="adapt/c127a")
        add_const = onnx.numpy_helper.from_array(
            np.full((1, 1, 1, 1), 127.5, dtype=np.float32), name="adapt/c127b")
        graph.initializer.extend([mul_const, add_const])
        # 插在最前面：ONNX 要求拓扑序（消费者必须在生产者之后）
        graph.node.insert(0, helper.make_node(
            "Add", ["adapt/denorm_mul", "adapt/c127b"], ["adapt/denorm_out"],
            name="adapt/denorm_add"))
        graph.node.insert(0, helper.make_node(
            "Mul", ["input_0", "adapt/c127a"], ["adapt/denorm_mul"],
            name="adapt/denorm_mul"))
        new_input = helper.make_tensor_value_info(
            "input_0", TensorProto.FLOAT, shape_of(graph.input[0]))
        del graph.input[0]
        graph.input.insert(0, new_input)
    else:
        # 只改名不改数值通路：把 old_input 的引用与声明整个换成 input_0
        for node in graph.node:
            for i, name in enumerate(node.input):
                if name == old_input:
                    node.input[i] = "input_0"
        new_input = helper.make_tensor_value_info(
            "input_0", TensorProto.FLOAT, shape_of(graph.input[0]))
        del graph.input[0]
        graph.input.insert(0, new_input)

    # ---- 2)+3) 末端 Identity 改名：boxes 保持，scores 改成中间名 ----------
    renamed = 0
    for node in graph.node:
        if len(node.output) == 1 and node.output[0] == old_boxes_name:
            node.output[0] = "boxes"
            renamed += 1
        elif len(node.output) == 1 and node.output[0] == old_scores_name:
            node.output[0] = "adapt/scores_raw"
            renamed += 1
    if renamed != 2:
        raise SystemExit("expected to rename 2 terminal outputs, renamed %d "
                         "(输出不是 Identity 直出？手工处理)" % renamed)

    # ---- 4) scores 前拼全 0 背景列：[1,N,C] -> [1,N,C+1]，类号整体 +1 ----
    bg = onnx.numpy_helper.from_array(
        np.zeros((1, num_anchors, 1), dtype=np.float32), name="adapt/bg_zero")
    graph.initializer.append(bg)
    graph.node.append(helper.make_node(
        "Concat", ["adapt/bg_zero", "adapt/scores_raw"], ["scores"],
        axis=2, name="adapt/pad_background"))

    # ---- 重写 graph.output：scores 在前（detectNet 按顺序绑 0=scores）-----
    new_scores_shape = [scores_shape[0], scores_shape[1], scores_shape[2] + 1]
    del graph.output[:]
    graph.output.extend([
        helper.make_tensor_value_info("scores", TensorProto.FLOAT,
                                      new_scores_shape),
        helper.make_tensor_value_info("boxes", TensorProto.FLOAT, boxes_shape),
    ])

    onnx.checker.check_model(model)
    onnx.save(model, out_path)
    print("OK  %s" % out_path)
    print("    input_0 %s  (denorm=%s)" % (shape_of(graph.input[0]), denorm))
    print("    scores %s  (class 0 = 全 0 背景，原类号 +1)" % (new_scores_shape,))
    print("    boxes  %s" % (boxes_shape,))
    print("    labels.txt 需要 %d 行：行0=BACKGROUND，行1..%d = 原类 0..%d"
          % (num_classes + 1, num_classes, num_classes - 1))


def main():
    parser = argparse.ArgumentParser(
        description="adapt an external ONNX to the detectNet contract")
    parser.add_argument("--onnx", required=True, help="input .onnx")
    parser.add_argument("--out", required=True, help="output .onnx")
    parser.add_argument("--boxes-index", type=int, default=0,
                        choices=[0, 1],
                        help="graph output index of the BOXES tensor "
                             "(from the annotated-photo probe; default 0)")
    parser.add_argument("--no-denorm", action="store_true",
                        help="model already takes [-1,1] input "
                             "(skip the Mul/Add prepend)")
    args = parser.parse_args()
    adapt(args.onnx, args.out, args.boxes_index, denorm=not args.no_denorm)


if __name__ == "__main__":
    main()
