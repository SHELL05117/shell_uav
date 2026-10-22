#!/usr/bin/env python3
"""ssd_detection_node —— 相机帧进，检测框出。

它只做一件事：把一帧图像送进 SSD，把框按 VisionDetectionArray 发出去。
任务逻辑（选哪个框、稳不稳、投不投）一律不在这里 —— 那在 navigation_2d 的
MissionVision 里，见 README §6.3。

为什么用 OpenCV DNN / onnxruntime 而不是 TensorFlow / TensorRT
--------------------------------------------------------------
仿真侧那台机器没有 CUDA，而 detectNet 是 jetson-inference（Jetson 上才装得了）。
OpenCV 本来就随 cv_bridge 在位，`cv2.dnn` 两种格式都能读，所以本节点按模型
后缀自动选：

  * `.onnx`（**现在的路**）—— 读的就是 Jetson 上 detectNet 读的**同一个
    文件**。仿真与实机共用一份模型，于是"仿真里过的那套权重"和"飞的时候用
    的那套权重"不可能是两件东西。
    读它有两个后端，**onnxruntime 优先**，没装才退回 `cv2.dnn`：训练导出的
    图是 opset 17，而 Noetic 自带的 OpenCV 4.2 读不了 opset>=10 的 Slice
    （报 `Required argument "starts" not found`）。两者输出逐元素等价。
    代价是这个 onnx 必须满足 detectNet 的契约（输入名 `input_0`、输出 0=scores /
    1=boxes、类数含背景、scores 已是概率、boxes 已解码），拷上机前用
    `tools/ssd_training/check_onnx_contract.py` 静态查一遍。
  * `.pb`（旧路径，保留是为了能跟新模型对比）—— `readNetFromTensorflow`。

两种跑法：
  * 作为 ROS 节点（默认）：订阅图像话题，发布检测结果。
  * 离线自检（`--image` / `--dir`）：不碰 ROS，直接跑一张/一个目录，
    打印框并可导出预览图。**换模型之后先跑这个** —— 检测器对不对和
    ROS 通不通是两件事，分开验证能少绕很多路。

    python3 ssd_detection_node.py --model m.onnx --labels l.txt \
        --image card.jpg --out check.jpg
    python3 ssd_detection_node.py --model m.onnx --labels l.txt --dump --image card.jpg
"""
from __future__ import print_function

import argparse
import glob as _glob
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# label 文件 / 类别 -> 任务区
# ---------------------------------------------------------------------------
def read_labels(path):
    """读 ssd_ros 那套 label 文件：一行一类，**行 0 必须是 background**。

    id 就是行号（0 = background），所以行 0 写错会让每一个类号都错位一格，
    而且不会报错 —— 现象是"框的位置对、类别全错"。
    """
    with open(path, "r") as handle:
        labels = [line.strip() for line in handle]
    while labels and not labels[-1]:
        labels.pop()
    if not labels:
        raise ValueError("label 文件是空的: %s" % path)
    # 大小写都认：train_ssd.py 写的是大写 BACKGROUND（它从 VOCDataset 的
    # classes.insert(0,'BACKGROUND') 继承来的），而 ssd_ros 那套 label 文件
    # 用小写。两边都被本工程接受 —— 判据是"第 0 行是不是背景"，不是它长什么样。
    if labels[0].strip().lower() != "background":
        # 不抛异常：有人故意用无 background 的文件也不是不可救，但一定要吼。
        print("[ssd_detection] 警告: %s 的第 0 行是 %r，不是 'background'；"
              "类别号会整体错位一格" % (path, labels[0]), file=sys.stderr)
    return labels


def zone_of(class_name):
    """类别名 -> "person" / "hazard"。

    用前缀而不是写死一张表：类别清单的唯一真相源在 labels.py，把 9 个危险品
    名字抄到节点里，改一次名字就有一处会忘（README 坑清单第 5 条）。
    人员类是唯一以 person 开头的，其余都是危险品。
    """
    return "person" if class_name.startswith("person") else "hazard"


# ---------------------------------------------------------------------------
# 推理核心
# ---------------------------------------------------------------------------
class SsdDetector(object):
    """把一个 frozen_inference_graph.pb 包成 detect(image) -> [检测框]。"""

    def __init__(self, model_path, label_file, config_path="", input_size=300,
                 swap_rb=True, score_threshold=0.5, max_detections=100):
        import cv2  # 只有真正要推理时才 import：离线跑 --help 不需要 cv2

        self.cv2 = cv2
        self.labels = read_labels(label_file)
        self.input_size = int(input_size)
        self.swap_rb = bool(swap_rb)
        self.score_threshold = float(score_threshold)
        self.max_detections = int(max_detections)

        if not os.path.isfile(model_path):
            raise IOError("找不到模型: %s" % model_path)
        if config_path and os.path.isfile(config_path):
            self.net = cv2.dnn.readNetFromTensorflow(model_path, config_path)
        else:
            self.net = cv2.dnn.readNetFromTensorflow(model_path)
        self.backend = "opencv-tensorflow"

        # 记下输出层的名字，只为 `--dump` 与换来源时的排查。旧路径读的是 OD API 的
        # frozen graph，输出是 detection_boxes / detection_scores /
        # detection_classes / num_detections；那条路已随 TF 1.15 训练链一起退役，
        # 留着只是**为了跟新模型做 A/B 对比**，新模型不要走这里。
        try:
            self.output_names = list(self.net.getUnconnectedOutLayersNames())
        except Exception:  # noqa: BLE001 - 老版本 OpenCV 没有这个 API
            self.output_names = []

    # -- 前向 ---------------------------------------------------------------
    def _forward(self, image_bgr):
        cv2 = self.cv2
        # scalefactor=255 是刻意的：blobFromImage 会先把像素除以 255，而这个
        # 图内部自带预处理（uint8 -> 2*(x/255-0.5)），它要的是 0..255 的输入。
        # 用默认的 1.0 会喂进去一个 0..1 的图，模型不报错，只是"偶尔能认"。
        blob = cv2.dnn.blobFromImage(
            image_bgr, scalefactor=255.0, size=(self.input_size, self.input_size),
            mean=(0, 0, 0), swapRB=self.swap_rb, crop=False)
        self.net.setInput(blob)
        if self.output_names:
            outs = self.net.forward(self.output_names)
        else:
            outs = [self.net.forward()]
        return outs

    @staticmethod
    def _table(output_names, outs):
        """把 forward 的结果统一成 {层名: ndarray}。

        OpenCV 有时返回 list（与 output_names 对齐），有时返回 dict；
        两种都见过，所以这里都认。
        """
        if isinstance(outs, dict):
            return outs
        if output_names and len(output_names) == len(outs):
            return dict(zip(output_names, outs))
        return {"out_%d" % i: arr for i, arr in enumerate(outs)}

    @staticmethod
    def _pick(table, *keywords):
        for name, arr in table.items():
            low = name.lower()
            if all(k in low for k in keywords):
                return arr
        return None

    def _decode(self, table):
        """-> [(class_id, score, (xmin, ymin, xmax, ymax))]，坐标为归一化值。

        两种输出形态都处理：
          ① 四个独立张量（我们自己冻的图就是这样）——
             detection_boxes [1,N,4]、detection_scores [1,N]、
             detection_classes [1,N]。坐标顺序 [ymin,xmin,ymax,xmax]。
          ② 单个合并张量 [1,1,N,7]（老式导出）——
             [batch, class, score, x1, y1, x2, y2]，坐标顺序 [x1,y1,x2,y2]。
        两种顺序不一样，混了就会把长宽互换，所以分开写、不共享代码。
        """
        boxes = self._pick(table, "box")
        scores = self._pick(table, "score")
        classes = self._pick(table, "class")

        found = []
        if boxes is not None and scores is not None:
            boxes = np.asarray(boxes).reshape(-1, 4)
            scores = np.asarray(scores).reshape(-1)
            if classes is None:
                classes = np.ones_like(scores)
            classes = np.asarray(classes).reshape(-1)
            for box, score, cls in zip(boxes, scores, classes):
                # 顺序 [ymin, xmin, ymax, xmax] —— y 在前。
                ymin, xmin, ymax, xmax = (float(box[0]), float(box[1]),
                                          float(box[2]), float(box[3]))
                found.append((int(cls), float(score),
                              (xmin, ymin, xmax, ymax)))
            return found

        # ② 合并张量
        for arr in table.values():
            arr = np.asarray(arr)
            if arr.ndim != 4 or arr.shape[-1] < 7:
                continue
            for row in arr.reshape(-1, arr.shape[-1]):
                cls = int(row[1])
                score = float(row[2])
                x1, y1, x2, y2 = (float(row[3]), float(row[4]),
                                  float(row[5]), float(row[6]))
                found.append((cls, score, (x1, y1, x2, y2)))
            break
        return found

    # -- 对外 ---------------------------------------------------------------
    def detect(self, image_bgr):
        """-> [dict(class_name, zone, confidence, xmin, ymin, xmax, ymax)]。

        已按 score_threshold 过滤、按置信度降序、截到 max_detections。
        """
        table = self._table(self.output_names, self._forward(image_bgr))
        rows = self._decode(table)

        out = []
        for class_id, score, (xmin, ymin, xmax, ymax) in rows:
            if score < self.score_threshold:
                continue
            if class_id <= 0 or class_id >= len(self.labels):
                # 0 是 background；越界说明 label 文件和模型对不上 —— 报出来，
                # 别静默丢，静默丢的表现是"某个类永远检不到"。
                continue
            name = self.labels[class_id]
            out.append({
                "class_name": name,
                "zone": zone_of(name),
                "confidence": score,
                "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,
            })
        out.sort(key=lambda d: d["confidence"], reverse=True)
        return out[:self.max_detections]

    def dump_outputs(self, image_bgr):
        """打印输出层名字与形状。换模型来源后第一件要看的东西。"""
        outs = self._forward(image_bgr)
        table = self._table(self.output_names, outs)
        print("输出层 (%d 个):" % len(table))
        for name in sorted(table):
            arr = np.asarray(table[name])
            print("  %-40s shape=%s dtype=%s"
                  % (name, arr.shape, arr.dtype))


class OnnxSsdDetector(object):
    """读 detectNet 契约的 .onnx —— 与 Jetson 上跑的是**同一个文件**。

    为什么仿真侧不另存一份模型：Jetson 走 jetson-inference 的 detectNet，
    它读的图有三条硬约定（输入名 `input_0`、输出 0=scores / 1=boxes、
    类数含背景）。仿真侧既然读同一个文件，就也得按这三条解，否则
    "仿真里过的那套权重"与"飞的时候用的那套权重"变成两件事。
    守这三条的是 tools/ssd_training/check_onnx_contract.py，拷上机前先跑它。

    这一段的下标运算与 detectNet 的 postProcessSSD_ONNX **逐行对应**，因为
    那个函数里没有 anchor 枚举、没有 sigmoid、也没有 softmax：

        mNumClasses = scores 的最后一维          （不是行 0，是最后一维）
        numBoxes    = boxes 的第一维
        numCoord    = boxes 的最后一维，必须是 4
        for m in 1 .. mNumClasses-1:            （class 0 是 BACKGROUND，跳过）
            score = scores[n * mNumClasses + m]
        Left = boxes[n*4+0] ... Bottom = boxes[n*4+3]   （归一化，左上右下）

    预处理同理对应 preProcess() 的 ONNX 分支：cudaTensorNormMeanRGB，
    即 **RGB** 通道序、`2*p/255 - 1`。blobFromImage 算的是
    `(p - mean) * scalefactor`，所以 mean=127.5、scalefactor=1/127.5。

    与 Jetson 侧唯一的实质差别：那边 NMS 是 detectNet 的 clusterDetections
    （`--clustering`，默认 0.75 重叠面积），这边用 cv2.dnn.NMSBoxes 的 IoU
    口径，`--nms-iou` 默认给 0.45。两边不完全等价，但都只影响"同一个物体
    出几个框"，而任务侧本来就只取一个框并做稳定帧判定。

    推理后端两套，onnxruntime 优先（见 __init__ 里的注释）。两者对同一张图
    的输出逐元素等价（scores max|d| 3e-6、boxes max|d| 4e-7），所以选哪个
    不影响检出结果，只影响能不能把这个 onnx 读进来。
    """

    def __init__(self, model_path, label_file, input_size=300,
                 score_threshold=0.5, max_detections=100, nms_iou=0.45):
        import cv2  # 与 SsdDetector 一样：离线跑 --help 不需要 cv2

        self.cv2 = cv2
        self.labels = read_labels(label_file)
        self.input_size = int(input_size)
        self.score_threshold = float(score_threshold)
        self.max_detections = int(max_detections)
        self.nms_iou = float(nms_iou)

        if not os.path.isfile(model_path):
            raise IOError("找不到模型: %s" % model_path)

        # 后端选择。为什么不是无条件用 cv2.dnn：
        # 这个 onnx 是 opset 17 导出的，里面四个 Slice 用的是 opset-10 之后
        # 的张量输入形式（starts/ends/axes/steps 是输入、不是属性）。OpenCV
        # 4.2 —— Noetic 自带的那版 —— 的 ONNX importer 只认 opset-1 的属性
        # 形式，读到这里就抛
        #   Required argument "starts" not found into dictionary
        # 模型本身没有问题（cv2 5.x 与 onnxruntime 都读得动、结果一致），
        # 纯粹是 4.2 的 importer 落后于导出器。onnxruntime 没有这个限制，
        # 并且装它不需要 CUDA，所以先试它（装了就走，没装退回 cv2）。
        self.backend = "opencv-dnn"
        self.session = None
        self.net = None
        self.input_name = ""
        self.output_names = []

        ort_error = None
        try:
            import onnxruntime as ort
        except ImportError:
            ort = None
        if ort is not None:
            try:
                self.session = ort.InferenceSession(
                    model_path, providers=["CPUExecutionProvider"])
                self.input_name = self.session.get_inputs()[0].name
                self.output_names = [o.name for o in self.session.get_outputs()]
                self.backend = "onnxruntime"
            except Exception as error:  # noqa: BLE001
                # 装是装了但起不来（缺 libgomp、模型损坏、架构不匹配）。
                # 记下来退回 cv2 —— 它若是同样读不动，main() 会把两条错误
                # 一起打出来，比只报其中一条好定位。
                ort_error = error
                self.session = None

        if self.session is None:
            self.net = cv2.dnn.readNetFromONNX(model_path)

            # 记下输出层名字只为两件事：打印出来给人看，以及在 --dump 里核对
            # **顺序**。真正取数据按下标 0/1，因为 detectNet 的绑定也是按图里的
            # 输出顺序来的（--output-cvg 绑 0、--output-bbox 绑 1）。名字对但
            # 顺序换了的图，check_onnx_contract.py 会在上机前报出来。
            try:
                self.output_names = list(self.net.getUnconnectedOutLayersNames())
            except Exception:  # noqa: BLE001 - 老版本 OpenCV 没有这个 API
                self.output_names = []
            if ort_error is not None:
                print("[ssd_detection] onnxruntime 起不来，退回 cv2.dnn（原因: %s）"
                      % ort_error, file=sys.stderr)
        if self.output_names and len(self.output_names) != 2:
            print("[ssd_detection] 警告: %s 有 %d 个输出，detectNet 契约要求正好 2 个"
                  "（scores、boxes）；这里按下标取前两个。"
                  % (os.path.basename(model_path), len(self.output_names)),
                  file=sys.stderr)

    # -- 前向 ---------------------------------------------------------------
    def _forward(self, image_bgr):
        cv2 = self.cv2
        # mean=127.5、scalefactor=1/127.5 ⇒ (p-127.5)/127.5 = 2p/255 - 1，
        # swapRB=True ⇒ 通道序是 RGB。size 给正方形 ⇒ 非等比拉伸，与训练一致。
        blob = cv2.dnn.blobFromImage(
            image_bgr, scalefactor=1.0 / 127.5,
            size=(self.input_size, self.input_size),
            mean=(127.5, 127.5, 127.5), swapRB=True, crop=False)

        # 两个输出必须一起取，不能只取第一个：
        #   * onnxruntime：run(None) 本来就返回全部输出；
        #   * cv2.dnn：forward() 不带参数只返回**第一个**输出（曾让 detect() 报
        #     "只返回了 1 个输出"）。
        # 之后按名字绑 scores/boxes（契约固定了这两个名字），名字不齐时按层
        # 顺序兜底（与 detectNet 的顺序绑定一致：0=scores、1=boxes）。
        if self.session is not None:
            outs = [np.asarray(a) for a in
                    self.session.run(None, {self.input_name: blob})]
        else:
            self.net.setInput(blob)
            if self.output_names:
                outs = [np.asarray(a) for a in
                        self.net.forward(self.output_names)]
            else:
                outs = [np.asarray(self.net.forward())]
        if len(outs) >= 2:
            named = dict(zip(self.output_names, outs))
            if "scores" in named and "boxes" in named:
                return [named["scores"], named["boxes"]]
            return outs[:2]
        return outs

    # -- 对外 ---------------------------------------------------------------
    def detect(self, image_bgr):
        """-> [dict(class_name, zone, confidence, xmin, ymin, xmax, ymax)]。

        与 SsdDetector.detect() 同签名同返回结构，所以 ROS 那一侧不需要知道
        载入的是 .pb 还是 .onnx。
        """
        cv2 = self.cv2
        outs = self._forward(image_bgr)
        if len(outs) < 2:
            raise RuntimeError("模型只返回了 %d 个输出，detectNet 契约要求 2 个"
                               "（scores、boxes）" % len(outs))

        scores_arr = np.asarray(outs[0])
        boxes_arr = np.asarray(outs[1])
        if scores_arr.ndim < 2 or boxes_arr.ndim < 2:
            raise RuntimeError("scores 形状 %s / boxes 形状 %s 与 [1,N,C]、[1,N,4] "
                               "不符" % (scores_arr.shape, boxes_arr.shape))

        num_boxes = int(scores_arr.shape[-2])
        num_classes = int(scores_arr.shape[-1])
        scores_flat = scores_arr.reshape(-1)
        boxes_flat = boxes_arr.reshape(-1)

        if boxes_flat.size < num_boxes * 4:
            raise RuntimeError("boxes 只有 %d 个数，%d 个框至少要 %d 个"
                               % (boxes_flat.size, num_boxes, num_boxes * 4))

        # 类数来自张量，不来自 label 文件 —— 与 detectNet 一致。两者对不上时
        # 越界 id 会被 GetClassDesc() 判成 "void" 并逐条丢弃（Jetson 侧），
        # 这里显式报出来，因为本机至少还能看见发生了什么。
        if num_classes != len(self.labels):
            print("[ssd_detection] 警告: 模型 scores 的类维是 %d，而 label 文件有 "
                  "%d 行。两者不一致 ⇒ 类别名会整体错位（Jetson 侧则表现为永远零"
                  "检出且不报错）。跑 tools/ssd_training/check_onnx_contract.py。"
                  % (num_classes, len(self.labels)), file=sys.stderr)

        found = []
        for n in range(num_boxes):
            best_class = -1
            best_score = -1.0
            # class 0 是 BACKGROUND，detectNet 从 1 开始；这里照抄。
            for m in range(1, num_classes):
                score = float(scores_flat[n * num_classes + m])
                if score < self.score_threshold:
                    continue
                if score > best_score:
                    best_score = score
                    best_class = m
            if best_class < 0:
                continue
            if best_class >= len(self.labels):
                continue
            base = n * 4
            left, top = float(boxes_flat[base]), float(boxes_flat[base + 1])
            right, bottom = float(boxes_flat[base + 2]), float(boxes_flat[base + 3])
            # boxes 不做 [0,1] 裁剪（贴边卡片会探出画面），而消息契约是归一化
            # 0..1，下游直接拿它算中心 —— 与 Jetson 侧同一处裁剪。
            left, top = min(max(left, 0.0), 1.0), min(max(top, 0.0), 1.0)
            right, bottom = min(max(right, 0.0), 1.0), min(max(bottom, 0.0), 1.0)
            if right <= left or bottom <= top:
                continue
            name = self.labels[best_class]
            found.append({
                "class_name": name,
                "zone": zone_of(name),
                "confidence": best_score,
                "xmin": left, "ymin": top, "xmax": right, "ymax": bottom,
            })

        found = self._nms(found, image_bgr.shape[1], image_bgr.shape[0])
        found.sort(key=lambda d: d["confidence"], reverse=True)
        return found[:self.max_detections]

    def _nms(self, detections, width, height):
        """按类做 NMS。detectNet 是跨类聚类的，这里按类分开做，结果几乎一样。"""
        if len(detections) <= 1:
            return detections
        by_class = {}
        for item in detections:
            by_class.setdefault(item["class_name"], []).append(item)
        kept = []
        for items in by_class.values():
            rects = [[int(item["xmin"] * width), int(item["ymin"] * height),
                      int((item["xmax"] - item["xmin"]) * width),
                      int((item["ymax"] - item["ymin"]) * height)]
                     for item in items]
            scores = [float(item["confidence"]) for item in items]
            picked = self.cv2.dnn.NMSBoxes(rects, scores, self.score_threshold,
                                           self.nms_iou)
            if picked is None:
                continue
            for index in np.array(picked).reshape(-1):
                kept.append(items[int(index)])
        return kept

    def dump_outputs(self, image_bgr):
        """打印输出层名字与形状。换模型来源后第一件要看的东西。"""
        outs = self._forward(image_bgr)
        print("输出层 (%d 个，按 detectNet 的绑定顺序):" % len(outs))
        for index, arr in enumerate(outs):
            name = self.output_names[index] if index < len(self.output_names) \
                else "out_%d" % index
            arr = np.asarray(arr)
            print("  %-40s shape=%s dtype=%s" % (name, arr.shape, arr.dtype))
        if len(outs) >= 2:
            scores_arr = np.asarray(outs[0])
            print("  => mNumClasses = %d（scores 最后一维），numBoxes = %d"
                  % (scores_arr.shape[-1], scores_arr.shape[-2]))
            print("     labels.txt 有 %d 行。两者必须相等。" % len(self.labels))


def make_detector(args):
    """按模型后缀选后端。对外接口一致，所以调用方不需要分支。"""
    if os.path.splitext(args.model)[1].lower() == ".onnx":
        return OnnxSsdDetector(
            model_path=args.model, label_file=args.labels,
            input_size=args.input_size,
            score_threshold=args.score_threshold,
            max_detections=args.max_detections,
            nms_iou=getattr(args, "nms_iou", 0.45))
    return SsdDetector(
        model_path=args.model, label_file=args.labels, config_path=args.config,
        input_size=args.input_size, swap_rb=args.swap_rb,
        score_threshold=args.score_threshold,
        max_detections=args.max_detections)


# ---------------------------------------------------------------------------
# 画框（离线预览 / 调试图共用）
# ---------------------------------------------------------------------------
def draw(image_bgr, detections, cv2):
    """红=危险品，蓝=人员。与 MissionVision 里的配色一致，看惯了不会认错。"""
    height, width = image_bgr.shape[:2]
    for det in detections:
        x1 = int(round(det["xmin"] * width))
        y1 = int(round(det["ymin"] * height))
        x2 = int(round(det["xmax"] * width))
        y2 = int(round(det["ymax"] * height))
        color = (255, 0, 0) if det["zone"] == "person" else (0, 0, 255)
        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), color, 3)
        label = "%s %.2f" % (det["class_name"], det["confidence"])
        cv2.putText(image_bgr, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return image_bgr


# ---------------------------------------------------------------------------
# 离线模式
# ---------------------------------------------------------------------------
def run_offline(args, detector):
    cv2 = detector.cv2

    if args.image:
        paths = [args.image]
    else:
        paths = sorted(p for p in _glob.glob(os.path.join(args.dir, "*"))
                       if os.path.splitext(p)[1].lower()
                       in (".jpg", ".jpeg", ".png", ".bmp"))

    if not paths:
        print("没有找到图片", file=sys.stderr)
        return 2

    total = 0
    if args.dump:
        image = cv2.imread(paths[0])
        if image is None:
            print("读不了图片: %s" % paths[0], file=sys.stderr)
            return 2
        print("图片: %s  %dx%d" % (paths[0], image.shape[1], image.shape[0]))
        detector.dump_outputs(image)
        print("")
        # dump 之后继续走正常流程，把框也打出来，一次看完

    for path in paths:
        image = cv2.imread(path)
        if image is None:
            print("读不了图片: %s" % path, file=sys.stderr)
            continue
        detections = detector.detect(image)
        total += len(detections)
        print("%-32s %d 个框" % (os.path.basename(path), len(detections)))
        for det in detections:
            print("    %-10s %s  conf=%.3f  [%.3f,%.3f,%.3f,%.3f]"
                  % (det["zone"], det["class_name"], det["confidence"],
                     det["xmin"], det["ymin"], det["xmax"], det["ymax"]))
        if args.out:
            if len(paths) > 1:
                base, ext = os.path.splitext(args.out)
                target = "%s_%s%s" % (base, os.path.splitext(os.path.basename(path))[0], ext)
            else:
                target = args.out
            cv2.imwrite(target, draw(image, detections, cv2))

    print("")
    print("共 %d 张图，%d 个框" % (len(paths), total))
    if args.out:
        print("预览已写 %s*" % os.path.splitext(args.out)[0])
    return 0


# ---------------------------------------------------------------------------
# ROS 模式
# ---------------------------------------------------------------------------
def run_ros(args, detector):
    import rospy
    import cv_bridge
    from sensor_msgs.msg import Image, CompressedImage
    from ssd_detection.msg import VisionDetection, VisionDetectionArray

    # init_node 已经在 main() 里调过了 —— 这里不能再调一次。
    bridge = cv_bridge.CvBridge()

    private = rospy.get_param
    input_topic = private("~input_image_topic", "/camera/image_raw")
    output_topic = private("~output_topic", "/ssd_detection/detections")
    # 唯一的回传画面：画了框的检测结果（红=危险品，蓝=人员）。ssd 后端下
    # 整条任务链里只有这一路图像话题 —— 没有 raw 话题，也没有第二路调试图。
    # 与 C++ 节点的同名参数同默认值，Foxglove 面板在仿真与实机上选同一个话题。
    publish_image = bool(private("~publish_image", True))
    image_topic = private("~image_topic", "/ssd_detection/image/compressed")
    jpeg_quality = int(private("~jpeg_quality", 70))
    # 出图前把长边压到这个宽度（0 = 原尺寸）。检测始终在原图上跑。
    video_width = int(private("~video_width", 640))
    # 推理限速（Hz），0 = 不限制。与 C++ 节点的同名参数同语义。
    max_rate = float(private("~max_rate", 0.0))
    last_infer = [None]

    publisher = rospy.Publisher(output_topic, VisionDetectionArray, queue_size=1)
    image_publisher = (rospy.Publisher(image_topic, CompressedImage, queue_size=1)
                       if publish_image else None)

    cv2 = detector.cv2

    def encode_compressed(image):
        if video_width > 0 and image.shape[1] > video_width:
            scale = video_width / float(image.shape[1])
            image = cv2.resize(
                image, (video_width, int(round(image.shape[0] * scale))))
        ok, buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY),
                            max(20, min(95, jpeg_quality))])
        if not ok:
            return None
        message = CompressedImage()
        message.header.stamp = rospy.Time.now()
        message.format = "jpeg"
        message.data = buffer.tobytes()
        return message

    def callback(message):
        # 限速：丢掉超额帧。queue_size=1 只是不积压，这一层是"算得比看得快"时
        # 主动降频。与 C++ 节点的 max_rate 同语义。
        if max_rate > 0.0:
            now = rospy.Time.now()
            if last_infer[0] is not None and \
                    (now - last_infer[0]).to_sec() < 1.0 / max_rate:
                return
            last_infer[0] = now

        try:
            image = bridge.imgmsg_to_cv2(message, "bgr8")
        except cv_bridge.CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "cv_bridge 转换失败: %s" % error)
            return

        detections = detector.detect(image)

        array = VisionDetectionArray()
        array.header = message.header
        # 帧尺寸随消息走：任务侧（MissionVision）靠它把归一化框换算成像素，
        # 不能假设它相机参数里的 640x480 就是这一帧的尺寸。与 C++ 节点同契约。
        array.image_width = image.shape[1]
        array.image_height = image.shape[0]
        for det in detections:
            item = VisionDetection()
            item.class_name = det["class_name"]
            item.zone = det["zone"]
            item.confidence = det["confidence"]
            item.xmin = det["xmin"]
            item.ymin = det["ymin"]
            item.xmax = det["xmax"]
            item.ymax = det["ymax"]
            array.detections.append(item)
        publisher.publish(array)

        if image_publisher is not None:
            picture = encode_compressed(draw(image, detections, cv2))
            if picture is not None:
                # 用原帧的时间戳，不是 now()：回传画面上的框属于那一帧。
                picture.header = message.header
                image_publisher.publish(picture)

    rospy.Subscriber(input_topic, Image, callback, queue_size=1, buff_size=2 ** 24)
    rospy.loginfo("ssd_detection: %s -> %s" % (input_topic, output_topic))
    rospy.loginfo("ssd_detection: 推理后端 %s（OpenCV %s）"
                  % (detector.backend, cv2.__version__))
    if image_publisher is not None:
        rospy.loginfo("ssd_detection: 回传画面 %s" % image_topic)
    rospy.spin()
    return 0


# ---------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="",
                        help=".onnx（detectNet 契约，与 Jetson 同一个文件）"
                             "或 frozen_inference_graph.pb（旧路径，保留是为了"
                             "能对比新旧模型）")
    parser.add_argument("--config", default="", help="可选的 .pbtxt（只对 .pb 有意义）")
    parser.add_argument("--labels", default="", help="label 文件（行 0 = background）")
    parser.add_argument("--score-threshold", type=float, default=0.5)
    parser.add_argument("--input-size", type=int, default=300)
    parser.add_argument("--no-swap-rb", dest="swap_rb", action="store_false",
                        help="默认把 BGR 换成 RGB；模型要 RGB，所以一般不用关"
                             "（只对 .pb 有意义，.onnx 恒为 RGB）")
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument("--nms-iou", type=float, default=0.45,
                        help="仅 .onnx：逐类 NMS 的 IoU 阈值（Jetson 侧 detectNet "
                             "用的是 --clustering 的重叠面积口径，默认 0.75；"
                             "两者不等价，但只影响同一物体出几个框）")
    # 离线自检
    parser.add_argument("--image", default="", help="单张图，离线模式")
    parser.add_argument("--dir", default="", help="图片目录，离线模式")
    parser.add_argument("--out", default="", help="把画了框的图写到这里")
    parser.add_argument("--dump", action="store_true",
                        help="打印输出层名字与形状（换模型后先看这个）")
    return parser


def main():
    args = build_parser().parse_args(clean_argv())
    offline = bool(args.image or args.dir or args.dump)

    if not offline:
        # ROS 模式：模型路径等优先取节点私有参数，命令行给了就以命令行覆盖
        import rospy
        rospy.init_node("ssd_detection", anonymous=False)
        args.model = args.model or rospy.get_param("~model_path", "")
        args.config = args.config or rospy.get_param("~config_path", "")
        args.labels = args.labels or rospy.get_param("~label_file", "")
        args.score_threshold = rospy.get_param("~score_threshold", args.score_threshold)
        args.input_size = rospy.get_param("~input_size", args.input_size)
        args.swap_rb = rospy.get_param("~swap_rb", args.swap_rb)
        args.max_detections = rospy.get_param("~max_detections", args.max_detections)
        args.nms_iou = rospy.get_param("~nms_iou", args.nms_iou)

    if not args.model or not args.labels:
        print("必须给 --model 与 --labels（ROS 模式下也可以是 ~model_path / "
              "~label_file 参数）", file=sys.stderr)
        return 2

    try:
        detector = make_detector(args)
    except Exception as error:  # noqa: BLE001
        print("加载模型失败: %s" % error, file=sys.stderr)
        # 这一条几乎只有一种成因，而且报错文本完全指不到点上（说的是
        # "starts" 这个参数名，看起来像模型坏了），所以在失败处直接点名：
        # 训练导出的 onnx 是 opset 17，Slice 用张量输入形式；OpenCV 4.2
        # 的 importer 只认 opset-1 的属性形式。模型没坏，是读的人太老。
        if os.path.splitext(args.model)[1].lower() == ".onnx":
            try:
                import cv2
                cv2_version = cv2.__version__
            except Exception:  # noqa: BLE001
                cv2_version = "?"
            print("  提示: 本机 OpenCV %s；若上面的错误是关于 \"starts\"/\"Slice\" "
                  "的，那是 OpenCV<=4.5 读不了 opset>=10 的 Slice，不是模型坏了。"
                  % cv2_version, file=sys.stderr)
            print("        装 onnxruntime 即走它（优先于 cv2.dnn）: "
                  "pip install onnxruntime", file=sys.stderr)
        return 2

    if offline:
        return run_offline(args, detector)
    return run_ros(args, detector)


def clean_argv():
    """去掉 roslaunch 注入的 __name:=__log:= 之类的重映射参数。

    rospy.myargv 只在装了 rospy 的地方才有，而离线自检恰恰可能在一个没有
    ROS 的机器上跑，所以 import 失败就退回原样。
    """
    try:
        import rospy
    except ImportError:
        return sys.argv[1:]
    return rospy.myargv(argv=sys.argv)[1:]


if __name__ == "__main__":
    sys.exit(main())
