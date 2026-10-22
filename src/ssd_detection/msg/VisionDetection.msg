# 一个检测框。
#
# 坐标是【归一化】的 0..1，原点在图像左上角，顺序 xmin / ymin / xmax / ymax。
# 归一化是刻意的：检测端和消费端各自拿到的图可能不是同一个分辨率
# （检测端内部会缩到 300x300），用归一化就不需要谁去猜对方那张图多大。
#
# 两个坐标坑都在这里踩过，写下来免得再踩一次：
#   1. 模型原始输出 detection_boxes 是归一化的，不是像素；
#   2. 模型给的顺序是 [ymin, xmin, ymax, xmax]，**y 在前**，
#      按 [xmin, ymin, ...] 读会把长宽互换。
# 本消息统一成 xmin/ymin/xmax/ymax，转换在检测节点里做一次，下游不再需要知道。

string class_name     # 类别名，逐字取自 label 文件（行 0 是 background，不会出现在这里）
string zone           # "hazard" 或 "person"；由类别名推导，任务侧按它筛选
float32 confidence    # 0..1
float32 xmin
float32 ymin
float32 xmax
float32 ymax
