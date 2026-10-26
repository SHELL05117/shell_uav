# 四类416模型部署说明

模型来自SHELL提供的temp1025/pack.7z。best.onnx、best.rknn与labels.txt保留收到时的原字节，部署不涉及重新训练、模型重导出、量化重建或阈值调优；训练工作由学长负责。完整压缩包及隔离副本按本地暂存保留，来源及逐文件哈希见[原包核查](../../../../docs/验证记录/学长新增包核查_2026-10-25/README.md)。

| ID | labels.txt | 含义 |
|---:|---|---|
| 0 | person | 人员 |
| 1 | explosive | 易爆 |
| 2 | flammable | 易燃 |
| 3 | toxic | 有毒 |

不要沿用历史六类模型的类别编号或把其两个干扰类当作本模型输出类别。四类标签不代表干扰图片可忽略。

## 输入与输出

- ONNX逻辑输入为[1,3,416,416]，RGB、float32、0..1；图像按等比例letterbox缩放，补边114。
- RKNN使用现有RK3399Pro 1.7.5 API／NPU服务。应用传入UINT8 RGB、NHWC、pass_through=0；实读模型输入属性为n_dims=4、fmt=0、dims=[416,416,3,1]，这是旧API反向维度顺序。
- 三个逻辑raw heads为[1,3,52,52,9]、[1,3,26,26,9]、[1,3,13,13,9]；9个字段为tx、ty、tw、th、objectness及四个类别logits。RKNN属性分别为[9,H,W,3,1]，n_elems为73008、18252、4563。
- outputs_get(want_float=1)取得浮点结果，数据按原逻辑[N,A,H,W,K]排列。应用转为[N,A,K,H,W]再合并成[1,27,H,W]，不能只reshape而不换轴。此适配在C++完成，模型文件不变。
- 使用包内18个anchors、strides=8,16,32，confidence=0.5、逐类NMS IoU=0.45；RGB通道、归一化、网格−0.5及letterbox反变换保持同一口径。包中没有独立训练／校准过程记录，不将这些缺失材料编造成已验证事实。

## 已有机上证据与限制

[固定12图CPU／RKNN比较](../../../../docs/验证记录/DWA四类部署_2026-10-25/fourclass-cpu-rknn-comparison.json)已取得真实五维张量并确认全为有限值；相同协议下ONNX与RKNN各输出10个检测，8张业务图均有对应类别。原始logits平均绝对差约0.4295、最低相关系数约0.9318，二者数值不完全一致，不能宣称浮点逐元素对拍通过。

**4张干扰图中2张（IMG_2828、IMG_2829）被两条路径识别为flammable。** 这是模型误报限制，不是部署成功就能消除的问题；没有通过降低阈值或排除负样本掩盖。该12图是固定回归样本，不是独立盲测精度验收。

本次NPU API set/run/get单图实测约650–666毫秒；不包含图像预处理、后处理及通信的完整链路。任务因此使用独立推理worker，控制定时器只读取已完成的新sequence，不能同步等待NPU。相机回传／标注图的发布FPS也不等于真实推理FPS。

检测代码要求相机header为非零、非未来且源图年龄不超过frame_timeout（默认1秒），拒收重复或乱序时间戳。推理保留输入的原始header、接收时已存在的延迟及steady接收时刻；完成后源图总年龄超过inference_timeout（默认3秒）就拒绝该结果，不进入新sequence缓存。健康判断同时核验当前相机、已完成结果的源图年龄和完成年龄；不能用持续到达的新相机图延长旧结果寿命，也不能用较短的1秒相机门限替代3秒结果门限。实际执行过NPU但结果已过期时日志为`completed inference rejected: source stale`，与模型初始化／API故障分别记录。

现有NPU自启服务和雷达2.1.0保持；本目录没有需要另行安装的固件或NPU服务。运行前可在本目录使用`sha256sum -c SHA256SUMS`核对三个部署文件。

## 有界相机检查

[rk_fourclass_camera_check.py](../../../../tools/rk_fourclass_camera_check.py)在已source的Melodic环境使用Python2运行：

```bash
python2 /path/to/rk_fourclass_camera_check.py /path/to/new_output /path/to/IMG_2766.jpg
```

它自建127.0.0.1:11330私有master，按顺序独占资源运行三个case：先将已知易燃图片保持比例缩小到1280×720包围尺寸（不放大、不裁剪、不修改原件），用真实detectnet持续14秒，核对flammable与inference OK日志并保存实际红框标注；再启动usb_cam及真实detectnet观测18秒，统计源图／输出图FPS和尺寸、保存实拍；最后以不存在的模型文件确认detectnet自然退出2。已知图回放用于模拟实际相机尺寸，不是原始24MP图片的吞吐验收；JSON记录原始／发布尺寸、缩放比例和原件SHA256。它不启动MAVROS、任务或舵机，不修改模型、阈值、相机标定或NPU服务。

主要输出为fourclass-camera-results.json、各进程日志、known-image/known-flammable-annotated.jpg，以及real-camera/camera-last.jpg、real-camera/output-last.jpg。真相机case不要求矿泉水瓶被四类模型识别。

脚本只有在三个case、正常进程退出与自建进程组清理均通过时退出0；任一失败退出1。缺模型case的2是被测detectnet的预期状态，不是整个检查脚本的失败码。若用了TERM／KILL升级，报告不会将其当作正常结束。源图≥20Hz和输出图≥5Hz只是图像链检查，推理还需独立看到实际inference OK日志；JSON明确output_hz_is_inference_hz=false。实际是否通过以本轮机上生成的JSON和日志为准，本说明不代替运行证据。

## 联合只读检查

[rk_dwa_readonly_stack_check.py](../../../../tools/rk_dwa_readonly_stack_check.py)与上方相机脚本放在同一目录，用`python2 rk_dwa_readonly_stack_check.py /path/to/new_output`运行。它只使用已装Melodic消息包、cv2/numpy及系统/proc，不增加pip依赖；私有master为127.0.0.1:11333，启动等待上限60秒、正式观测20秒。

检查相机、雷达、只读白名单MAVROS、Cartographer、原180°激光静态TF和明确关闭actuation／auto_start／真实舵机的DWA任务链。定位显式使用未发布数据的测距topic，既不打开I²C又保留真实测距缺失时的禁发门。全程监看飞控未解锁、模式稳定、setpoint／GPIO／vision pose零消息，记录mission/readiness/location状态、真实DWA动态库载入、消息连续性、JPEG、进程资源峰值和系统可用内存。任务标注图按共享launch的publish_scale=0.5核对，不要求与1280×720源图完全同尺寸。

正式20秒窗口至少接收10条readiness，inference_ready=true比例须≥95%；连续false时长、消息最大间隔、窗口首尾消息缺口均不得超过1秒。JSON保存比例及各间隔，证明窗口内检测结果维持源帧／完成时间健康；readiness通常10Hz发送，但同一推理结果可以被多次报告，不能当作10FPS推理。真实inference OK日志仍须至少3条，该计数包含预热，并在所有子进程退出、日志关闭后读取，避免重定向stdout缓冲导致误判。动态库载入不代替实际路径规划或避障验收。

输出为dwa-readonly-stack-results.json、camera-last.jpg、output-last.jpg及各启动进程日志。只读检查及正常清理全部通过退出0，检查失败退出1，启动脚本的预检异常退出2。只清理由本进程创建、或按PID与内核starttime证明属于它的进程组，不使用pkill。SLAM地图只按实收有效栅格如实记录；此工具不开展飞行、舵机动作或有效高度／整机定位验收。
