# RK3399Pro NPU恢复与六类YOLO实板验证

[任务状态](../../项目资料/任务状态.md)｜[操作说明](../../项目资料/RK_NPU上电脚本使用说明_2026-10-24.md)｜[日志§15](../../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md#15-shell授权代执行npu上电固件与识别验证)｜[先前训练成果验收](../YOLO训练成果验收_2026-10-24/README.md)

实际操作：2026-10-25凌晨，工程归2026-10-24。SHELL授权代执行上电／固件／识别验证，并提醒本地已有训练成果；本轮使用交回六类RKNN补目标机测试，没有重训或改动权重。板端日期落后，证据中的PC capture为电脑实际时间，保留板端原始时间，不混用。

**结论：NPU已恢复枚举、代理和真实推理；交回六类RKNN在本块板完成36/36张冻结测试图推理，目标机加载与运行通过。业务识别仍未通过；未完成主机重启自启、相机端到端、ROS任务及飞行验收。** 后续收尾SSH出现连接重置／banner超时，不能将推理完成时状态写成收尾时仍在线；详见第5节。

## 1. 实测恢复过程

| 环节 | 证据与实际结果 |
|---|---|
| 首次连接 | [01](01_上电前复核.txt)、[02](02_已有上电复核与加载条件.txt)：已存在本脚本powered归属，六路使能及复位高、24MHz计数1、板内3-1.1为2207:180a。上电早于本轮首次SSH，不能归为本轮助手首次开启；重复on不复位 |
| 固件／工具核对 | [03](03_固件完整性与唯一USB目标.txt)、[04](04_NPU固件装载预检.txt)：四个二进制及代理与官方仓库Git blob一致，只有一个板内启动目标。parameter.txt有CRLF差异而导致03整体非0；它未用于本次装载 |
| 初次Loader失败 | [05](05_NPU固件实际装载.txt)、[07](07_Loader超时诊断.txt)、[08](08_Loader工具底层日志.txt)：db超时30s，随后设备请求失败；未把失败记作完成 |
| 独立NPU重启修正 | [09](09_NPU独立断电上电后Loader重试.txt)、[10](10_关断后低电平再启动回归.txt)：off成功，但sysfs unexport保留输出低，旧预检过严而拒绝on。修正为未占用且锁存／电平低即可，新增off→on回归；本机和RK Python3.6均13项通过，含27处启动写失败回退 |
| Loader与运行系统 | [11](11_Loader就绪后固件运行.txt)只是一次提前退出的预检；[12](12_实际装载uboot_trust_boot并运行.txt)才是实际td、三个wl、rs成功。运行态[13](13_NPU运行态USB与设备列表.txt)为1-1.1、2207:0019、5000M |
| 官方YOLO对照 | [14](14_官方YOLO真实NPU推理.txt)、[16](16_普通账户命令与推理复验.txt)：API／DRV均1.7.5，COCO公交图片检出3人及公交，普通账户100轮平均32.75ms。独立编译[06](06_官方YOLO示例独立构建.txt)已到100%，当次退出1来自末尾ldd命令CR，不是编译失败；后续运行证明可执行 |
| 代理正式位置 | [17](17_代理正式路径与手动服务.txt)：/usr/local/bin/npu_transfer_proxy；shell-npu-proxy.service为active、static，未启用开机启动。代理替换进程后出现模型加载挂起，六类[21](21_六类模型推理执行.txt)90s及官方[22](22_六类加载超时后官方模型对照.txt)30s均超时，不能归因于六类权重 |
| 第二次恢复与对照 | [23](23_正式代理启动前NPU独立重启与官方复验.txt)：停止代理，独立NPU off→on→固件成功，3s后启动测试仍太早而接口打开失败。待就绪后[24](24_固件就绪后官方与六类模型对照.txt)官方100轮平均32.01ms，六类亦成功加载／查询三头。首次图片循环因Windows CRLF清单被安全校验拒绝；修正清单为LF后[25](25_六类模型36张冻结测试图实板推理.txt)36张全部完成，退出0 |

六类首次编译缺-pthread，见[20](20_六类模型真实NPU推理.txt)，随后增加链接选项；不涉及修改SDK。代理切换导致的加载异常只是本次观察，底层原因未完全定位；今后需要独立验证代理重启恢复，不能仅用devices有一行就宣称推理健康。

最终成功设备行（13／16／20中有原始输出）：

```text
List of ntb devices attached
14ade952d732d316    a0096b1e    USB_DEVICE
```

## 2. 固件来源与写入范围

主机为Ubuntu18.04 AArch64、4.4.194，设备树兼容rockchip,rk3399pro-toybrick-prod-linux。固件来自板端/home/jcfh/RK3399Pro_npu，官方[airockchip仓库提交](https://github.com/airockchip/RK3399Pro_npu/tree/8858114555f1700e41e185b14ec3d266342f8059)。USB固件drivers/npu_firmware/npu_fw：

| 文件 | 字节 | SHA256 |
|---|---:|---|
| MiniLoaderAll.bin | 160078 | e554a1510ec462a31242c99b87b5ad83fbccdb570f33db8305cb3d9ff3fa6332 |
| uboot.img | 2097152 | e3246735294a5cae0f0cb3a9d9ad5b16a0663815e202e366220d9cd6c7c3e3ca |
| trust.img | 2097152 | 92d650fe50a9a53e638ebd80d25d793fc298201675ab8cd1370d7593c59a5be2 |
| boot.img | 21183488 | 7cef42d2c487daccd38b9b8b67f6eb3ee40b8d853bf7185a1b30c673a089df29 |

LinuxAArch64 upgrade_tool来自[JeffyCN镜像固定提交](https://github.com/JeffyCN/mirrors/blob/2a532b012b5179dd573d8b7f98fc2c51b3046409/bin/upgrade_tool)，3089960字节，SHA256=09dfa63f4a9f3e38bab3c0ee2731a08e96e0fb52f21d259a028080204621da4c，Git blob=a03e75920b21b80d954d84537b8db73b79f2fb9c。流程依据同提交[厂商npu_upgrade](https://github.com/JeffyCN/mirrors/blob/2a532b012b5179dd573d8b7f98fc2c51b3046409/bin/npu_upgrade)，[原脚本副本](参考_npu_upgrade.sh)保留。

[tools/rk_npu_boot.sh](../../../tools/rk_npu_boot.sh)执行NPU专用db→td→wl 0x20000／0x20800／0x21000→rs的RAM启动流程，锁定工具哈希、仓库提交、固件blob、本次power归属、唯一2207设备、板内USB位置及LocationID=30101。当前NPU运行时拒绝重复启动。不改主CPU的boot_linux、rootfs、eMMC分区、内核或DTB；没有重启主机。

## 3. 使用SHELL自己的六类模型

模型原件位于项目yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/models/sixclass_yolov5s_relu_rk3399pro.rknn，7174523字节，SHA256=f922b48bdd34e0ed47b94a8c3975a6b12b58989c8cf4c5816f66cc3a00b6eab1。板端20输出核对一致，单独放在/home/jcfh/npu_runtime_175/sixclass，未替换src中的旧视觉模型。

- [C++板端探针](../../../tools/rk_sixclass_probe.cpp)使用官方1.7.5头文件／lib64库编译（-std=c++11 -O2 -Wall -Wextra -pthread）；不套用官方80类后处理。
- [PC核验工具](../../../tools/verify_rk_sixclass.py)复用前轮独立RGB预处理与正确解码。36张原冻结test，每类6张；原ZIP图／XML哈希再次逐个核对。OpenCV双线性letterbox到640、灰114、RGB uint8 NHWC，pass_through=0；转换设置mean=0、scale=255，不重复除255。
- 板端输出want_float=1，三头NCHW [1,33,80,80]、[1,33,40,40]、[1,33,20,20]；原API维度顺序W,H,C,N，见[tensors.tsv](tensors.tsv)。输出均有限且在sigmoid范围内，见[范围记录](sixclass_head_ranges.json)。
- 结果取回PC后使用同一anchors、网格−0.5、score=obj×class、逐类NMS0.6还原到原图。mAP使用multi-label conf>0.001；部署式工作点使用best-class conf>0.25。没有针对测试集改阈值／调权重。

| 同36张、同固定640协议 | ONNX CPU（此前） | 本次RKNN NPU |
|---|---:|---:|
| mAP50 | 0.7631 | 0.7465 |
| mAP50–95 | 0.6683 | 0.6447 |
| 工作点存在正确类且IoU≥0.5，人员 | 6/6 | 6/6 |
| 有毒 | 3/6 | 3/6 |
| 易燃 | 2/6 | 4/6 |
| 易爆 | 5/6 | 5/6 |
| 干扰1 | 5/6 | 5/6 |
| 干扰2 | 6/6 | 5/6 |
| 12张干扰图仍有任意业务类框 | 3/12 | 3/12 |

量化mAP50比同口径ONNX低1.664个百分点；不能和原交回PyTorch矩形batch的0.722或训练验证0.90158直接相减。存在正确框允许同图还有错误框；NPU工作点最高分框正确数为[6,2,2,5,3,5]/各6张，所以不能将“易燃4/6存在正确框”理解为4/6张任务决策正确。

36次rknn_inputs_set＋run＋阻塞outputs_get：平均47.575ms、中位47.280ms、最小43.470ms、最大53.809ms，含首帧；见[timings.tsv](timings.tsv)。不含PC预处理、解码、画图、文件写入，不等于相机端到端FPS。

机器证据：[逐图输入哈希](sixclass_input_manifest.json)、[量化评估](sixclass_rknn_evaluation.json)、[工作点预测](sixclass_rknn_bestclass_predictions.json)、[低阈值预测](sixclass_rknn_predictions.json)、[下载输出清单](sixclass_downloaded_outputs_manifest.json)。全部108份float头和两份TSV已取回电脑，原始张量约40MB留在temp1022/npu-runtime-20261024/sixclass/outputs及板端，不批量纳入Git。清单是取回后的本地哈希；收尾SSH失败，未取得独立板端输出哈希，不将其称为两端哈希对拍。

[36图工作点结果图](sixclass_rknn_test_sheet.jpg)：绿框GT，红框预测，文字为类别ID／置信度。0人员、1有毒、2易燃、3易爆、4干扰1、5干扰2。原始官方对照[bus检测图](官方YOLO_bus_检测结果.bmp)另存。

## 4. 复验与当前安装

源代码／使用方式见[操作说明](../../项目资料/RK_NPU上电脚本使用说明_2026-10-24.md)。当前板端安装：

- /home/jcfh/rk_npu_power.py：独立上电控制，最新SHA256=d9f151d62a7a049d53bf68f4c28c47858389379a319ae8ff3f1dfd18904bc2ba；[RK13项回归输出](18_RK_Python36十三项回归.txt)。
- /home/jcfh/npu_runtime_175：upgrade_tool、固件辅助脚本、官方demo构建、sixclass探针／模型／图片／输出。
- /usr/local/bin/npu_transfer_proxy及/etc/systemd/system/shell-npu-proxy.service：手动启动代理，未开机启用。官方仓库原文件未改写。

PC重算已有张量：

```powershell
& 'C:/Users/alexh/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' tools/verify_rk_sixclass.py evaluate --package 'yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24' --source D:/SHELL/TargetDelection1.1.zip --work temp1022/npu-runtime-20261024/sixclass --report 'docs/验证记录/RK_NPU实板上电_2026-10-24' --runtime C:/Users/alexh/AppData/Local/Temp/uav-yolo-acceptance-runtime
```

## 5. 未完成事项与收尾连接异常

[26收尾输出](26_六类推理后最终状态.txt)在36张推理完成、全部张量下载及本地评估后，SSH连接重置；其后多次banner超时，[27最后复查](27_收尾SSH复查.txt)仍失败，PC ping 192.168.77.1为1ms可达。无法继续读取板端进程／资源，未将该现象归因为NPU故障，也未声称长期稳定通过；没有因此重启主机或写入其它硬件。最后一次成功的真实推理证据是25，最后成功查询设备／服务早于该连接异常。

待连接恢复后复核devices和重复推理／资源，进一步定位代理重启恢复、SSH异常；随后分别验证主机重启后的NPU初始化与相机端到端性能。业务模型仍存在有毒／易燃混淆和干扰误报，原训练接收结论维持“首轮试验基线”；ROS接口、任务触发、投放和飞行未操作、未放行。
