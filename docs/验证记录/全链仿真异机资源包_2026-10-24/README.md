# 全链仿真异机资源包

实际准备时间2026-10-25凌晨，按SHELL规则归2026-10-24。SHELL要求将另一台已配置Docker／WSL电脑开展全链仿真所需的项目材料打包，本轮只归档和核验，不安装环境、不运行仿真或操作设备。

## 交付

- ZIP：项目外`D:/SHELL/UAV_sim_handoff_1024_2026-10-24.zip`，984,766,117字节，约939.15MiB。
- 外部校验：同目录ZIP文件名加`.sha256`。
- SHA256：`e46431a40ec894a03950d6f04be5c6182b5659c9da0bc6d5c5078fa27a42b720`。
- 基线：`1024`标签对应`2f1853ba9b75f9a912541ec2a8df1d1c767d108f`。原标签不随打包工具／文档提交移动。
- 根目录为`UAV_sim_handoff_1024/`，包含`project/`冻结快照、`handoff/`接续材料、`inputs/`原始数据及清单。

753个原已跟踪文件全部按原字节入包，含8包源码、场景／传感器／控制、模型／配置／导出脚本、技术说明、日志和证据。另附本轮交接材料、完整180张JPG／180份VOC XML原始素材ZIP与BASELINE.json，共766个载荷文件。MANIFEST.json与SHA256SUMS.txt不自包含。

交接说明及可直接发给接收端agent的提示词源文件见[先读说明](../../../tools/sim_handoff/README_先读.md)、[接收端提示词](../../../tools/sim_handoff/另一台电脑执行提示词.md)、[依赖与下载](../../../tools/sim_handoff/环境与外部依赖.md)。Docker基础模板、apt清单、Python CPU候选依赖、Windows／Linux只读检查和包校验脚本均随包提供。打包工具为[package_sim_handoff.py](../../../tools/package_sim_handoff.py)。

## 核验

- 包内766个载荷逐文件SHA256／大小、ZIP成员集合核对通过。
- 独立以Git blob算法复核ZIP内753个project文件，全部与1024原树对象相符。
- Python打包时计算的外部ZIP SHA256与PowerShell Get-FileHash独立结果一致。
- 原始数据ZIP内180 JPG／180 XML数量确认；数据ZIP随包保持原字节，SHA256见[核验结果](打包核验.json)。
- 无训练checkpoint、TensorRT engine、临时固件或嵌套.git；保留ONNX／TorchScript／RKNN导出模型。
- 新Python与PowerShell脚本语法解析通过；未声称Docker模板完成构建或Linux环境执行通过。
- 机器详情、载荷清单见[核验JSON](打包核验.json)、[包内清单](包内清单.json)、[外部校验记录](SHA256SUMS.txt)。

## 实施边界

这不是完整离线镜像。Docker镜像、ROS／Gazebo软件包、PX4 v1.13.3及递归子模块、Cartographer／bridge、PC推理wheel仍由接收电脑联网获取。发送端尝试PX4 git ls-remote时网络超时，未将其写成下载或固定tag解析成功；接收端须锁定实际commit和子模块。

旧仿真入口仍包含SSD节点，接收端须新增YOLO接口；start_px4_slam.sh默认Noetic，选择Melodic时须显式覆盖ROS_DISTRO。原训练完整交付ZIP和checkpoint未入包，独立核验工具的原ZIP全量audit不能直接重跑；图像推理适配可使用模型、anchors、class_map及已纠正的推理实现。现有模型只是试验基线，业务识别未通过。

包内project是原1024快照，不带Git历史；本轮工具／交接材料在handoff另行覆盖最新说明。接收端先盘点及构建，再依次验证SITL控制、传感器／SLAM、YOLO、任务／投放／返航与故障分支。纯软件仿真不等待TFmini／PCA实物接线，不能替代机体复检和真实飞行。
