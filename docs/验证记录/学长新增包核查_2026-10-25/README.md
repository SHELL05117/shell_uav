# 学长新增包只读核查｜2026-10-25

[评估与合并建议](../../项目资料/学长新增NPU与ROS包评估_2026-10-25.md)｜[当天日志](../../../logs/2026-10-25_装配后功能复检.md)｜[任务状态](../../项目资料/任务状态.md)

SHELL要求优先完成当天任务4。电脑开始核查时间为2026-10-25 23:32（Asia/Shanghai），工程归10月25日；原件只读属性及字节保持。只做隔离提取、逐文件比较和静态模型／脚本核验，没有运行包内install.sh、上电二进制、ROS入口或模型推理，没有SSH、训练、构建或覆盖src。

| 原件 | 字节数 | SHA256 |
|---|---:|---|
| temp1025/npu-fix-1.7.5.gz | 18736218 | ce8bd37fff5c165f7a91d45842363cc3a79099c48ca9d20fda4bd5b8a8d358f4 |
| temp1025/pack.7z | 28707834 | 2558ec3c8ed58e3be16d6cfb5d9e4fb351561ef5551ef2ac2ea063abd4ef0fc0 |

原件核查前后哈希一致。工具先验证成员路径、重复名称和类型，只接受普通文件／目录；不恢复链接，不执行压缩包中的代码。提取到temp1025/inspection_20261025，不进入src，重复执行默认拒绝覆盖已有检查目录。

## 1. 已保存证据

- [完整机器清单与比较](archive-audit.json)：NPU包23成员／16文件，15条内置SHA256全过；ROS包180成员／140文件，31相同＋28仅换行＋63改变＋18新增，本地对应四包另有43个未随包提供文件。
- [NPU成员目录](npu-fix-1.7.5.gz.members.txt)、[ROS成员目录](pack.7z.members.txt)、[执行摘要](01_目录提取与哈希核验.txt)、[源码差异](source-differences.patch)。差异用于审阅，**未应用到工作源码**，并非可直接部署补丁。
- [模型与ELF结构](model-and-binary-contracts.json)、[执行输出](02_模型与ELF静态核验.txt)：ONNX输入1×3×416×416，三个5维raw heads；四个ELF均AArch64。读取使用既有项目外CPU核验依赖C:/Users/alexh/AppData/Local/Temp/uav-yolo-acceptance-runtime，未安装依赖或推理。
- [与锁定NPU参考比较及ONNX结构检查](reference-comparison.json)、[输出](03_NPU参考比对与ONNX结构检查.txt)：四核心固件、代理、API库相同；ONNX checker通过，三个输出均来自Transpose→Reshape→Conv→Relu链，不代表检测后处理或精度通过。
- [现有仿真入口参数对照](04_仿真入口参数兼容性.txt)：两入口均未传入新包未声明参数；parameter.txt与参考只存在换行差异。
- [21份关键原文字节清单](critical-source-manifest.json)，对应原文位于本目录原始关键文本/，均按原字节复制核对，不替换原归档；未收入固件、ELF、ONNX或RKNN大文件。评估中引用的文件行号以这些原文为准。

## 2. 关键原文入口

NPU：[安装器](原始关键文本/npu-fix-1.7.5/install.sh)、[上电／升级脚本](原始关键文本/npu-fix-1.7.5/bin/npu_upgrade)、[启动](原始关键文本/npu-fix-1.7.5/sbin/rknn-start)、[停止](原始关键文本/npu-fix-1.7.5/sbin/rknn-stop)、[rknn.service](原始关键文本/npu-fix-1.7.5/systemd/rknn.service)。

ROS：[导航构建声明](原始关键文本/pack/src/navigation_2d/CMakeLists.txt)、[识别实现](原始关键文本/pack/src/navigation_2d/src/detect.cpp)、[识别入口](原始关键文本/pack/src/navigation_2d/launch/detect.launch)、[任务入口](原始关键文本/pack/src/navigation_2d/launch/navigation_2d.launch)、[任务实现](原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)、[定位实现](原始关键文本/pack/src/location/src/location_with_ftmini.cpp)、[雷达SDK版本](原始关键文本/pack/src/rplidar_ros/sdk/include/sl_lidar.h)、[舵机调试前端](原始关键文本/pack/src/servo_control/script/servo_debug.py)。

## 3. 复现与范围

[审计工具](../../../tools/inspect_1025_packages.py)使用Python标准库与Windows tar，输入参数为项目绝对目录和证据绝对目录；提取路径需事先不存在，避免覆盖已保存的检查现场。JSON记录输入哈希、全部文件哈希及比较状态。模型结构另用既有onnx1.17.0的load／checker读取，ELF仅读头部与版本字符串，未调用模型或二进制。

NPU参考副本为本地锁定官方仓库yolov5training/rknn/npu/RK3399Pro_npu，先前已部署版本及提交见[NPU实板报告](../RK_NPU实板上电_2026-10-24/README.md)与[本日自启修复](../NPU自启修复与相机复查_2026-10-25/README.md)。本轮没有把“和参考相同”扩大成对压缩包其他二进制或整套服务的运行验收。

本轮仅增加离线检查工具、证据、评估及日志；.gitignore只追加/temp1025/暂存范围，不忽略docs／logs。旧SDK、模型与包内注释作为收到的原件事实保留，不覆盖当前后续证据。没有源码合并、硬件或部署结果。
