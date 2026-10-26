# Lichtblick 本地教学验证记录

SHELL 于 2026-10-27 凌晨授权实机连接、Computer Use 操作和图文教学，工程归 2026-10-26。教程见 [详细教学](../../监看/Lichtblick连接与常用功能教学_2026-10-26.md)，布局见 [正式四面板布局](../../监看/RK_DWA_常用监看_教学版.json)。

## 已实际完成

- Windows USB 网卡最初为以太网 4、192.168.77.2/24；RK USB 为 192.168.77.1。通过已配置别名 `rk-uav` 登录，板端已有 foxglove_bridge 0.6.3。
- Computer Use 启动现有 Lichtblick 1.29.1，选择 Foxglove WebSocket 并实际连接 `ws://192.168.77.1:8765`，收到 MAVROS state、真实相机 JPEG、地图／TF／激光。静止地面显示不属于实飞或建图质量验收。
- 原 image_transport compressed 插件不在当前 RK 上，首次 JPEG 转发失败。改用板上已有 cv_bridge／OpenCV，脚本 `src/location/scripts/monitor_jpeg.py` 将现有 `/detect_result` 压成 JPEG，保留原图 header，不伪造时间。未下载／安装压缩插件。
- 模型单独使用原四类 RKNN，未训练、未改变模型／标签字节；地面入口令 detectnet 为 required，模型节点退出会结束该入口。未运行飞行任务、move_base、location_node 或舵机。
- 运行隔离 Master 11333 的 [约 12 秒只读观测](只读观测.json)：地图 13 条、激光 121 条、IMU 606 条、飞控状态 13 条、JPEG 47 条。原始工具先写 JSON 再打印摘要，期间订阅仍可更新，因此摘要末尾多 1 条激光和 2 条 IMU；本记录只采用已落盘 JSON 的数量，不把它当精确同步采样边界。状态均 connected=true、armed=false、MANUAL；观测列表内 setpoint／GPIO／vision_pose／目标消息均为零。它只覆盖该短窗口及所列话题，不是整轮全过程的零输出审计。
- 桥参数 capabilities、client_topic_whitelist、param_whitelist、service_whitelist 均为 []；topic_whitelist 为精确显示话题。当前只证明参数和只读观测，尚未用恶意客户端做协议拒绝测试。
- GUI 实际导入布局、添加 Plot 并填写 `/mavros/battery.voltage`、Save changes、Export JSON。原 GUI 导出保持在 [四面板原始导出](GUI导出_四面板原始.json)。随后调整面板分割及当前路径名称生成正式布局，并再次通过 GUI 导入；未直接写 AppData 数据库。
- 拍摄连接、导入、地图设置、图像设置、面板菜单、Plot 字段、保存菜单和断线旧值等实际截图，位于 [附件目录](../../../logs/附件/2026-10-26/Lichtblick/)。截图未作生成式修饰。

## 连接中断与当前待续

首次会话 boot_id 为 e9f7713d-9c4b-429f-a1a3-22b9794bf434。USB 消失后 SHELL 按 Esc 中止 Computer Use；助手停止操作。SHELL 随后报告 SSH 已重新连接并明确授权重新使用 Computer Use。恢复后 boot_id 为 60f1e00a-67ae-41a0-a556-4797dabe3951，旧 ROS 进程已不存在；再次部署并开始新的地面入口。

RK 冷启动时日期为 2026-10-24 02:08 左右。两次在启动 ROS 前按本机 UTC 校时，原核验输出保留，未把旧时间改写成历史实际操作时间。

约 00:44:32，第二次会话再次中断；Windows 中 USB 网卡不再出现，USB SSH 及无线候选 SSH 均超时。此后只完成本地布局和教程，没有盲目重启／刷机／调整电源或追查断连根因。已向 SHELL 请求恢复 USB；没有把等待回复视为已经恢复。

**尚未完成：** 第二次断连后的板端进程／句柄释放最终检查、最终版桥小改动的再次机上核验，以及补录用于离线回放的短样本。第二次恢复前已在板端运行的版本不含随后补入的 bridge 模式 JPEG 开关／Navfn 话题小改动；最新本地代码不能冒充已再次部署和验收。未记录到正式四面板同时显示实时数据的最终全景。

无线 192.168.1.102 仅为板端 ip addr 实读到的配置；本轮电脑到该地址的 SSH 尝试超时，未通过无线显示／距离／带宽／延迟验收。地面地图、相机与遥测成功均不代表 TFmini、PCA9685、电池负载、飞控定位融合、真实飞行或投放验收。

## 证据文件

| 文件 | 内容 |
|---|---|
| [01_地面监看运行.txt](01_地面监看运行.txt) | 首次 source ROS 环境触发 nounset，修正后再运行 |
| [02_地面监看运行.txt](02_地面监看运行.txt) | 缺 compressed 插件的原始失败及该次运行输出 |
| [03_地面监看运行.txt](03_地面监看运行.txt) | scripts 目录尚未创建导致节点未找到的失败；随后 USB 断联 |
| [04_只读观测摘要.txt](04_只读观测摘要.txt) | 第二次恢复后 12 秒观测的摘要输出 |
| [只读观测.json](只读观测.json) | 当前已取回的完整观测与 ROS Master 状态 |
| [GUI导出_四面板原始.json](GUI导出_四面板原始.json) | GUI 保存后实际导出的布局 |

失败记录不作为成功证据。实际成功的 JPEG 入口是源码脚本方案，第三次运行的完整板端日志尚未取回。
