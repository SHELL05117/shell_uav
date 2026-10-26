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

## 连接中断与当时待续（已由下节接续）

首次会话 boot_id 为 e9f7713d-9c4b-429f-a1a3-22b9794bf434。USB 消失后 SHELL 按 Esc 中止 Computer Use；助手停止操作。SHELL 随后报告 SSH 已重新连接并明确授权重新使用 Computer Use。恢复后 boot_id 为 60f1e00a-67ae-41a0-a556-4797dabe3951，旧 ROS 进程已不存在；再次部署并开始新的地面入口。

RK 冷启动时日期为 2026-10-24 02:08 左右。两次在启动 ROS 前按本机 UTC 校时，原核验输出保留，未把旧时间改写成历史实际操作时间。

约 00:44:32，第二次会话再次中断；Windows 中 USB 网卡不再出现，USB SSH 及无线候选 SSH 均超时。此后只完成本地布局和教程，没有盲目重启／刷机／调整电源或追查断连根因。已向 SHELL 请求恢复 USB；没有把等待回复视为已经恢复。

**00:44之后的历史状态：** 第二次断连后的板端进程／句柄释放最终检查、最终版桥小改动的再次机上核验，以及补录用于离线回放的短样本当时尚未完成。此前已运行的版本不含随后补入的 bridge 模式 JPEG 开关／Navfn 话题小改动；当时的本地代码不能冒充已再次部署和验收。最终完成情况见下节。

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

## 01:30恢复后的最终收尾

**SHELL授权：** 回复“已恢复”，并要求收尾及交付完成后执行 `sudo shutdown -h now`。本次新 boot_id 为 269deb4b-c978-4d91-9563-66dcb46be37c；启动 ROS 前按电脑 UTC 校时。最新桥、脚本、安装／依赖声明及 detect launch 已同步；[最终部署与释放核验](最终部署与释放核验.json)确认10份相关文件两端 SHA256 一致。

**真实显示与观测：** 正式四面板同时显示地图、相机、飞控状态与电压 Plot，保存[实时全景](../../../logs/附件/2026-10-26/Lichtblick/10_最终实时地图相机状态与电压.png)。[最终观测](最终只读观测.json)通过：约12秒map13、scan120、IMU601、state13、JPEG48，状态为MANUAL／connected／未解锁，列出的动作输出为0。工具已增加锁与冻结快照，最终JSON与打印摘要的计数一致。此次[WebSocket握手](最终WebSocket握手.json)实读capabilities=[]，10个实际显示channel；未进行发送飞行命令的拒绝测试。

**样本／回放：** 录制17.9409647秒、8006616字节，地图19、激光178、TF2338、静态TF1、JPEG70、state19、电池9，见[录制核验](录制核验.json)。电脑原始文件在`recordings/lichtblick_20261026/地面教学样本.bag`，板端原件`/home/jcfh/lichtblick-ground-sample-final.bag`保留，两端SHA256均为`c1cb858819fea65fb87b08fe9f37d5e4f02d5a36d64803dee390335a6141a3b2`。按项目规则原始bag被Git忽略，保留校验记录。GUI已Ctrl+O打开、实际播放到末尾，并点击时间条跳到约8.946秒处。教程新增回放步骤及截图，共12张截图、13节。原样本JPEG偏暗／蓝（[原始首帧](录制首帧.jpg)可见），电压全为0，pose未收到；未把它们当有效图像质量／电池测量／定位验收。

**占用检查修正及集成验证：** 退出时发现RK没有fuser，原guard会把命令缺失误当空闲；已改为sudo只读扫描/proc的`monitor_device_handles.py`，辅助脚本／权限／设备异常均拒绝启动。以普通临时文件验证真实打开句柄能被发现，不为了测试去打开硬件。[7项集成核验](入口与占用集成核验.json)全通过：bridge模式附着隔离Master11335并自动启动JPEG转发、没有硬件业务节点、四类能力为空、重复ground拒绝、真实设备空闲、普通文件占用可检出、缺失路径拒绝。第一次手动bridge复查时有界测试已超时退出，09号错误是该次Master已停止；后续自动化集成核验通过，不把09号错误写成通过证据。

**退出与服务：** 地面入口收到SIGINT正常收尾，[最终运行日志](07_最终地面运行.txt)保留。桥测试仅用自身创建的进程组收尾。最终ps无本轮ROS／相机／雷达／MAVROS／识别／录制进程，/proc检查相机和串口句柄均空，NPU target／power／firmware／proxy与USB SSH五项均active，见[最终释放记录](12_最终释放服务与源码SHA.txt)。11号初查误用了不存在的shell-npu-agent.service，不能把该行inactive当真实proxy停机；实际正式名称shell-npu-proxy.service已重新核实正常。

**关机实际执行：** 完成交付提交9cd127c后，于Windows本地2026-10-27 01:47:55按SHELL授权在RK SSH会话执行`sudo shutdown -h now`。RK先返回01:47:54时间，随后连接reset、SSH退出255；01:48:30再次SSH复查超时、USB网卡已不出现，见[关机执行记录](关机执行记录.json)。这是已下发命令和网络断开的证据，未取得正常退出0回执或物理电源测量，不把SSH255称为命令正常退出0，也不把网络不可达单独当完全断电证明。没有再启动无人机来验证关机。未验边界仍为无线覆盖／延迟、真实高度与融合、舵机／投放／实飞。

说明：[最初本地交付快照](本地交付核验.json)保留第一次交付时的待恢复状态及当时暂存哈希，不覆盖后续成功证据。最终状态以本节及[最终部署与释放核验](最终部署与释放核验.json)为准；09号手动复查的标准错误单独保存在[工具错误摘录](09_bridge模式核验_工具错误摘录.txt)。
