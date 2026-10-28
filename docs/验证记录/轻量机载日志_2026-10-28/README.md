# 轻量机载日志与OFFBOARD正常解锁入口

## 当前状态

SHELL于2026-10-28要求补回轻量记录，并查明／解决OFFBOARD切换问题。QGC先OFFBOARD再Takeoff与本次ULog一致：命令22主动让PX4进入AUTO.TAKEOFF，发生在离地之前。该控制路径已查明；振动和定位间断不能据此宣布全部解决，见[事件分析](../起飞摇晃偏航排查_2026-10-28/README.md)。

14:20正常关机供SHELL修复起落架时：**轻量基础文件已安装，`uav logs status`可用；新增`uav arm`仅在本地完成，尚未同步到RK。** 本地5项记录器安全测试和8项解锁门控测试通过；RK5项记录器测试／Python2与Bash语法通过。首轮关闭记录因RANGE_STALE无法稳定就绪，没有进入90s性能测量，不能作为性能通过。隔离录包／解锁服务验证和实板开关性能对照待恢复USB后接续。本文件按最终证据继续更新。

## 实现范围

默认`uav a`／`uav check`创建`~/uav_flight_logs/<时间>-lite-<随机ID>/`，C窗口关联同一轮。可用`UAV_LOG_ROOT`覆盖根目录；`UAV_LOG_DISABLED=1`只用于明确关闭记录或性能对照。`UAV_LOG_PROFILE=full/core`不切换本次轻量入口，昨日完整实现仍在本地历史源码，安装脚本不部署它。

- 独立C++ rosbag记录小白名单：外部vision pose、PX4本地pose／velocity、原始设定值／反馈、state／extended_state／estimator_status／statustext／RC／电池、定位status、任务status／readiness、DWA cmd_vel／目标／结果、GPIO命令／ACK／servo_ready。
- 不录相机／推理源帧、地图、scan或全量TF；不重新编译任务／定位／视觉节点。现有状态可定位“何时断流”，不能保证单靠这些话题区分全部传感器／SLAM内部原因。
- 独立Python监督进程不订阅ROS消息；每5秒保存系统CPU、iowait、可用内存、温度和关键进程CPU／RSS。进程发现每30秒一次，不每秒遍历全部进程IO。真实主控CPU百分比按在线核归一化，进程百分比以一核100%计。
- 每轮保存启动stdout／ROS文本日志、模式和命令参数、一次关键源文件／二进制／模型／标定哈希；参数在启动后及C窗口关联时各取一次。不保存凭据或整个环境。
- 60秒分包、16MiB缓冲、不压缩；启动需3GiB空闲，运行低于约1.1GiB停记录，不删旧日志。记录器异常不停止定位、导航或飞控。SIGINT正常封包，突然断电仍可能损坏最后一段。

日志CLI（基础文件已安装）：

```bash
uav logs
uav logs status
uav logs stop
uav logs report /home/jcfh/uav_flight_logs/某轮目录
```

`stop`只停止记录；`report`在落地解除武装且录制关闭后运行，不在线扫描活动包。报告显示话题计数、接收间隔、ROS头时间年龄与模式事件；“文件正常关闭、未报告写入错误”不表示数据涵盖所有飞行阶段。未出现的设定值／投放话题必须结合当时阶段判断，不能伪报为零动作。PX4 ULog独立保留和取回，不由此报告替代。

## OFFBOARD正常解锁修正

新命令`uav arm`的用途是：**已选择OFFBOARD、正式任务和定位稳定就绪后，仅发送正常解锁，由现有任务立即进入TAKEOFF。** 它不发送NAV_TAKEOFF，不自动切模式，不强制解锁、不降低飞控门槛、不自动重试；接管／切模式中止行为保持。该命令同步和验证完成前不能按已部署使用。

门控检查包括：未解锁且FCU连接、模式为OFFBOARD；model/frame/inference/pose/FCU/TF均健康、任务phase=0且未中止、动作门true／auto_start=false／危险品目标明确；外部vision／本地pose时间新鲜、设定值流至少有连续样本，并保持约2秒稳定。`uav arm --check`同样检查，但不调用任何动作服务。正常执行`uav arm`会真实解锁并立即开始任务，不是地面只读命令。

新流程为A→正式mission→确认健康→QGC只选择OFFBOARD→空闲RK终端`uav arm --check`→现场放行后`uav arm`。QGC的Takeoff属于另一条AUTO.TAKEOFF控制路线，不用于这个任务入口。若OFFBOARD不能选中或检查不通过，应排查设定值／定位／模式，不用自动起飞按钮绕过。

## 验证记录与待办

1. 实板安装前备份`~/shell_backups/lite-logging-20261028/before.tar.gz`，SHA256 `4ab385ce066dfefde8f48128ab6d33b49075c34649579101a779fcb108659d6f`。
2. 安装后读到任务／定位源文件及二进制SHA与昨夜回退一致；仅改变脚本入口，无C++重编译。
3. 记录器安全测试5项、本地解锁门控8项通过。尚未在真实飞控调用arming／mode／takeoff服务。
4. 首轮off-1失败原因是无稳定readiness，实际观察RANGE_STALE、pose_fresh=false、tf_fresh=false、FCU未解锁。SHELL随后确认起落架损坏、机体可能倾斜；未据此判TFmini接口损坏，也未放宽门限。该轮不生成有效CPU比较。
5. 14:20:34按SHELL断电修复意图下发正常关机，SSH reset后复查超时；未替代物理断电测量。
6. 待恢复USB：同步最新脚本；运行独立127.0.0.1:11340合成消息／模拟服务测试，验证记录故障隔离、`--check`不解锁、正常解锁仅调用CommandBool、AUTO.TAKEOFF和重复解锁被拒绝。再在修复放平的真实硬件上执行禁动关→开→开→关各90s性能窗口。任何解锁／投放只限模拟端点，不在真实硬件验收本轮动作。

性能预设：额外整机CPU≤5个百分点、可用内存减少≤150MiB、iowait P99≤5%，关键话题频率下降≤5%、健康≥99%、无真实设定值／GPIO命令或录包溢出。禁动没有实际设定值发送，不冒充空中控制性能。结果未齐前保持“基础部署、性能待验”的状态。
