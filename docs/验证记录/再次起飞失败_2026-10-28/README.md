# 约16:40再次起飞失败：USB取证、Nano坐标对照及附加负载修复

工程日期2026-10-28；SHELL报告的现场时间约16:40。板钟错误，下面用ROS原始时间戳、飞控日志原路径及相对时间关联，不把板端10月24日／ULog目录10月27日当成真实事故日期。

## 1. 本轮结论

**已查实并修复一项由此前启动辅助程序引入的持续CPU开销；尚不能认定撞墙根因闭环，也未完成飞行验收。** `uav_start.py`启动检查返回后仍接收整幅相机、地图、IMU、雷达和TF；已回收这些一次性订阅，备份后同步RK实际入口及工作区。`uav_arm.py`只改善拒绝原因提示，未放宽定位、模型、时间或模式要求。

两份最新ULog和对应ROS日志确认：

- **早切OFFBOARD的229号架次：** 切换后仍有TF过期阻塞，ROS直到切换约3.1秒后才进入TAKEOFF，随后再次暂停；飞控估计高度一直在地面附近。锁桨前主EKF已有惯性推算事件。它支持“升空被中断”，不支持简单断言OFFBOARD没有控制能力。
- **晚切OFFBOARD的230号架次：** AUTO.TAKEOFF已升至约1.5m，后切入OFFBOARD，ROS随后进入TAKEOFF，并实际下发首个航点。之后在TF_STALE／恢复之间反复切换、撤销／重发首点，符合断续控制的风险。**这次导航有启动，不能沿用13点那次的“从未进入任务”判断。**
- **坐标对照没有发现SLAM的XY轴交换或正负号反转。** 但定位姿态拼接、新鲜度处理、任务和规划器确实有改动，详见第4节；没有完成真实方向移动核验，不能仅凭静态对照排除所有坐标问题。
- **此次软件联查通过的范围仅为USB禁动与附加开销修复。** 当前机体不能平放、TFmini不可用于高度判断、舵机损坏。没有进行真实解锁、切模式、起飞、舵机动作或物理方向移动。

## 2. 原件与关联

原件保存在项目Git忽略目录`recordings/incident_20261028_1640/`：

| 原件 | SHA256 |
|---|---|
| incident_20261028_1640_ros.tgz | `734690486d4a9a56fdc747f6b45dd5a86f80ace0d8fc773be96aeba7ccd54018` |
| incident_deployed_sources.tgz | `08e084cd0f6b220e601d43119c5d498907b7c396df06a45a23ad2ee12a49694f` |
| log_229.ulg | `27bb702c79a0b6af96908c9373a82d731356b341f1b1a687ecce5daf3629dae8` |
| log_230.ulg | `a1f9e0aa044f0482ade01fe9a72fe77a37c88567bfc0b23c71992882f1dd0b30` |
| usb-readonly-evidence.tgz | `ae633002ba5d36cf286ad847fedbb5cc3a07fc47dfca111ff1d6f01d3fcdec9d` |

均经板端与本机SHA核对。ROS tar内611个普通文件逐项相同；Windows未展开的5个`latest`符号链接仍完整留在tar内，见[取证完整性](取证完整性.json)。只复制原件，不清理原飞行日志。事故取证先于部署和传感器检查。

| ULog | FCU原路径 | ROS运行ID | 关联事件 |
|---|---|---|---|
| 229 | `/fs/microsd/log/2026-10-27/23_41_50.ulg` | `629137b6-cf10-11f1-9d4f-14f5f9e6cd08` | logger路径、AUTO.TAKEOFF→OFFBOARD、反复kill |
| 230 | `/fs/microsd/log/2026-10-27/23_44_24.ulg` | `e89920a8-cf10-11f1-966a-14f5f9e6cd08` | logger路径、晚切OFFBOARD、首航点、kill |

SHELL提供的[FinalShell转录](SHELL提供的FinalShell转录.md)是整理材料，不能当作精确原始终端流水：其中`flamable`不在实际入口接受值中，实际ROS任务为`flammable`、stage1；`top`包含线程视图，多个相同命令行不能直接算作多份任务。GF／RP／WP mission received来自MAVROS任务协议同步，不能据此认定消防导航已开始。`GP: No GPS fix`也不单独证明室内外部定位失效。

唯一新增轻量轮次`20261024-020834-lite-7def300c`已经正常封包，但袋中实有消息仅覆盖1792778923–1792778961附近，缺vision pose、本地pose、任务状态、设定值等关键话题；它没有记录229／230号架次。正常封包不等于飞行阶段齐全，见[摘要](唯一新增录包摘要.json)。没有重新开启已禁用的记录器。

## 3. 两次切模式后的实际过程

下列ULog相对时间均以各自日志起点为0；飞控位置为NED估计，不是外部真实轨迹。锁桨前也可能已接触墙面，不能把“锁桨前”等同“碰撞前”。

| 事件 | 229早切 | 230晚切 |
|---|---:|---:|
| 离地检测 | 1.778s | 1.767s |
| AUTO.TAKEOFF→OFFBOARD | 2.900s | 12.008s |
| ROS任务TAKEOFF，按logger／kill事件近似关联 | 约6.3s | 约12.9s |
| 首航点 | 未见 | 约18.4s开始，反复重发 |
| 第一次kill | 7.729s | 23.723s |

229切换附近，ROS在1792780605.59进入`BLOCKED_TF_STALE`，1792780608.59才恢复并进入TAKEOFF，1792780608.79再次暂停。这解释了为什么切入OFFBOARD后仍可能没有持续爬升指令：进入TAKEOFF前的WAIT_FCU持有当前位置，输入阻塞后又保持故障时捕获的位置。不能靠QGC Takeoff绕过`uav arm`的拒绝来解决定位条件。

230在1792780769.53进入TAKEOFF，1792780774.93进入TASK_HAZARD，1792780775.03第一次发`hazard0`，随后在约0.4–0.8s间隔恢复、取消和重发。整轮ROS记到33次`location: TF_STALE`及33次`BLOCKED_TF_STALE`。输入恢复时并非从头重启任务，但会重新提交未完成目标。晚切后的ULog水平速度设定值存在正负变化；这支持控制确实在工作，不能直接推定其方向错误。

230的主EKF是实例1；**锁桨前未见该实例进入惯性推算或拒绝yaw事件**，相关事件发生在kill后约1.7秒及以后。因此不能把13点架次或229的EKF失效时序套用到230上。230对齐外部观测记录在OFFBOARD段最大间隔约0.752s，但该话题降频存储，不能把它当作原始ROS发布频率；定位断续由ROS TF_STALE独立支持。当前不能唯一确定“额外负载→TF延迟→摇晃→撞墙”的完整因果链。

详情：[ROS事件与计数](ROS事件关联.json)、[229分析](log_229_analysis.json)、[230分析](log_230_analysis.json)、[分阶段设定值和定位](分阶段定位与设定值.json)、[229逐样本](log_229_trace.csv)、[230逐样本](log_230_trace.csv)。分析使用现有[离线工具](../../../tools/analyze_px4_takeoff.py)。

## 4. Nano参考与当前实际RK的坐标链

只读对照SHELL指定目录下的`开发工作区/catkin_ws/src`与`Nano导出/2026-10-11/analysis_copy/catkin_ws/src`，以及本次从RK取回的部署源码；未改旧目录或归档。[坐标链文件SHA清单](Nano与RK坐标链文件SHA.json)保存四路文件身份。参考源码“存在”本身不证明该版本曾完成实飞验收。

| 环节 | 对照结果 | 解释 |
|---|---|---|
| SLAM主Lua | 两份Nano参考、当前本地、实际RK字节一致 | SHA256 `6c58aabb669ccee6cf3accc28c4f5b7860f90fb7bbdba7cf58c3ac233ce51ff9`；map／odom／base_link、IMU启用与无外部odom一致 |
| 雷达安装 | 仍为base_link→laser_link，yaw约π | 没有把180°安装旋转改成0或反转 |
| 定位XY | 仍读取odom→base_link并直接复制x/y | 没有新增x/y互换、取负或手工ENU→NED |
| 定位姿态 | 旧版直接用SLAM四元数；新版用IMU roll/pitch＋SLAM yaw | 确实修改过。二维SLAM不能提供完整真实倾斜姿态，修正意图是保留倾斜；本轮未在损坏机体上做方向移动验收 |
| 高度 | 旧版测距减固定离地值；新版投影、杆臂与标定门控 | 确实修改过。本轮停用TFmini且不输出vision，未重新校准或评价当前数值 |
| 时间／异常 | 旧版用当前时间反复发布可取得的TF；新版拒绝过期、跳变、缺测 | 旧实现可能把旧定位包装为新数据。不能为复刻旧行为而恢复伪新鲜时间戳 |
| 导航 | 旧EGO／rescue_mission换为已授权的DWA／MoveBaseMission | 不是原任务逐字迁移；当前增加map与PX4 local的平移／航向对齐 |
| 速度转换 | DWA输出body-FLU，经实际PX4 local yaw旋转一次到local ENU | 数学旋转与原点平移7项板端测试通过；物理轴向正负仍待人工移动核验 |

当前场地航点先按`R(-field_yaw_deg)`变换到SLAM map，之后以同一机体的map／local观测建立坐标对齐。`--yaw -90`不是要求飞控yaw=-90°。首点`field(0,-1.95)→map(1.95,0)`符合这一公式，但物理现场是否按同一套场地定义摆放尚未验证。

`FRAME_LOCAL_NED`枚举不能单凭名称解释为ROS调用方应手工交换XY／翻转Z：实际MAVROS1.16的[setpoint_raw转换](https://github.com/mavlink/mavros/blob/1.16.0/mavros/src/plugins/setpoint_raw.cpp)和[vision_pose转换](https://github.com/mavlink/mavros/blob/1.16.0/mavros_extras/src/plugins/vision_pose_estimate.cpp)已执行ENU／NED及机体轴转换。本轮不盲改yaw、速度符号或地图旋转。

本机候选`shoot_with_move_base.cpp`含昨夜诊断发布等修改，SHA与实际RK不相同；已取回实际源码核对，差异主要是audit／诊断状态，未以本机候选冒充当前部署。板端任务二进制SHA保持`e1c5e29da9ce687f067867a3c3c12a1eeee83080a0d3d3f0a4da7dd3eb5a5d28`，本轮没有重新编译或替换C++。

## 5. 已修复的附加开销与部署

`wait_ready()`的局部订阅回调仍被rospy持有；函数返回不会自动注销。A及mission启动器都可能在后续整个飞行期间继续处理完整RGB帧、地图与TF。它不是日志禁用开关控制的录包进程，因此即便`UAV_LOG_DISABLED=1`也存在。

在私有`127.0.0.1:11341`、无硬件／真实飞行服务的环境，使用1280×720 RGB／30Hz及合成未解锁传感器运行[回归脚本](../../../tools/uav_start_gate_probe.py)：

| 场景 | 检查返回后相机连接 | 启动检查进程CPU（单核100%口径） |
|---|---:|---:|
| 原版正常就绪 | 1 | 51.84% |
| 修正版正常就绪 | 0 | 0.60% |
| 修正版缺vision超时 | 0 | 0.60% |

见[机器结果](启动订阅回收对照.json)。这是单个辅助进程的对照，**不是整机CPU从52%降到1%，也不是Wi-Fi或飞行性能验收**。本轮未优化相机、RKNN、SLAM或DWA主功能；未停用NPU服务。

已在`/home/jcfh/shell_backups/start-gate-20261028/`备份实际入口和工作区两份旧文件，再同步新文件。Python2编译检查通过，本机、`~/catkin_ws/tools/`和`~/.local/share/uav/`SHA一致：

- uav_start.py：`75f77b1b1b61a3c1d1267da2104988b3606fedf320b5a91dda12c2adbb8f4d50`。
- uav_arm.py：`b3695e0eac7d76c81a1f2d1ada17a5c1453fd767d425e76d8e2914777991b76a`。拒绝提示会列出具体`tf_fresh`／`pose_fresh`等项目及定位状态；8项离线门控测试通过，既有约束不变。

## 6. 真实USB禁动验证及收尾

独立`127.0.0.1:11342`启动真实PX4传感器白名单MAVROS、雷达／Cartographer／相机、禁动DWA和RKNN。MAVROS没有cmd／set-mode／setpoint插件，任务`enable_actuation=false`、`auto_start=false`、`use_real_servos=false`；定位`use_tf_mini=false`，不打开TFmini测距，不输出vision pose。脚本见[本次实际执行副本](USB禁动联查脚本.py)。

稳定窗口60.53s，见[机器摘要](USB禁动验证.json)：

| 项目 | 结果 |
|---|---|
| FCU | connected，未解锁，MANUAL |
| IMU | 3048条，约50.35Hz，最大接收间隔50ms |
| 雷达scan | 610条，约10.08Hz，最大间隔123ms |
| 相机 | 1832条，约30.26Hz，最大间隔73ms |
| 地图 | 61条；只证明地图生成，不验收倾斜机体下地图几何 |
| map→base_link TF | 120次采样最大年龄71.35ms；全部小于250ms |
| 系统CPU | 此测试平均49.86%，含Python图像观测器开销；不同工况，不与旧Wi-Fi／飞行直接作差 |
| 禁动 | 无servo节点；控制设定值／GPIO消息0；vision pose 0 |
| 状态 | HEIGHT_DISABLED、BLOCKED_POSE_STALE，符合禁用高度且未建立完整飞行定位的预期 |

本轮**没有验收`uav arm`可通过或正式任务定位READY**，也未放宽条件制造通过。板端坐标数学7项、定位几何／新鲜度11项测试通过，均为无动作测试。曾因诊断脚本雷达设备路径不匹配中断第一轮，已清理自有进程、修正为枚举实际串口后重新完整测量；失败轮未计入结果。

最终退出所有本轮自有ROS、传感器和测试进程，经root只读扫描`/proc/*/fd`核对飞控／雷达／相机／I²C句柄空闲；日志INACTIVE且禁用配置保留，NPU固件、电源、代理及target四项均active。末尾5个1秒空闲整机CPU样本为0.68%、0.68%、0.17%、0.17%、0.34%，只说明收尾无持续测试负载，不能当成飞行负载。见[部署哈希、最终进程和资源核验](部署与最终资源核验.json)。未关机，未更改飞控参数／系统时钟、未恢复记录器、未切换Wi-Fi调试。

## 7. 接续条件

1. SHELL空闲接入局域网后，继续在静止禁动状态核对实际A／C启动器不保留订阅，以及原生监看连接后的TF时间年龄和额外进程。USB结果不代替该工况。
2. 当前不能平放且TFmini不可作为高度依据；在恢复机械支撑／可靠测高前，完整定位READY、正常解锁和起飞不列为已验收。损坏舵机不得启动正式mission入口，因为它会启用真实舵机。
3. 之后由SHELL明确配合做无动力的前／后／左／右平移及yaw转动，对照真实方向、SLAM odom／map、PX4 local ENU；核对物理方向和延迟后才决定是否需要坐标修改。
4. 对照229与230分别追查定位间断；230没有锁桨前EKF失效证据，不能用229结论覆盖。必要证据包括实际网络监看负载、TF／IMU时间链、机械振动和独立方向反馈；不以立即试飞猜测根因。

本轮USB限定验收结果已具备；Gmail通知只说明这一范围和待局域网接续，不表示无人机可以起飞。
