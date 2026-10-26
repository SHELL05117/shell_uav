# location：S2、飞控IMU与TFmini定位桥

本包读取Cartographer产生的`odom → base_link`，结合独立飞控IMU姿态和TFmini测距，发布`/mavros/vision_pose/pose`。桥接节点不发布`odom → base_link`，避免与SLAM重复广播。

当前TFmini仍未接线，真实测距、安装几何与PX4高度融合尚未验收。默认`height_reference_calibrated=false`；0.09 m和零偏移只是配置占位值，不代表SHELL已经测量。不能仅为获得输出而将标定标志改成true。

## 数据有效性

节点在有实际有效输入时才发布vision pose，状态位于私有`~status`话题。通过`location.launch`启动时通常为`/location_node/status`。

- TF必须包含有限位置、有效四元数和真实动态时间戳；默认超过0.25 s停止发布。不能用时间戳0的静态`odom → base_link`替代动态定位。
- TFmini量程默认0.05–6.0 m，单次跳变门限0.60 m；连续20个独立、彼此接近的异常阶跃样本才允许重建基准。重复同一时间戳不增加确认次数，拒收样本不刷新有效时间。
- 有效测距默认超过0.30 s即停止发布，绝不继续使用旧距离并改盖当前时间，也不在TFmini缺失时回退成2D SLAM的零高度。
- IMU姿态必须来自`source_frame_id`（默认`base_link`），四元数有效、姿态协方差未标为不可用，且默认不超过0.30 s。2D SLAM的yaw不能代替真实roll/pitch。
- XY跳变默认门限0.30 m；连续10个独立且接近的新位置才允许重新接受。异常位置期间不发布伪造的新位姿。
- 高度默认允许范围−0.20–3.00 m，光束偏离竖直超过35°或几何无效时拒发，不将超限高度强行伪装成有效读数。

输出时间戳保留真实参与测量的最早时间（TF、range、IMU之最小值），不会用当前时间替代旧测量时间。I2C/UART模式下，测距时间是板端通信读取调用附近的本机时间；它不证明传感器内部另有独立时间戳。

常见状态：`WAIT_TF`、`TF_STALE`、`WAIT_RANGE`、`RANGE_STALE`、`IMU_STALE`、`HEIGHT_REFERENCE_UNCALIBRATED`、`HEIGHT_DISABLED`、`TF_JUMP`、`HEIGHT_GEOMETRY_OR_LIMIT`；输入全部有效时为`READY`。设备无法打开会非零退出，不能作为模块已正常运行。

## 姿态、外参与高度基准

`tf_mini_height`表示水平起飞参考姿态下，传感器光学中心到地面的**竖直**距离。`sensor_offset_x/y/z`是光学中心相对机体原点的安装位置，采用机体系FLU：x前、y左、z上；`sensor_beam_x/y/z`是同一机体系下的测距光束方向，默认`(0,0,-1)`。

设测距为d，IMU旋转为R，单位光束方向为b，安装位置为r，参考竖直间隙为h0，代码使用：

```text
z = d × (−R·b).z − h0 + r.z − (R·r).z
```

这使斜距先投影到竖直方向，再修正姿态变化引起的安装位置高度变化。输出姿态采用IMU的roll/pitch及SLAM的yaw。光束、外参和h0均需按实物测量；该模型假设目标地面平整且起飞参考姿态按约定建立，不能宣称常数一定由PX4自动抵消。

将`height_reference_calibrated`设为true前，须记录实际安装位置、光束方向和参考间隙，确认IMU坐标／时序，再分别验证地面零点、抬升变化、倾斜补偿、掉线拒发及PX4融合。此标志只是对已完成标定的显式声明，不会自动测量或标定。

## 真实入口与参数

`location.launch`会启动真实雷达、MAVROS及SLAM，不是只看数据的无硬件命令。当前保留雷达SDK 2.1.0。原独立只接收飞控入口`rk_mavros_bench.launch`另行保留。

| 参数 | 默认 | 作用 |
|---|---|---|
| `target_frame_id` / `source_frame_id` | `odom` / `base_link` | 读取的动态TF |
| `publish_rate` | 20 Hz | 最大位姿发布频率，输入无效时拒发 |
| `use_tf_mini` | true | 测距高度必需；关闭后状态为`HEIGHT_DISABLED`并拒发vision pose，当前2D SLAM不提供完整3D高度降级 |
| `tf_mini_type` | `i2c` | `i2c`、`uart`或显式合成`topic` |
| `height_reference_calibrated` | false | 未完成几何标定时阻止测距融合输出 |
| `tf_mini_height` | 0.09 m | 待实测的光学中心参考竖直间隙 |
| `sensor_offset_x/y/z` | 0/0/0 m | 待实测的FLU安装位置 |
| `sensor_beam_x/y/z` | 0/0/−1 | 待确认的机体系光束方向 |
| `imu_topic` | `/mavros/imu/data` | 独立真实姿态输入 |
| `tf_timeout` / `range_timeout` / `imu_timeout` | 0.25/0.30/0.30 s | 节点私有参数的新鲜度门限 |
| `bus` / `address` | 6 / 16 | 本次BSP已核对I2C6对应`/dev/i2c-6`；0x10仍是候选设备地址，不是接线／扫描验收 |
| `serial_port` / `baud_rate` | `/dev/ttyUSB0` / 115200 | UART配置，不能与雷达串口混用 |

UART读取采用有界poll，每次先丢弃调用前的串口积压，再等待新到达的完整帧并排空当前数据，返回最新校验有效帧。一次成功不能在后续无新数据时重复使用，EOF／无数据／校验错误不再无限等待。I2C回复校验失败不再被当作成功。它们仍需在TFmini实际接线后验证，软件改动或合成输入不能替代真实设备验收。

I2C默认总线6依据[本项目接线表与只读软件映射证据](../../docs/项目资料/RK板_PCA9685与TFmini接线表_2026-10-24.md)：`i2c-6`的设备树节点为`i2c@ff150000`，对应`i2c6-xfer`。这只确认当前BSP的软件映射；TFmini模式、地址、引脚电平和实物接线仍待核对，不能因为默认值已调整就宣称外设已经接通。所有入口保留显式覆盖总线／地址的能力。

## 不连接设备的验证

纯函数测试覆盖范围、重复样本、阶跃、XY跳变、时间新鲜度、斜距／安装位置投影、倾斜和高度限制及UART帧校验。`localization_guard_test.cpp`自身提供唯一main入口。

在已编译的catkin工作空间执行：

```bash
source devel/setup.bash
catkin_make run_tests_location
catkin_test_results build/test_results/location
```

其中`tfmini_uart_pty_test`通过Linux伪终端向真实UART类发送协议字节，不接传感器；覆盖调用前积压拒收、新样本替换、跨多次read的大突发取末帧、噪声／坏校验／分片重同步，以及停流后不能重复刷新。该测试验证主机串口行为，不能证明真实传感器内部采样时钟。

集成probe由脚本自行建立独立`127.0.0.1:11329` master，仅启动真实定位桥并注入合成TF、Range和IMU；测距接口明确选`topic`，输出重映射到`/location_probe/pose`。不启动MAVROS、飞行任务、雷达、舵机或I2C/UART设备。

```bash
source devel/setup.bash
python2 src/location/test/localization_bridge_probe.py /绝对路径/本次独立输出目录
```

脚本检查15个场景：缺TF、缺测距、正常高度、坏值过期、恢复、缓存测距过期、TF过期、IMU过期、倾斜投影、过度倾斜、未标定阻断及关闭高度时拒发SLAM-z等，结果保存为`localization-results.json`和原始ROS日志。应以实际退出码／JSON判定是否通过，不提前宣布成功。

`localization_guard_bench.launch`提供同一合成接口的手动节点入口，但必须由调用者使用独立master并提供测试消息；建议使用上述自动probe。合成测试中的`height_reference_calibrated=true`只声明测试夹具的已知几何，不能复制成真实无人机已标定状态。
