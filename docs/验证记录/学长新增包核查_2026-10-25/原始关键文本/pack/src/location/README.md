# location —— S2 + Pixhawk 6C + TFmini 定位桥

2026 机上只有三件定位硬件：RPLIDAR S2（2D 激光）、Pixhawk 6C（IMU/飞控）、TFmini
（向下装在机臂上，只用来定高）。`location.launch` 把这条链一次起完：

```
rplidarNode(/scan) ─┐
                    ├─ cartographer ── map -> odom -> base_link
/mavros/imu/data ───┘                         │
                                              ▼
                                   location_node: 读 odom->base_link
                                   + TFmini 高度 -> /mavros/vision_pose/pose
                                              │
                                              ▼
                                            PX4 EKF
```

## 启动

```bash
roslaunch location location.launch
# 可选：open_rviz:=true，雷达口 lidar_serial_port:=/dev/ttyUSB0
```

`cartographer_2Dlidar_mapping_location_params.lua` 是唯一在用的 SLAM 配置：
`tracking_frame=base_link`、`published_frame=base_link`、`provide_odom_frame=true`，
所以 **odom->base_link 由 cartographer 发**，`location_node` 只读不写。

## TFmini 定高

TFmini 向下装在机臂上，起飞时离地 `tf_mini_height`（默认 0.09 m）。节点发布：

```
z = 距离 - tf_mini_height
```

放在地面时 `z≈0`，之后 `z` 就是相对起飞点的离地高度（传感器处）。PX4 的 local z
原点本来就在起飞点，所以直接把这个 z 作为 `/mavros/vision_pose/pose` 的高度即可；
**不需要 IMU 姿态、三维安装偏移或额外的高度基准**。

因为这个量只用于定高，`tf_mini_height` 只影响绝对高度读数：填 0 不影响高度保持
（常数在 PX4 的起飞零点里被抵消），填对只是让日志/高度更好读。机臂上的横向偏移、
机体原点与传感器的竖直差值也都不参与定高——高度保持只看距离的变化量。

## 参数（只列还在用的）

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_frame_id` / `source_frame_id` | `odom` / `base_link` | 发给 PX4 的位姿是哪条 TF |
| `publish_rate` | 20 | vision_pose 频率 |
| `use_tf_mini` | true | 关掉就用 SLAM 的 z（2D 时恒为 0，不建议） |
| `tf_mini_type` | i2c | `i2c` 或 `uart` |
| `tf_mini_height` | 0.09 | 起飞时 TFmini 离地高度；`z = 距离 - 它` |
| `bus` / `address` | 1 / 16 | I2C 总线和 0x10 地址；换主板先 `i2cdetect -l` |
| `serial_port` / `baud_rate` | /dev/ttyUSB0 / 115200 | 仅 `tf_mini_type=uart` |
| `fcu_url` | /dev/ttyACM0:921600 | Pixhawk 6C USB；USB CDC 忽略波特率，但 MAVROS 要匹配 |

## 已经删掉的分支

* T265（`rs_t265.launch`、`rplidar_location=false`、roll/pitch/yaw 安装角补偿）——
  机上没有 T265；
* `is_simulation` —— 仿真用 `gc_2026_sim` 自己的 rig/SLAM，不进这个包；
* S2 型号开关 —— 只有 S2，固定 1 Mbaud + DenseBoost；
* `load_model` / iris urdf 的 RViz 显示；
* TFmini 的三维安装偏移/倾斜补偿 —— 只定高不需要。

`param/ROSSLAM/` 里还留着 T265 和 3D 的 lua（`cartographer_2Dlidar_mapping_params.lua`、
`cartographer_3Dlidar_location_params.lua` 等），它们**不在**当前 launch 链上，只是历史
参考；真正加载的只有 `cartographer_2Dlidar_mapping_location_params.lua`。

## 台架检查

```bash
# TFmini 单独读数（放在地面应约 0.09 m）
rosrun location tfmini_test i2c 1 16
rosrun location tfmini_test uart /dev/ttyUSB0 115200

# TF 链：map -> odom -> base_link -> laser_link 都要能查
rosrun tf tf_echo odom base_link
rostopic hz /mavros/vision_pose/pose      # ~20 Hz

# 放在地面 z≈0；手动抬高 0.5 m，z 应增加约 0.5
rostopic echo /mavros/vision_pose/pose/pose/position/z
```
