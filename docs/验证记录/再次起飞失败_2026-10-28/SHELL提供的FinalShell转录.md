

为了方便阅读，我将日志分为三个部分：终端指令交互记录、系统核心运行日志、以及系统资源状态快照。

---

第一部分：终端操作与指令反馈记录（对应图2）

注：图2未显示具体时间戳，但根据图3的时间（02:38:24）以及图2上次登录时间（02:34:57），这属于发生在前期的指令操作。

```text
Welcome to Ubuntu 18.04 LTS (GNU/Linux 4.4.194 aarch64)
Last login: Sat Oct 24 02:34:57 2026 from 192.168.1.100

jcfh@toybrick:~$ uav arm --check
UAV ARM: Arming refused: mission inputs not ready, vision missing/stale, location not READY

jcfh@toybrick:~$ uav arm
UAV ARM: Arming refused: mission inputs not ready, vision missing/stale, location not READY
```

(日志分析：此时由于视觉或定位未准备好，无人机拒绝解锁。但在随后的图3日志中，定位状态变为了READY。)

---

第二部分：ROS与建图定位核心日志（对应图3、图4）

注：已按 [Unix时间戳] 严格升序排列，并剔除了图3与图4完全重复的截图重叠部分（如 1792780704 相关的 Found xx.lua 等）。

```text
[ INFO] [1792780704.103443087]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder.lua'.
[ INFO] [1792780704.103548087]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder.lua'.
[ INFO] [1792780704.103708212]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder_2d.lua'.
[ INFO] [1792780704.103854045]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder_2d.lua'.
[ INFO] [1792780704.104290962]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder_3d.lua'.
[ INFO] [1792780704.105260753]: I1024 02:38:24.000000 21751 configuration_file_resolver.cc:41] Found '/opt/ros/melodic/.../trajectory_builder_3d.lua'.
[ INFO] [1792780704.176799878]: I1024 02:38:24.000000 21751 submap_2d.cc:187] Added submap 1
[ INFO] [1792780704.177035837]: I1024 02:38:24.000000 21751 map_builder_bridge.cc:130] Added trajectory with ID '0'.
[ INFO] [1792780704.395317462]: I1024 02:38:24.000000 21751 ordered_multi_queue.cc:172] All sensor data for trajectory 0 is ready.
[ INFO] [1792780704.407232337]: I1024 02:38:24.000000 21751 local_trajectory_builder_2d.cc:138] Extrapolator is still initializing.
[ INFO] [1792780704.665431129]: location: READY
READY: unarmed FCU, live IMU/scan, map, location and TF.
Foxglove native ROS 1: http://192.168.1.102:11311; set client ROS_HOSTNAME to the computer LAN IP; map fixed/display frame.
[ INFO] [1792780710.717951215]: HP: requesting home position
[ INFO] [1792780711.509334715]: MPP/RGA frame 301: 1280x720 NV12 stride=1280x720 -> RGB8
[ INFO] [1792780715.715692259]: GF: mission received
[ INFO] [1792780715.718395717]: RP: mission received
[ INFO] [1792780715.720554634]: WP: mission received
[ INFO] [1792780719.179827760]: I1024 02:38:39.000000 21751 collated_trajectory_builder.cc:72] imu rate: 50.00 Hz 2.0...
[ INFO] [1792780719.180078885]: I1024 02:38:39.000000 21751 collated_trajectory_builder.cc:72] scan rate: 10.01 Hz 9.99...
[ INFO] [1792780720.717576303]: HP: requesting home position
[ INFO] [1792780721.507165303]: MPP/RGA frame 601: 1280x720 NV12 stride=1280x720 -> RGB8
[ WARN] [1792780723.967988471]: location: range sample rejected (range, jump, or repeated timestamp)
[ WARN] [1792780728.667715765]: location: range sample rejected (range, jump, or repeated timestamp)
[ INFO] [1792780730.717533724]: HP: requesting home position
[ WARN] [1792780730.768827599]: GP: No GPS fix
[ INFO] [1792780731.508942266]: MPP/RGA frame 901: 1280x720 NV12 stride=1280x720 -> RGB8
```

---

第三部分：系统资源占用快照（对应图1）

注：图1展示的是 top 命令的实时快照，包含了系统负载和各ROS节点进程的CPU/内存占用，无独立时间戳，因此单独列出。

```text
Tasks: 53 total, 152 thr, 6 running
Load average: 12.27 5.48 2.96
Uptime: 00:15:59

[进程列表节选 - 占用较高者]
PID    USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
7608   jcfh      20   0  492M 5192R 31792 R  91.9   1.4  1:44.75 /home/jcfh/catkin_ws/devel/lib/usb_cam/usb_cam_node __name:=camera...
8125   jcfh      20   0  668M 36928 18668 R  87.5   1.0  1:44.14 /opt/ros/melodic/lib/cartographer_ros/cartographer_node -configuration_directory...
8363   jcfh      20   0  1589M 156M 33912 S  61.1   3.9  1:03.67 /home/jcfh/catkin_ws/devel/lib/navigation_2d/shoot_with_move_base_node __name:=shoot_with_move_base...
7639   jcfh      20   0  492M 5192R 31792 R  58.5   1.4  1:03.67 python2 /home/jcfh/.local/share/uav/uav_start.py mission --target flamable --stage 2 --yaw -90
8222   jcfh      20   0  134M 10216 5216 S  52.7   2.7  1:00.06 python2 /home/jcfh/.local/share/uav/uav_start.py mission --target flamable --stage 2 --yaw -90
8114   jcfh      20   0  132M 165M 5012 R  50.8   4.3  0:37.34 python2 /home/jcfh/.local/share/uav/uav_start.py mission --target flamable --stage 2 --yaw -90
...
```

(注：从图1可以看出，此时CPU负载非常高（Load Average 达到了 12.27），Cartographer和USB摄像头节点占用了大量的CPU资源。)

---

日志关键点总结：

1. 起先尝试 uav arm 解锁失败，因为定位未就绪（location not READY）。
2. 随后 Cartographer 初始化完成，输出了 location: READY。
3. 无人机开始接收任务（GF/RP/WP: mission received），IMU和雷达工作频率正常（50Hz / 10Hz）。
4. 运行中出现了一些警告：雷达范围采样被拒绝（可能因为跳跃、距离异常或时间戳重复），以及后续丢失了GPS定位（GP: No GPS fix）。
