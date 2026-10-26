# DWA导航与四类YOLO任务

本包当前采用move_base／DWA规划、独立异步RKNN识别和消防／人员巡检状态机。SHELL已确认退出EGO默认启动链，保留现有NPU服务和雷达SDK 2.1.0；历史实现仍可从Git历史查看。

本包只使用学长交付的训练产物，不负责训练。当前类别顺序为0 person、1 explosive、2 flammable、3 toxic。模型为416输入，文件位于detect_mode/26；模型、labels和解析布局必须一起匹配，不能混用旧六类模型。

[项目状态](../../docs/项目资料/任务状态.md)和[工程日志](../../logs/2026-10-25_装配后功能复检.md)记录实际部署／验证证据。本文描述接口和行为，不把构建、mock或单图推理通过写成飞行验收。

## 启动入口

相机与独立识别：

```bash
roslaunch usb_cam rk_usb_camera.launch
roslaunch navigation_2d detect.launch
```

默认任务入口：

```bash
roslaunch navigation_2d navigation_2d.launch
```

默认enable_actuation=false、auto_start=false、use_real_servos=false。此时任务节点只运行模型、订阅输入并发布状态，不输出飞控设定值、模式／解锁请求、GPIO或move_base目标；真实舵机节点也不启动。缺少定位、TF或飞控输入时会报告阻断，不因程序仍运行就判定整机就绪。

location.launch负责真实定位／MAVROS等外设链，不能当作单纯看图入口；TFmini等尚未接入时不得据此宣称定位验收。Gazebo／PX4 SITL资源继续位于gc_2026_sim，不由本包启动真实飞控来代替仿真。

实际任务必须显式开启enable_actuation；自动切模式／解锁还需auto_start=true，真实舵机还需use_real_servos=true。即使显式开启，模型和关键输入门控仍生效。台架、仿真与真机的参数及验收分别记录。

## 模型与健康门控

RKNN使用已验证的1.7.5头文件／API库与现有shell-npu.target，不安装第二套NPU上电服务。CMake需要正确找到该SDK；不同芯片系列的运行库不能按相似文件名替换。

构造时模型初始化失败，main以非零状态退出，不继续一个无模型任务。成功后启动独立推理线程；任务callback／50Hz控制timer均不调用同步推理。原始四类模型曾实测约0.66秒/图，实际性能以日志为准。

任务只有在新鲜图像及至少一次有效完成的推理满足inferenceReady()后才允许推进。运行中的模型失败／超时或图像失效同样阻断业务。缓存使用单调sequence，只有新完成结果可计入detect_times，不能把同一结果反复累计。

默认detect_times=5、detect_timeout=8s、detect_confirmation_window=6s。确认窗口按原始模型实际速率扩大，进入DETECT会跳过当前缓存；相机姿态切换和实际识别精度仍须台架验收。frame_timeout默认1s、inference_timeout默认3s。空检测结果可以是成功推理，但不能当作投放授权。

## 控制、坐标与时效

move_base只做水平导航。DWA保留全向XY速度配置，航向保持起飞时捕获的方向，高度由任务的本地高度目标控制。

/cmd_vel按body-frame速度解释，每次发布前用当前有效机体yaw旋入PX4本地ROS坐标；水平速度按向量范数限制为max_vel，不能分别限制两轴后允许对角超速。最终ENU／FLU到PX4坐标的转换由MAVROS负责。

map和PX4本地系不假定原点／航向一致。节点使用同一时段map→base_link TF与MAVROS local→body位姿建立平面旋转／平移，用于固定救助投放点和goal航向。起飞XY捕获实际本地位姿。场地航点仍按field_yaw约定旋入map；现场H点与map原点、场地角度必须另行核对，不能用坐标代码代替场地测量。

| 参数 | 默认 | 含义 |
|---|---:|---|
| pose_timeout | 0.5s | 本地位姿最大年龄，非有限位姿／无效四元数也拒绝 |
| state_timeout | 3s | FCU状态最大年龄，且必须connected |
| tf_timeout | 0.5s | map/body TF和变换缓存最大年龄 |
| alignment_tolerance | 0.2s | TF与本地位姿时间差上限 |
| planner_timeout | 0.5s | DWA速度最大年龄；过期发零水平速度并保高度 |
| map_frame / body_frame | map / base_link | 任务TF坐标系，改变时需同步核对costmap配置 |
| odom_topic | /mavros/local_position/odom | 显式提供给DWA的速度里程计话题 |

关键输入故障时暂停业务计时、清空识别累计并取消在途导航目标；有新鲜本地反馈时保持捕获的位置，没有新鲜反馈则不伪造新控制样本。恢复后重新取得导航／识别输入再继续。任务callback及timer互斥，重推理留在独立线程，真实控制频率须机上验收。

TAKEOFF、TRANSIT、SETTLE、DETECT、DESCEND、DROP、CLIMB要求飞控持续armed且处于OFFBOARD。新鲜飞控状态一旦显示解除武装或人工切离OFFBOARD，立即取消在途目标、停止任务设定值／GPIO／模式请求并锁存BLOCKED_FLIGHT_AUTHORITY_LOST；即使之后恢复OFFBOARD也不自行继续，必须重新启动任务节点。WAIT_FCU中的显式自动准备不受此锁存条件替代。

起飞只有达到fly_height并满足2秒起飞观察时间后才进入危化巡检。takeoff_timeout默认20秒，可配置用于台架mock；到期未满足起飞条件则锁存BLOCKED_TAKEOFF_TIMEOUT，停止任务输出，不再按原包行为横向巡航。该中止同样需要重启任务，不能靠后续高度变化自动解除。

LAND只接受节点自己请求的模式转换：stage0保持OFFBOARD并请求POSCTL，stage1等待实际POSCTL后才请求AUTO.LAND，stage2允许POSCTL→AUTO.LAND及正常落地解除武装。LAND出现MANUAL等未请求模式仍锁存飞行权丢失；离开OFFBOARD后不再发送任务设定值，正常解除武装可完成任务，不重新解锁。

costmap的0.32m候选footprint及膨胀参数来自收到的配置，并非已测机体包络；实际旋翼／机臂外缘和通道净空仍须测量。雷达2.1.0通过不代表避障参数自动合格。

## 任务与投放策略

流程为WAIT_FCU→TAKEOFF→危化巡检→人员巡检→RETURN→LAND。危险品目标只接受explosive／flammable／toxic及对应中文别名；person不得作为危险品目标或授权消防投放。

消防投放必须先取得本识别阶段对指定危险品的确认。所有危险品卡片均未识别时记录HAZARD_NOT_FOUND_NO_DROP，不在最后一点盲投，继续人员巡检后返航。无人员识别时同样不投救助物资。

stage=1时确认person只标注并返航；stage=2或rescue_drop_enabled=true时，确认person后可授权救助物资投放。消防与救助授权分别检查；超时、空结果或路线耗尽不产生授权。

按SHELL决定保留：**已经识别并取得合法授权、进入DESCEND后，即使高度或XY未达标，下降10秒超时仍转DROP；投放回执超时仍爬升继续。** 这不等于未识别也投放。关键模型／输入故障期间暂停业务及相关计时，恢复后继续原状态，不能绕过识别授权。

GPIO回执表示节点报告动作结束，不等于传感器确认物资实际释放。ACK超时也不能在工程记录中冒充投放成功。

## 观察接口

- /detect_result：图像预览／标注。
- /mission/status：锁存事件，包括DRY_RUN_READY、WAIT_PLANNER、WAIT_TARGET、BLOCKED_MODEL_INFERENCE、BLOCKED_CAMERA_STALE、BLOCKED_POSE_STALE、BLOCKED_FCU_STALE、BLOCKED_TF_STALE、BLOCKED_FLIGHT_AUTHORITY_LOST、BLOCKED_TAKEOFF_TIMEOUT、HOLD_PLANNER_STALE，以及未识别不投放事件。
- /mission/readiness：锁存String JSON，包括model_loaded、frame_fresh、inference_ready、pose_fresh、fcu_fresh、tf_fresh、actuation_enabled、flight_authority、mission_aborted、abort_reason、phase、hazard_detection、person_detection。flight_authority表示当前新鲜状态是否armed+OFFBOARD，LAND正常退出OFFBOARD后可为false；mission_aborted及abort_reason表示不可自动恢复的任务中止。检测状态为pending／confirmed／not_found／no_route；confirmed只表示识别，不表示投放完成。
- /gpio_cmd及MAVROS控制接口：受动作总闸和健康门控约束，默认不发布。

phase整数依次为0 WAIT_FCU、1 TAKEOFF、2 TRANSIT、3 SETTLE、4 DETECT、5 DESCEND、6 DROP、7 CLIMB、8 LAND、9 DONE。独立ROS mock不能连接真实MAVROS或舵机；以日志验证闸门、坐标、时效和策略，真实飞行验收另行进行。
