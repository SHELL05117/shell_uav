# 10月23日后非学长来源算法与运行策略审查清单

SHELL逐项排查用；按SHELL要求启用审查方式。本轮只读取本地Git、学长原包、既有源码／实机证据并写本文，**没有改算法、回退版本、重新编译、连接RK、启用日志或发任何硬件命令**。

这里的“审查模式”是本轮只追溯、记录和暴露不确定性的工作方式，并非声称切换了应用内某个未核实的开关。

## 1. 范围、来源与阅读规则

- 时间范围采用北京时间2026-10-23之后的真实提交；为避免漏掉工程归10月23日的凌晨工作，也纳入10月24日03:31的b1cbd46。起点为10月22日基线d75551b，终点为本次审查前HEAD ccbb548；下列附录给全部提交，不只挑“看起来危险”的几行。
- **Git作者全部使用同一项目账户，不能证明代码由谁创作。** 本文以10月25日学长pack.7z原件／关键文本与实际diff判定：整文件导入不等于我原创，基于学长框架新增的门控／数学转换则单独列为我改的。来源不明不擅自认定为学长。原Nano仅作继承链对照，不改归档。
- 详细条目覆盖我的新增／重写／调参／运行策略；辅助测试和离线评估算法也列在T项，避免“所有”被缩成只有yaw。导入模型、学长原值、第三方原代码及纯构建／文档在附录逐文件交代。
- 每项依次给**原来怎样→我改了什么→为什么→依据→验证与缺口→排查提示**。“为什么”若只是我的工程推断，会明说；SHELL授权修正坐标／保护不等于学长指定了我的公式。现有实现反推的理由不伪装成学长原话。
- 提交时刻不等于实际写出代码或部署时刻。尤其18cd805是迁移集中提交，31a9aa0包含当天多轮修复，2f1853b是归档提交；部署状态以证据SHA／回退记录为准。本轮未在线复核，所有板端状态均注明为已保存证据。
- 这些是**需要审查的改动，不是已证实的故障归因**。数学／模拟／地面通过不能证明真机轴向、动态融合、闭环稳定性或实飞通过。请在每项“排查结论”填写自己的结果。

## 2. 建议先看的区域与事故版本边界

| 排查现象 | 优先条目 | 为什么先看 |
|---|---|---|
| uav arm反复被拒 | R12、C02、L02、L07、L14 | 我新增的解锁门、输入合取、定位断流及旧标定配置会直接拒绝 |
| OFFBOARD后停止升空 | C08–C12、L02、L04–L08 | 起飞捕获、阻塞位置保持、时间暂停／中止与vision拒发可能改变任务行为 |
| 左右摇晃／朝向疑似错 | L05–L06、C03–C07、C18–C19 | 姿态／高度、map-local对齐、实际yaw速度旋转、odom语义与避障权重均有实质变化 |
| 负载／短时卡顿 | R02、R05–R07、R10、R13–R16、V07 | 我新增媒体／推理／启动观察／日志进程；必须看事故前还是事故后版本 |

这张表只是阅读顺序，不是根因概率排名。

1. 16:40事故后取证时实际任务源码SHA为`01eab407cc619db1be18f5689dff54a35123c708528810462d7591d2e1c9577b`，二进制SHA为`e1c5e29da9ce687f067867a3c3c12a1eeee83080a0d3d3f0a4da7dd3eb5a5d28`；当前本地任务C++另含已回退的审计诊断。不能整份HEAD直接当事故源码。
2. e4882bd重型日志／C++诊断已于凌晨按性能条件回退；f253060/e3a1d97轻量版另行部署。SHELL于16:35要求关闭后`UAV_LOG_DISABLED=1`，之后记录器INACTIVE。旧诊断线程和记录器不能凭本地存在就算16:40常驻负载。
3. 28a178e的启动门订阅回收、细化arm拒绝信息，以及17f2b16的JPEG闲置订阅回收，均在16:40事故取证之后。本表列它们是为了追溯后续修复，不能倒置为事故前代码。
4. 229早切架次与230晚切架次不可混用：晚切约18.4秒已有hazard0反复下发，主EKF锁桨前没有229同样的惯性推算事件。原始位置是估计值，无碰墙真值时间。
5. 当前机械／TFmini／舵机条件已改变；本文不是复飞清单，也不把历史0.12m标定、空仓动作和静止验收当当前有效。

## 3. 原始证据入口

- **E0**：[学长10月25日原包、关键文本与包比较](../docs/验证记录/学长新增包核查_2026-10-25/README.md)。
- **E1**：[RK迁移回环与定时器退出故障](../docs/验证记录/RK迁移与通信核验_2026-10-23/README.md)。
- **E2**：[四类／DWA迁移授权、模型元信息及测试边界](../docs/验证记录/DWA四类部署_2026-10-25/README.md)。
- **E3**：[10月24日PX4 USB／GPS及相机修复](../docs/验证记录/RK外设修复_2026-10-24/README.md)。
- **E4**：[10月27日I²C协议、零点／近似授权与真实融合](../docs/验证记录/接线后整机复检_2026-10-27/README.md)。
- **E5**：[重型日志候选性能失败及精确回退](../docs/验证记录/机载飞行记录_2026-10-27/README.md)。
- **E6**：[无线监看、相机媒体后端与统一入口](../docs/验证记录/无线地面排查_2026-10-27/README.md)。
- **E7**：[16:40再次事故、Nano比较、ULog及启动负载修复](../docs/验证记录/再次起飞失败_2026-10-28/README.md)。
- **E8**：[隔墙Wi-Fi取证／显示转发负载修复](../docs/验证记录/隔墙WiFi禁动续验_2026-10-28/README.md)。
- **E9**：[轻量日志与显式解锁的范围／实测／暂停](../docs/验证记录/轻量机载日志_2026-10-28/README.md)。
- **E10**：[NPU电源归属／回退与厂商参考依据](../docs/验证记录/RK_NPU上电脚本验证_2026-10-24/README.md)。
- **E11**：[NPU实际上电／固件／六类实板推理](../docs/验证记录/RK_NPU实板上电_2026-10-24/README.md)。
- **E12**：[NPU开机自动准备与实际重启](../docs/验证记录/RK_NPU自启_2026-10-24/README.md)。
- **E13**：[NPU固定USB编号故障及拓扑修复](../docs/验证记录/NPU自启修复与相机复查_2026-10-25/README.md)。
- **E14**：[六类交回脚本问题与独立CPU复测](../docs/验证记录/YOLO训练成果验收_2026-10-24/README.md)。
- **E15**：[学长包选择性合并前的逐项评估](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)。
- **原包补充副本**：此前21份关键原文未覆盖的8份小型源码按原字节补入证据目录，[逐文件来源与SHA清单](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本SHA256.json)。只是原件副本，不修改学长代码；供云端同样可打开依据。
- **JPEG晚间修复前板端原件**：[docs/验证记录/非学长算法审查_2026-10-28/晚间修复前_JPEG板端原件.py](../docs/验证记录/非学长算法审查_2026-10-28/晚间修复前_JPEG板端原件.py)；SHA256 `49cb2d5942b257617347c8ef14e8721268defcb7de4de2adaa376d6408c3f84d`。它不含archive分流，与E5回退SHA一致；R06/R07的部署先后以此纠正，不改写旧报告原文。

所有下文“当前位置”链接指向工作树文件，标签附真实一基行号；链接目标保持普通相对路径，方便整体搬移。精确版本应结合提交或末尾SHA。

## 4. 定位／测高／输入桥

### L01 定位节点定时器生命周期（10月23日工程日）

- **提交／部署边界：** `b1cbd46`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/location_with_ftmini.cpp:239](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** 10月22日基线把两个ros::Timer放在文件全局；这与后来pack的全局定时器设计相似。
- **我实际修改：** 10月24日03:31提交先把Timer移到main内；18cd805迁移时进一步让桥对象拥有Timer、Buffer、传感器和互斥锁。
- **为什么这样改／要解决什么：** 当时回环退出出现TimerManager／静态析构异常，需保证退出顺序。不是为了改变定位数值。
- **依据与来源限制：** [E1](../docs/验证记录/RK迁移与通信核验_2026-10-23/README.md)“location-shutdown-gdb.txt”与该提交diff；源码显示只改变对象生命期。
- **已做验证及其范围：** RK应用回环正常退出；之后桥类经过合成输入测试。
- **SHELL排查点：** 此项不能用来解释方向反转，但新增多线程对象生命周期仍属我的实现。

```text
旧：ros::Timer timer, height_timer; // 全局
先修：ros::Timer height_timer; ros::Timer timer = nh_private.createTimer(...);
现：LocalizationBridge bridge(...); // 成员Timer随对象释放
```

- [ ] SHELL排查结论：________；对应证据：________。

### L02 定位时间戳及TF／测距／IMU新鲜度拒收

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/location_with_ftmini.cpp:156](../src/location/src/location_with_ftmini.cpp)；[src/location/include/localization_guard.h:10](../src/location/include/localization_guard.h)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack取最新TF后把输出stamp写为now；读过一次有效测距后range_valid持续为true，没有失联老化。
- **我实际修改：** TF默认0.25s、range／IMU各0.30s；定位允许未来误差0.02s。发布沿用TF时间，组合高度后取TF、range、IMU三者最早时间；不刷新旧输入。
- **为什么这样改／要解决什么：** 避免把缓存TF／旧高度包装成“刚测到”的外部定位供PX4持续融合。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)定位原文positionCallback中的now及range_valid；[E15](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)评估§4.3明确指出缓存测距失联仍刷新。阈值及最早时间策略是我的工程选择，无学长逐项指定证据。
- **已做验证及其范围：** 定位15项合成场景＋11项gtest；229／230真实日志存在TF_STALE（[E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md)），说明此门会实际截断vision。
- **SHELL排查点：** 优先核对真实TF最坏年龄、队列延迟及三源时序；现有静止采样不能排除飞行中门限过严、定位间断。硬件range在主机读回时盖时间戳，不是传感器内部采样时钟。

```text
if (!fresh(tf_stamp, now, 0.25)) { status("TF_STALE"); return; }
output.stamp = min(tf_stamp, min(range_stamp, imu_stamp));
```

- [ ] SHELL排查结论：________；对应证据：________。

### L03 测距量程、跳变和独立样本确认

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/include/localization_guard.h:17](../src/location/include/localization_guard.h)；[src/location/src/location_with_ftmini.cpp:44](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack只拒绝非有限／≤0读数；旧Nano／10月22日基线已有部分量程和跳变保护，算法思想并非本次首次出现。
- **我实际修改：** 重写为RangeGate：0.05–6m、单次跳变0.60m、20次相邻候选在0.10m内才接受新台阶；严格递增stamp，重复缓存不计数。
- **为什么这样改／要解决什么：** 保留旧保护，并消除重复使用一帧就能越过“连续确认”的漏洞。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)定位原文与G类逐段对比；[E15](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)评估§4.3。新状态机和计数／时间语义由我实现，不能称为原包原样。
- **已做验证及其范围：** 合成量程、跳变、重复时间戳gtest／定位桥测试通过；没有运动中的TFmini真值对照。
- **SHELL排查点：** 大跳变确认期会沿用旧高度直至超时，可能停止vision；应联合看RANGE_STALE及原始读数。硬件主机每次成功读回有新stamp，这不能独立证明传感器内部产生了新采样；当前损坏测距不作实机判据。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
16: // Counts distinct measurements, never repetitions of one cached ROS message.
17: class RangeGate
18: {
19: public:
```

- [ ] SHELL排查结论：________；对应证据：________。

### L04 SLAM平面位置跳变门

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/include/localization_guard.h:48](../src/location/include/localization_guard.h)；[src/location/src/location_with_ftmini.cpp:174](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack直接复制SLAM的XY；旧基线已有防跳变意图。
- **我实际修改：** XY变化>0.30m进入候选，候选连续10次且相邻差≤0.10m才接纳；重复相同TF不累加。拒收期间不发布vision。
- **为什么这样改／要解决什么：** 防止扫描匹配跳变直接送进飞控，同时允许持续一致的新位置恢复。
- **依据与来源限制：** 代码G及[E15](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)评估中的定位保护差异；具体阈值／恢复实现来自我，尚无实飞测量支持。
- **已做验证及其范围：** 定位gtest与15项桥测试；缺少已知距离的物理平移记录。
- **SHELL排查点：** 需要核对正常快速运动或低频TF是否也跨过0.30m，以及反复候选是否造成定位断流。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
47:
48: class PositionGate
49: {
50: public:
```

- [ ] SHELL排查结论：________；对应证据：________。

### L05 将直接减高度改为姿态投影＋杆臂补偿

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/include/localization_guard.h:94](../src/location/include/localization_guard.h)；[src/location/src/location_with_ftmini.cpp:195](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** 学长原文明确用z=d-tf_mini_height，并称无需IMU／三维安装偏移。
- **我实际修改：** 光束和杆臂按四元数旋转：z=d*(-world_beam.z)-h0+offset.z-world_offset.z；默认光束FLU负Z，倾角上限35°，z范围[-0.20,3.0]m。
- **为什么这样改／要解决什么：** 我认为倾斜时斜距不能直接当垂直高度，传感器离机体原点有偏移时需要补偿。
- **依据与来源限制：** 上述几何公式与G实现是我的推导；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)当时记录原包简化条件未验证。没有学长要求此公式或实机动态标定的证据。
- **已做验证及其范围：** 零姿态／倾斜／杆臂／超限数学测试；10月27日静止零点采样（[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)），不覆盖飞行动态精度。
- **SHELL排查点：** 必须审查IMU四元数坐标、beam符号和基准定义；姿态误差会直接变成高度误差。这是实质算法改动。

```text
world_beam = R(q_imu) * normalized(beam);
world_offset = R(q_imu) * offset;
z = range * (-world_beam.z) - h0 + offset.z - world_offset.z;
```

- [ ] SHELL排查结论：________；对应证据：________。

### L06 输出姿态从纯SLAM改成IMU横滚俯仰＋SLAM航向

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/location_with_ftmini.cpp:203](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack完整复制TF的orientation；2D SLAM通常只提供平面姿态。
- **我实际修改：** 把IMU的roll／pitch与SLAM的yaw重新setRPY成四元数，送/mavros/vision_pose/pose。
- **为什么这样改／要解决什么：** 配合倾斜测高，避免外部位姿总带零横滚俯仰；这属于我额外做的姿态拼接。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)定位position.pose.orientation=transform.rotation；当前combined.setRPY三行；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)定位几何说明。没有学长原包实现依据。
- **已做验证及其范围：** 合成姿态测试和静止融合有效；没有可追溯的真实机体正反转动、航向闭合或动态融合对照。
- **SHELL排查点：** 优先检查SLAM yaw与PX4 yaw原点／方向及时间一致性；输入来自飞控IMU又回送飞控的相关性也需审查。不能用“未改PX4固件”排除此项影响估计航向。

```text
getRPY(q_imu, roll, pitch, unused_yaw);
getRPY(q_slam, unused_roll, unused_pitch, yaw);
q_vision.setRPY(roll, pitch, yaw);
```

- [ ] SHELL排查结论：________；对应证据：________。

### L07 测高关闭／未标定／无输入时禁止vision及新增状态

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/location_with_ftmini.cpp:161](../src/location/src/location_with_ftmini.cpp)；[src/location/src/location_with_ftmini.cpp:194](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)；[src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack在无有效测距时回退SLAM z；use_tf_mini=false时也直接发SLAM三维位姿。
- **我实际修改：** use_tf_mini=false直接HEIGHT_DISABLED返回；IMU、range、标定任一缺失均不发vision；发布READY／WAIT／STALE／JUMP等状态。
- **为什么这样改／要解决什么：** 2D SLAM的z不能被我当成有效测高；不希望缺测时误发完整3D定位。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)无range回退分支与[E15](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)评估；SHELL授权修定位保护不等于逐项指定此阻断法。
- **已做验证及其范围：** 15项合成桥场景；USB／Wi-Fi实测关闭测高时确实无vision。
- **SHELL排查点：** 此项直接使正式定位／uav arm不能就绪；排查时区分“门控主动阻断”与SLAM没启动。不能只恢复假z来制造就绪。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
160:     // not permission to send its zero z to PX4 as a complete 3D observation.
161:     if (!use_range_) { status("HEIGHT_DISABLED"); return; }
162:     geometry_msgs::TransformStamped transform;
163:     try { transform = buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0)); }
```

- [ ] SHELL排查结论：________；对应证据：________。

### L08 IMU／测距输入合法性与互斥处理

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/location_with_ftmini.cpp:138](../src/location/src/location_with_ftmini.cpp)；[src/location/src/location_with_ftmini.cpp:132](../src/location/src/location_with_ftmini.cpp)；[src/location/include/localization_guard.h:80](../src/location/include/localization_guard.h)。
- **可逐行对照原件：** [src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)；[src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** pack定位桥不订阅IMU；无topic测距测试接口。
- **我实际修改：** 要求IMU frame_id==base_link、姿态covariance有效、四元数范数0.8–1.2且时间递增；增加Range消息frame／量程核对及topic测试通道；分输入／硬件／发布互斥。
- **为什么这样改／要解决什么：** 避免错frame或无姿态数据参与投影，允许隔离验证，不让并发读数破坏状态。
- **依据与来源限制：** L回调及G.rotate；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)测试记录。frame名称、范数区间和并发结构由我确定。
- **已做验证及其范围：** 合成坏四元数／过期／错frame测试；真实IMU地面输入通过。
- **SHELL排查点：** 合法格式不能证明IMU物理方向正确；frame命名变化会使IMU全部被拒收。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
137:   }
138:   void imuCallback(const sensor_msgs::ImuConstPtr &message)
139:   {
140:     if (message->header.frame_id != source_frame_ || message->orientation_covariance[0] < 0.0 ||
```

- [ ] SHELL排查结论：________；对应证据：________。

### L09 TFmini UART接收器重写

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/TFplus.cpp:50](../src/location/src/TFplus.cpp)；[src/location/include/TFplus.h](../src/location/include/TFplus.h)。
- **可逐行对照原件：** [src/location/src/TFplus.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/TFplus.cpp)；[src/location/include/TFplus.h 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/include/TFplus.h)。
- **修改前／来源：** pack保留旧串口读取实现及缓存字节处理。
- **我实际修改：** 非阻塞raw串口；每次调用清输入积压，300ms有界poll；只接收59 59且9字节校验正确的新完整帧，返回当前批最新值；部分帧不跨调用缓存；静默返回-1。
- **为什么这样改／要解决什么：** 避免积压旧帧被标为本次新测量、噪声／坏校验当作高度及读取长期卡住。
- **依据与来源限制：** 原TFplus与当前函数diff；G.validSerialFrame；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)五项真实POSIX PTY测试。清积压及时间边界是我的策略。
- **已做验证及其范围：** PTY验证积压、最新帧、噪声、分片、停流；本机实际使用I²C，因此不是本轮UART硬件验收。
- **SHELL排查点：** 跨调用丢弃半帧可能降低有效率；当前I²C链中不是活跃读数路径，保留为可切换算法。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
49:
50: float TFplus::getDist()
51: {
52:   if (!is_open_) return -1.0f;
```

- [ ] SHELL排查结论：________；对应证据：________。

### L10 TFmini I²C资源释放、命令防护与校验语义

- **提交／部署边界：** `18cd805；a765be7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/tf_mini_i2c.cpp:8](../src/location/src/tf_mini_i2c.cpp)；[src/location/src/i2c.cpp:13](../src/location/src/i2c.cpp)。
- **可逐行对照原件：** [src/location/src/tf_mini_i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/tf_mini_i2c.cpp)；[src/location/src/i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/i2c.cpp)。
- **修改前／来源：** pack含静态命令缓冲、构造时无协议探测写地址、sleep(0.x)整数截断、非测距校验失败返回true和只close不delete。
- **我实际修改：** 移除构造探测写；关闭失败fd；删除拥有的I2C对象；命令缓冲改局部／边界检查；sleep改usleep；坏校验统一false并核对帧头／回复长度／命令。
- **为什么这样改／要解决什么：** 消除资源泄漏、并发缓冲混用及错误回复伪成功；修正实际等待未发生。
- **依据与来源限制：** pack与当前tf_mini_i2c.cpp差异；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)、[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)及tfmini_i2c_protocol_test。分两次提交完成，不能只归到10月27日。
- **已做验证及其范围：** 驱动单测／真实静止读数通过；偶发坏帧仍保留。
- **SHELL排查点：** 拒收比例可能影响L02；本轮不将坏帧修成有效。当前损坏TFmini不作算法真值依据。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
7:
8: bool TFMiniPro::SendCommand(uint32 cmnd, uint32 param, uint8 *response)
9: {
10: 	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
```

- [ ] SHELL排查结论：________；对应证据：________。

### L11 TFmini I²C从SMBus寄存器改为原始整帧事务

- **提交／部署边界：** `a765be7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/i2c.cpp:113](../src/location/src/i2c.cpp)；[src/location/src/i2c.cpp:121](../src/location/src/i2c.cpp)；[src/location/src/tf_mini_i2c.cpp:43](../src/location/src/tf_mini_i2c.cpp)。
- **可逐行对照原件：** [src/location/src/i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/i2c.cpp)；[src/location/src/i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/i2c.cpp)；[src/location/src/tf_mini_i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/tf_mini_i2c.cpp)。
- **修改前／来源：** 原包write_register_block带寄存器／长度，随后逐个read寄存器；实机tfmini_test持续-1。
- **我实际修改：** 原样写5A 05 00 01 60后一次read九字节；完整长度才成功，EINTR重试，不拼接部分I²C事务。
- **为什么这样改／要解决什么：** 现场直接原始事务读到59 59有效距离，而旧调用封装不符合实际协议。
- **依据与来源限制：** [E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)日志§3／§5原始帧59 59 0C 00 79 0E 20 09 6E及25项定位／协议测试；为我的驱动修复，非学长代码。
- **已做验证及其范围：** 真实I²C读距约12cm和校验通过；长期零误码未通过。
- **SHELL排查点：** 检查事务间隔、400kHz总线和共享PCA的时序，不将通信成功等同测高准确。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
112:
113: void I2C::write_bytes(const uint8 *data, std::size_t length)
114: {
115: 	ssize_t count;
```

- [ ] SHELL排查结论：________；对应证据：________。

### L12 I²C测距1ms等待、盲区／信号强度过滤

- **提交／部署边界：** `a765be7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/src/tf_mini_i2c.cpp:60](../src/location/src/tf_mini_i2c.cpp)；[src/location/src/tf_mini_i2c.cpp:122](../src/location/src/tf_mini_i2c.cpp)。
- **可逐行对照原件：** [src/location/src/tf_mini_i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/tf_mini_i2c.cpp)；[src/location/src/tf_mini_i2c.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/location/src/tf_mini_i2c.cpp)。
- **修改前／来源：** pack测距命令后立即读，仅距离0无效，不按flux过滤。
- **我实际修改：** 命令至read等1ms；dist<10cm或>1200cm、flux<100或65535拒收；之后L03仍限制有效0.05–6m。
- **为什么这样改／要解决什么：** 现场立即读偶发首字节2C；我加入等待与弱信号／过曝筛查，避免坏测量刷新高度。
- **依据与来源限制：** [E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)现场1ms／5ms对照与协议测试；1ms不是零错帧保证，5ms也未根治。质量阈值是我采用的协议筛选。
- **已做验证及其范围：** 1ms后200帧199有效；60秒5ms1196/1200有效，真实失败保留。
- **SHELL排查点：** 12cm基准离10cm拒收下限很近，起落架损坏或倾斜可能频繁拒收；这一点不能从静止融合成功排除。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
59: 		// immediate reads. A bounded 1 ms command-to-read gap is bench-verified.
60: 		usleep(1000);
61:
62: 	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
```

- [ ] SHELL排查结论：________；对应证据：________。

### L13 RK I²C总线映射及舵机入口门控

- **提交／部署边界：** `18cd805；a765be7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/launch/location.launch:28](../src/location/launch/location.launch)；[src/servo_control/src/servo_controller_node.cpp:23](../src/servo_control/src/servo_controller_node.cpp)；[src/servo_control/launch/servo_bench.launch:32](../src/servo_control/launch/servo_bench.launch)。
- **可逐行对照原件：** [src/location/launch/location.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/launch/location.launch)；[src/servo_control/src/servo_controller_node.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/servo_control/src/servo_controller_node.cpp)；[src/servo_control/launch/servo_bench.launch 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/servo_control/launch/servo_bench.launch)。
- **修改前／来源：** pack／Nano默认I²C1；舵机角度与动作插值来自原包／既有基线。
- **我实际修改：** 定位和舵机相关默认总线改6，launch透传bus；导航真实舵机必须同时通过enable_actuation与use_real_servos。
- **为什么这样改／要解决什么：** 适配RK BSP排针实际I²C6，并在禁动联查避免构造真实舵机节点。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)迁移记录与[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)接线／i2c读回；总线选择由我按板端映射实现，实际接线由SHELL确认。
- **已做验证及其范围：** 10月27日共享I²C6两设备通信及当时空仓动作；当前舵机已损坏，不能复用当时验收。
- **SHELL排查点：** 总线是接口配置非飞行控制律，但选错会使高度缺失；不能因此把继承角度／servo_debug整文件算成我新写。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
27:   <!-- I2C6 / 0x10 real ranging verified 2026-10-27; height geometry stays uncalibrated. -->
28:   <arg name="bus" default="6" />
29:   <arg name="address" default="16" />
30:   <arg name="serial_port" default="/dev/ttyUSB0" />
```

- [ ] SHELL排查结论：________；对应证据：________。

### L14 高度配置0.12m、零杆臂近似与默认加载

- **提交／部署边界：** `a765be7；8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/param/rk_height_reference_20261027.yaml:5](../src/location/param/rk_height_reference_20261027.yaml)；[src/location/launch/location.launch:11](../src/location/launch/location.launch)；[tools/uav_start.py:183](../tools/uav_start.py)。
- **可逐行对照原件：** [src/location/launch/location.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/launch/location.launch)。
- **修改前／来源：** pack默认离地0.09m；18cd805默认height_reference_calibrated=false。
- **我实际修改：** 保存h0=0.12m、标定true、offset全零及beam=(0,0,-1)专用yaml；preflight和统一A入口默认加载该profile，通用无profile仍不标定。
- **为什么这样改／要解决什么：** 按SHELL尺量12±1cm／垂直下视及DDL接受厘米级误差，使用80帧投影零点0.119966659m并取整，避免每次手输。
- **依据与来源限制：** [E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)日志§8–10与80帧真实记录。这是SHELL近似授权＋我的参数落地，不是学长0.09m原值，也不是精密外参标定。
- **已做验证及其范围：** 两轮真实静止融合／17项联合通过，无飞行动态高度对照。
- **SHELL排查点：** 机械损坏后旧支撑高度和beam安装可能已变化；旧profile仍被默认加载是当前必须审查的配置风险，不能把calibrated=true当永久有效。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
4: # Re-zero if support height or sensor installation changes.
5: height_reference_calibrated: true
6: tf_mini_height: 0.12
7: sensor_offset_x: 0.0
```

- [ ] SHELL排查结论：________；对应证据：________。

## 5. 任务、坐标、控制与DWA

### C01 导航三动作开关与默认禁动

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:115](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/launch/navigation_2d.launch:8](../src/navigation_2d/launch/navigation_2d.launch)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/launch/navigation_2d.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/launch/navigation_2d.launch)。
- **修改前／来源：** pack默认auto_start=true、use_real_servos=true，无enable_actuation总门。
- **我实际修改：** 默认三开关false；控制输出／模式请求／解锁／GPIO／move_base goal及析构取消按动作权限约束，禁动模式只运行推理和readiness。
- **为什么这样改／要解决什么：** 真实设备验证时防止启动默认launch就执行飞行／舵机；正式任务必须显式打开。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)原launch前10行及[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)授权和迁移日志；默认禁动是我的安全实现，非原包。
- **已做验证及其范围：** 模拟任务禁动用例及真实联合零设定值／GPIO；不覆盖正式动作组合飞行。
- **SHELL排查点：** 配置没打开会拒绝任务，打开则可能真实动作。需记录实际命令而不能只看源码默认。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
114:   nh_private_.param("auto_start", auto_start_, false);
115:   nh_private_.param("enable_actuation", enable_actuation_, false);
116:   nh_private_.param("pose_timeout", pose_timeout_, 0.5);
117:   nh_private_.param("state_timeout", state_timeout_, 3.0);
```

- [ ] SHELL排查结论：________；对应证据：________。

### C02 任务输入校验、readiness与超时门限

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:340](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:483](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:116](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack收到pose就has_pose，无相同的持续新鲜度／readiness合取。
- **我实际修改：** local pose有限／四元数有效且0.5s新鲜，FCU3s且connected，TF0.5s、同步差0.2s、planner0.5s；周期合取模型／源图／推理／pose／FCU／TF，各项不满足阻断。
- **为什么这样改／要解决什么：** 避免陈旧pose／离线FCU或失效模型仍推进任务，给启动和解锁入口可读的依据。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)任务回调与[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)保护授权；这些时限、JSON键和组合是我选定，非学长明确指定。
- **已做验证及其范围：** gtest＋十项模拟任务；真实arm --check缺输入拒绝。
- **SHELL排查点：** tf_fresh实际代表map与local可对齐，不等于仅SLAM TF新鲜；严格合取可能阻断起飞，应逐键定位而非把所有拒绝归因飞控。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
339:
340: bool MoveBaseMission::poseFresh() const
341: {
342:   const geometry_msgs::PoseStamped pose = poseSnapshot();
```

- [ ] SHELL排查结论：________；对应证据：________。

### C03 动态估计map到PX4 local的二维刚体对齐

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:354](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/include/dwa_control_math.h:66](../src/navigation_2d/include/dwa_control_math.h)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack默认map／local可直接共用，没有用同一机体的两份观测求对齐。
- **我实际修改：** 每个任务tick（约10Hz）取map→base_link和PX4 local pose；要求两stamp差≤0.2s且新鲜。局部点p_local=body_local+R(yaw_local-yaw_map)*(p_map-body_map)。
- **为什么这样改／要解决什么：** 我认为SLAM map与飞控local的原点／航向不能默认完全相同；用于固定救助点及goal航向转换。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)任务实现无该函数；H中的公式及[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)坐标修正记录。公式和动态重估办法由我新增。
- **已做验证及其范围：** 合成平移／90°旋转／非零原点数学测试；没有实机定向平移／转动验收。
- **SHELL排查点：** 动态漂移、时间差和SLAM回环可能让对齐变化；它不是经过真实外参标定的固定变换。优先看实际两路yaw／TF及goal取消重发。

```text
delta_yaw = local_body.yaw - map_body.yaw;
p_local = local_body.xy + R(delta_yaw) * (p_map - map_body.xy);
```

- [ ] SHELL排查结论：________；对应证据：________。

### C04 DWA机体系速度按实际PX4 local yaw旋转

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:609](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/include/dwa_control_math.h:52](../src/navigation_2d/include/dwa_control_math.h)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack PlannerVelCallback注释承认假设yaw≈0、body≈odom，把cmd_vel XY直接当local XY。pack已经读取PX4 local yaw来保持航向。
- **我实际修改：** 保留DWA cmd为body FLU，控制时用当前local yaw执行R(yaw)*v_body后发MAVROS local；没有自行再做ENU→NED翻轴。
- **为什么这样改／要解决什么：** 消除非零航向下将前向速度当作固定世界轴速度的假设；这是我的算法修正。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)任务原文PlannerVelCallback约247行与当前H；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)日志记录“机体系速度按实际yaw转飞控本地系”。原包没有此旋转，不能归给学长。
- **已做验证及其范围：** 数学0°／90°／非零角测试和模拟任务；未做真实“机头前推在local应往哪边”的物理核验。
- **SHELL排查点：** 本项与local yaw定义／DWA实际输出frame的假设直接相关，属于优先审查区域；数学公式自洽不能证明设备轴向一致。

```text
vx_local = cos(yaw_local)*vx_body - sin(yaw_local)*vy_body;
vy_local = sin(yaw_local)*vx_body + cos(yaw_local)*vy_body;
```

- [ ] SHELL排查结论：________；对应证据：________。

### C05 move_base目标航向与固定救助点转换

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:1385](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:1229](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack发送goal时把hold_yaw直接作为map航向；固定救助点fieldToMap后直接赋local XY。
- **我实际修改：** goal yaw_map=hold_yaw-(yaw_local-yaw_map)；固定救助坐标先fieldToMap，再经C03变换到local。
- **为什么这样改／要解决什么：** 让地图goal朝向对应飞控保持航向，让固定救助位置不混用地图和飞控原点。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) sendGoal／onConfirmed与当前代码diff；实现由我推导。fieldToMap的R(-field_yaw)本身来自原包，见继承清单。
- **已做验证及其范围：** 坐标数学和模拟goal用例；未真实验证整场路线和航向。
- **SHELL排查点：** DWA角速度配置为0，目标航向容差是否能与实际保持航向匹配须联合审查；固定救助点仅对应任务分支，不影响全部航点XY。

```text
goal_yaw_map = hold_yaw_local - (local_body.yaw - map_body.yaw);
```

- [ ] SHELL排查结论：________；对应证据：________。

### C06 二维速度由分量截断改为模长限幅

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/include/dwa_control_math.h:39](../src/navigation_2d/include/dwa_control_math.h)；[src/navigation_2d/src/shoot_with_move_base.cpp:589](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack将vx／vy各自限幅max_vel，斜向合速度可到sqrt(2)*max_vel。
- **我实际修改：** DWA输入及HOLD的P控制水平速度统一按hypot缩放到max_vel（默认0.35m/s），非有限输入拒收。
- **为什么这样改／要解决什么：** 保证水平合速度上限与参数含义一致，而不是两个轴各自都达到上限。
- **依据与来源限制：** 原分量clamp与H.limitPlanar对照；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)坐标测试。此限幅选择来自我。
- **已做验证及其范围：** 对角线、零速、异常值数学测试；没有实际加速度／闭环稳定性测试。
- **SHELL排查点：** 它改变斜向响应和到点时间；P增益1.2／最大0.35的原始值来自包，不能把这些参数也算作我新调。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
38:
39: inline bool limitPlanar(const Vec2 &input, double maximum, Vec2 &output)
40: {
41:   output = Vec2();
```

- [ ] SHELL排查结论：________；对应证据：________。

### C07 DWA速度命令过期转零速保高

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:323](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:616](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack把最新cmd_vel缓存后持续使用，没有同等planner TTL。
- **我实际修改：** 记录cmd_vel收到时刻；超过0.5s或转换失败，则发送XY零速度＋fly_height的z位置保持，状态HOLD_PLANNER_STALE。
- **为什么这样改／要解决什么：** 规划器退出／断流时避免无限执行最后一条水平速度。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) PlannerVelCallback／ControlLoop和当前diff；无学长指定0.5s依据。
- **已做验证及其范围：** 数学新鲜度和隔离任务断流验证；真实运动中DWA停流节拍未验。
- **SHELL排查点：** cmd_vel无header，记录的是接收时刻而非规划源时刻；无线延迟或队列旧指令不能由此完全识别。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
322:   planner_velxy_posz_.y = bounded.y;
323:   planner_command_stamp_ = ros::Time::now();
324: }
325:
```

- [ ] SHELL排查结论：________；对应证据：________。

### C08 TAKEOFF捕获当前位置和重新采样保持航向

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:980](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack首次pose就设hold_yaw；beginTakeoff把目标XY设0,0，未在进入起飞时重采yaw。
- **我实际修改：** 进入TAKEOFF捕获当前local XY为takeoff_origin并重采local yaw；起飞期间XY保持该点。
- **为什么这样改／要解决什么：** 避免飞控local原点不正好在机身当前位置时，起飞同时飞向0,0；避免沿用很早第一帧航向。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) beginTakeoff／PoseCallback与当前函数；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)当日记录“起飞捕获实际XY”。重新采样是我加的。
- **已做验证及其范围：** 模拟初始非零XY／航向用例；实际早切／晚切OFFBOARD时触发时刻未通过物理飞行对照。
- **SHELL排查点：** QGC先AUTO.TAKEOFF后切OFFBOARD会在空中捕获起点和航向；实际时机与原包不同，需和模式事件一起检查。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
979:
980: void MoveBaseMission::beginTakeoff()
981: {
982:   const geometry_msgs::PoseStamped pose = poseSnapshot();
```

- [ ] SHELL排查结论：________；对应证据：________。

### C09 起飞必须到高，否则20s中止

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/include/dwa_mission_policy.h:29](../src/navigation_2d/include/dwa_mission_policy.h)；[src/navigation_2d/src/shoot_with_move_base.cpp:752](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack原TAKEOFF等待到高，也有超时推进分支（原文见MP）；我不保留未达高度照常转水平巡航。
- **我实际修改：** elapsed>2s且atHeight才Reached；默认20s仍未到高就abortMission并停止任务输出。
- **为什么这样改／要解决什么：** 避免高度没建立却开始水平路线。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) TAKEOFF分支原来已有20s常数；我将它参数化并把超时继续改为中止。POL和[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)十项模拟任务“起飞失败”给出实现依据，20s不是我新调的数值。
- **已做验证及其范围：** 模拟未到高超时／到高通过；未进行真实起飞验收。
- **SHELL排查点：** 中止后不自动降落或切模式，可能由PX4自身失控保护处理；不要把它叫自动安全落地。L02暂停会影响20s计时。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
28:
29: inline TakeoffProgress takeoffProgress(double elapsed, bool at_height, double timeout)
30: {
31:   if (at_height && elapsed > 2.0)
```

- [ ] SHELL排查结论：________；对应证据：________。

### C10 关键输入故障时取消导航、保持故障前位置

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:402](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:569](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack无这套跨阶段输入故障阻塞状态。
- **我实际修改：** 首次故障捕获新鲜local位置、取消活动goal、清DWA命令及hits；有新鲜pose／FCU时发故障位置保持；pose／FCU过期则返回、不再伪造设定值。
- **为什么这样改／要解决什么：** 我希望缺相机／推理／TF时先暂停任务，避免用旧目标继续水平飞行。
- **依据与来源限制：** M.blockMission／ControlLoop与[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)模拟故障测试；[E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md)晚切架次确有反复TF_STALE与取消重发。
- **已做验证及其范围：** 合成缺测／恢复和真实静止拒收；未在运动飞行验证故障保持瞬态。
- **SHELL排查点：** 优先查反复阻塞—恢复造成goal取消重发及位置保持目标跳变；过期时断设定值与有pose时保持是两种不同分支。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
401:
402: void MoveBaseMission::blockMission(const std::string &reason, const ros::Time &now)
403: {
404:   if (!mission_abort_reason_.empty())
```

- [ ] SHELL排查结论：________；对应证据：________。

### C11 输入恢复后补偿业务计时、自动恢复任务

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:466](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack没有对应关键输入暂停，业务超时按墙钟持续走。
- **我实际修改：** 恢复后给phase_start／goal_sent／drop_cmd_time／land_stage_time加暂停时长；清fault_hold及planner时刻，允许继续／重发goal。
- **为什么这样改／要解决什么：** 避免故障暂停期间消耗10s下降或ACK计时，恢复立刻触发超时投放／爬升。
- **依据与来源限制：** M.resumeMission注释及[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)测试；此恢复策略由我设计。
- **已做验证及其范围：** 隔离任务；未覆盖真实高频间歇TF的全部恢复瞬态。
- **SHELL排查点：** 输入时好时坏可能延长起飞／任务超时并反复重发目标；不能把20s或10s理解成任何情况下的绝对墙钟上限。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
465:
466: void MoveBaseMission::resumeMission(const ros::Time &now)
467: {
468:   if (!runtime_blocked_)
```

- [ ] SHELL排查结论：________；对应证据：________。

### C12 人工接管／解除武装中止锁存

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:230](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:449](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/include/dwa_mission_policy.h:11](../src/navigation_2d/include/dwa_mission_policy.h)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack没有目前所有活动阶段统一的authority锁存机制。
- **我实际修改：** 活动阶段要求armed且OFFBOARD；新鲜state回调和两个定时环都检查；丢权限后取消goal、清命令和投放授权、control IDLE且不自动重启。
- **为什么这样改／要解决什么：** 人工切模式或锁桨后不允许任务悄悄恢复控制／投放。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)状态／控制分支与POL；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)十项模拟“接管／解除武装锁存”。规则由我写。
- **已做验证及其范围：** 模拟服务及状态消息测试；没有真实空中接管演练。
- **SHELL排查点：** 首次进入任务后短暂的模式过渡也可能锁存；需按事故state时间核对。WAIT_FCU不等于已有飞行权限。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
229:
230: void MoveBaseMission::StateCallback(const mavros_msgs::State &msg)
231: {
232:   std::lock_guard<std::recursive_mutex> runtime_lock(runtime_mutex_);
```

- [ ] SHELL排查结论：________；对应证据：________。

### C13 着陆阶段权限例外及停止竞争设定值

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/include/dwa_mission_policy.h:18](../src/navigation_2d/include/dwa_mission_policy.h)；[src/navigation_2d/src/shoot_with_move_base.cpp:551](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack已有POSCTL→AUTO.LAND的着陆流程；该流程本身不是我新增。
- **我实际修改：** 为自己请求的LAND过渡设置允许模式集合；确认POSCTL且0.5s后才请求AUTO.LAND；已离OFFBOARD不再发送竞争设定值；land_stage≥2新鲜解除武装可结束，即使相机后续丢失。
- **为什么这样改／要解决什么：** 防止C12把自己的正常着陆切换当人工接管；避免ROS设定值与飞控降落竞争。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)原LAND分支、POL.landingAuthority与M；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)模拟着陆用例。
- **已做验证及其范围：** 隔离状态／服务测试；真实自动着陆未验。
- **SHELL排查点：** 没有因此新增失败自动回退或模式请求可靠性保证；这是原流程周围的权限修正。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
17: // Old mode is allowed while the request is in flight; manual modes never are.
18: inline bool landingAuthority(bool armed, const std::string &mode, int land_stage)
19: {
20:   if (!armed)
```

- [ ] SHELL排查结论：________；对应证据：________。

### C14 删除未识别兜底投放、增加分类授权令牌

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:1038](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:1271](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/include/dwa_mission_policy.h:43](../src/navigation_2d/include/dwa_mission_policy.h)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack危险品路线耗尽可兜底投放；backend失效也被设计成可飞固定投放点。
- **我实际修改：** 危险品只允许explosive／flammable／toxic且匹配本轮目标；路线耗尽只转人员巡检不消防投放；beginDrop必须来自本次DETECT确认的Fire／Rescue授权；人员仅person。
- **为什么这样改／要解决什么：** 避免“没识别出来”被当成投放依据；模型失败阻断为SHELL明确要求，删除盲投是按修不合理任务分支落地。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) nextLeg／onConfirmed／beginDrop；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)授权及“无目标不投放”模拟结果。具体令牌状态机由我实现。
- **已做验证及其范围：** 真实模型＋模拟动作服务器通过；不代表识别业务精度合格或实物释放成功。
- **SHELL排查点：** 原10s下降超时仍投放、ACK超时爬升按SHELL要求保留，不能误列成我的新策略。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
1037:       ROS_WARN("[mission] no matching hazard at any card; no fire drop, continuing person survey");
1038:       setStatus("HAZARD_NOT_FOUND_NO_DROP");
1039:       holdCurrent();
1040:       startTask(Task::PERSON);
```

- [ ] SHELL排查结论：________；对应证据：________。

### C15 独立推理结果确认、跳过旧缓存与6秒窗口

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:1109](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:165](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/include/latest_result_cache.h:10](../src/navigation_2d/include/latest_result_cache.h)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** pack检测环直接infer并在短窗口累计命中；慢RKNN与任务tick相互耦合。
- **我实际修改：** 每个结果带序号只消费一次；进入DETECT先跳过已缓存结果；5次确认保留，窗口从原值改6s，检测总timeout仍默认8s；识别候选限定实际任务类别。
- **为什么这样改／要解决什么：** 实板原四类模型约650–666ms／次，不能按高频tick把同一次结果重复计作多次确认；短窗口难积够5次。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)真实12图耗时、原任务DETECT窗口与当前pollAsync；窗口6s由我按延迟调整，不是学长指定。
- **已做验证及其范围：** 结果序号gtest／真实模型＋模拟任务；不是盲测识别精度验收。
- **SHELL排查点：** 确认关联仍主要按命中窗口／类别，不能宣称严格同一物体跟踪；旧图源时间另由V06约束。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
1108:
1109: void MoveBaseMission::beginDetect()
1110: {
1111:   phase_ = Phase::DETECT;
```

- [ ] SHELL排查结论：________；对应证据：________。

### C16 任务共享状态递归锁与独立推理并发组织

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:232](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp:1552](../src/navigation_2d/src/shoot_with_move_base.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)。
- **修改前／来源：** 原包的任务tick同步推理，共享状态锁范围与现在不同。
- **我实际修改：** 状态／pose／planner／ACK／target／控制／tick在统一recursive_mutex下串行关键状态；推理独立线程；保留单独pose／state快照锁。main六线程spinner来自原包，我仅在try/catch中继续使用。
- **为什么这样改／要解决什么：** 在慢推理独立运行后避免回调与控制／投放状态竞争，让接管锁存及时生效。
- **依据与来源限制：** M各回调与mission_with_move_base.h；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)模拟接管与并发结果测试。锁结构和工作线程是我的实现，六线程数不是。
- **已做验证及其范围：** 地面／模拟时序测试；没有实飞最坏调度和锁等待测量。
- **SHELL排查点：** 锁内action调用、TF查询或日志延迟可能影响控制环；不能凭定时器配置50Hz就断言真实发送恒定50Hz。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
231: {
232:   std::lock_guard<std::recursive_mutex> runtime_lock(runtime_mutex_);
233:   bool was_armed;
234:   std::string old_mode;
```

- [ ] SHELL排查结论：________；对应证据：________。

### C17 飞控输出有限数过滤、时间戳与服务响应诊断

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/flight_control.cpp:24](../src/navigation_2d/src/flight_control.cpp)；[src/navigation_2d/src/flight_control.cpp:133](../src/navigation_2d/src/flight_control.cpp)；[src/navigation_2d/src/flight_control.cpp:89](../src/navigation_2d/src/flight_control.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/flight_control.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/navigation_2d/src/flight_control.cpp)；[src/navigation_2d/src/flight_control.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/navigation_2d/src/flight_control.cpp)；[src/navigation_2d/src/flight_control.cpp 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/navigation_2d/src/flight_control.cpp)。
- **修改前／来源：** pack已修默认构造委托和publisher有效性；这两点属于学长。部分原有解锁响应诊断被pack删掉。
- **我实际修改：** 按type_mask仅检查参与控制的position／velocity／acc／yaw／yaw_rate为有限数；统一发送前stamp=now；PoseStamped设置单位姿态；保留／恢复解锁拒绝和mode_sent日志。
- **为什么这样改／要解决什么：** 拒绝NaN/Inf控制指令，保证控制消息有发送时间；保留PX4拒绝原因。
- **依据与来源限制：** pack flight_control.cpp与当前diff；有限数、stamp和Pose姿态是我加的，诊断部分继承旧基线。
- **已做验证及其范围：** 坐标／输出合成测试与编译；未真实解锁。
- **SHELL排查点：** 不改变MAVROS自身ENU/NED转换，也不绕过PX4解锁门槛；丢帧策略仍可能触发OFFBOARD失流，应看实际输出证据。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
23:
24: void publish_raw(ros::Publisher &publisher, mavros_msgs::PositionTarget &message)
25: {
26:     if (!setpoint_publisher_ready(publisher, "/mavros/setpoint_raw/local"))
```

- [ ] SHELL排查结论：________；对应证据：________。

### C18 给DWA显式指定PX4里程计话题

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/launch/move_base.launch:10](../src/navigation_2d/launch/move_base.launch)；[src/navigation_2d/launch/navigation_2d.launch:26](../src/navigation_2d/launch/navigation_2d.launch)。
- **可逐行对照原件：** [src/navigation_2d/launch/move_base.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/launch/move_base.launch)；[src/navigation_2d/launch/navigation_2d.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/launch/navigation_2d.launch)。
- **修改前／来源：** pack move_base.launch没有该显式参数，默认/odom在此RK图中不作为速度反馈源。
- **我实际修改：** 新增可透传odom_topic，默认/mavros/local_position/odom给DWA。
- **为什么这样改／要解决什么：** 让DWA速度采样／动态窗口使用真实飞控速度输入，避免等待错误默认话题。
- **依据与来源限制：** 原pack和当前move_base.launch两行diff；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)实际联合参数核对。选择由我做。
- **已做验证及其范围：** 真实里程计可用及合成0°/90°DWA场景；缺少真实飞行时frame_id／child_frame_id与速度轴的闭合核对。
- **SHELL排查点：** 必须区分odom pose所在frame与twist所在frame；不能因为话题有数据就认定满足DWA全部语义。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
9:     <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
10:     <param name="DWAPlannerROS/odom_topic" value="$(arg odom_topic)" />
11:     <rosparam file="$(find navigation_2d)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
12:     <rosparam file="$(find navigation_2d)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
```

- [ ] SHELL排查结论：________；对应证据：________。

### C19 降低DWA障碍软代价权重0.15→0.02

- **提交／部署边界：** `a965a7c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/param/dwa_local_planner_params.yaml:42](../src/navigation_2d/param/dwa_local_planner_params.yaml)。
- **可逐行对照原件：** [src/navigation_2d/param/dwa_local_planner_params.yaml 原包](../docs/验证记录/非学长算法审查_2026-10-28/原包补充文本/src/navigation_2d/param/dwa_local_planner_params.yaml)。
- **修改前／来源：** pack occdist_scale=0.15；max/min速度、零角速度、加速度、goal容差、sim_time、采样数、footprint、inflation等其余生产数值来自pack。
- **我实际修改：** 只把生产occdist_scale改0.02；尝试过的forward_point_distance=0未采纳，最终仍0.2。
- **为什么这样改／要解决什么：** 修正完整时间积分探针后，同一合成障碍场景0.15振荡／近停，单改0.02到点。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)最终三场景JSON和43–46号原始日志、原参数／工具积分复核。此项完全是我的调参，依据是合成闭环，不是实机避障。
- **已做验证及其范围：** 真实RK运行Navfn/DWA＋合成地图／动力学到点；候选footprint最小间距约0.05269m。
- **SHELL排查点：** 降低权重改变软避障偏好；硬碰撞拒收保留不能替代真实桨叶包络。应审查真实墙面距离与机体尺寸，不能把合成到点写成实飞安全。

```text
- occdist_scale: 0.15
+ occdist_scale: 0.02
```

- [ ] SHELL排查结论：________；对应证据：________。

## 6. 视觉推理、解码与识别时序

### V01 RKNN1.7.5维度反序及五维输出转置

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:456](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/include/yolo_tensor_layout.h:69](../src/navigation_2d/include/yolo_tensor_layout.h)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack按dims[1..3]视为CHW，只接受普通4维头；收到的模型实际有三个[1,3,H,W,9]头。
- **我实际修改：** 旧API查询dims反序还原；将[1,3,H,W,9]实质转置为[1,27,H,W]，不是直接reshape；三个网格52／26／13。
- **为什么这样改／要解决什么：** 使学长原模型能在当前1.7.5运行，避免错误维度／错位logits造成无框或错误框。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)模型结构与RK真实metadata、[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md) 01–12号原始张量和12图CPU/NPU比较。适配逻辑是我写的，模型字节来自学长。
- **已做验证及其范围：** 13项张量／解码／序号／新鲜度gtest，12固定图真实NPU对照；量化数值不逐位相同。
- **SHELL排查点：** 适配只针对已核模型合同，不能泛化所有RKNN版本；误报两张干扰图仍存在，非业务精度通过。

```text
logical_shape = reverse(rknn_attr.dims);
head[(anchor*K+field)*H*W+pixel] = raw[(anchor*H*W+pixel)*K+field];
```

- [ ] SHELL排查结论：________；对应证据：________。

### V02 OpenCV ONNX五维头和严格输出布局支持

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:1031](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp:1116](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/include/yolo_tensor_layout.h:32](../src/navigation_2d/include/yolo_tensor_layout.h)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack ONNX只处理m.dims==4，五维头被跳过。
- **我实际修改：** ONNX也使用同一4／5维布局归一化；验证CV_32F、continuous、batch=1、元素乘积、grid／stride和三头完整性。
- **为什么这样改／要解决什么：** 让CPU参考路线可正确读取同一个学长模型，用于RKNN适配对拍。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) ONNX checker的真实shape和DP.runYoloInference；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)独立CPU/NPU结果。
- **已做验证及其范围：** 固定图及张量测试；不是本机重新训练，也没有改变模型权重。
- **SHELL排查点：** 正式部署默认RKNN，ONNX是可切换／对照路径；不能拿CPU正确证明全部NPU数值精度。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
1030:
1031: bool Detect::runYoloInference(const cv::Mat &canvas, std::vector<Tensor> &tensors)
1032: {
1033: #ifdef USE_RKNN
```

- [ ] SHELL排查结论：________；对应证据：________。

### V03 模型输入输出及阈值配置严格验证

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:131](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp:124](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/include/yolo_tensor_layout.h:80](../src/navigation_2d/include/yolo_tensor_layout.h)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack部分配置只warn，允许多于三输出或不完整布局继续。
- **我实际修改：** 要求18个正anchors、3个不同正stride、恰3输出、合法输入shape／格式／大小和有限logits；conf/NMS在[0,1]、时限正数；输出buffer长度与n_elems一致并检查release结果。
- **为什么这样改／要解决什么：** 不在模型合同不明时猜测解码，以免错框参与投放。
- **依据与来源限制：** D.loadParams／initRknn／runYoloInference及V01模型真实元信息；错误输入边界是我的补充。
- **已做验证及其范围：** 张量错误输入gtest和真实模型启动；没有覆盖全部SDK失败类型。
- **SHELL排查点：** 可能使原先“能启动但无框”的模型现在直接失败；应保留明确错误而非用ready字样推断成功。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
130:   {
131:     ROS_ERROR("Detect: model requires three distinct positive strides");
132:     return;
133:   }
```

- [ ] SHELL排查结论：________；对应证据：________。

### V04 模型文件名／后端构建与标签合同落地

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:104](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/CMakeLists.txt:38](../src/navigation_2d/CMakeLists.txt)；[src/navigation_2d/launch/detect.launch:23](../src/navigation_2d/launch/detect.launch)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/CMakeLists.txt 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/CMakeLists.txt)；[src/navigation_2d/launch/detect.launch 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/launch/detect.launch)。
- **修改前／来源：** pack部分入口仍指不存在gc-yolo-raw-416；CMake预置空RKNN_LIBRARY可阻断find_library。
- **我实际修改：** 默认实际best.rknn／best.onnx；模型与labels保持原件；修RKNN头库查找／后端构建和四类调用，缺所需后端不得伪装成功。
- **为什么这样改／要解决什么：** 修复文件名与ABI依赖，按SHELL只用四类产物的决定部署。
- **依据与来源限制：** [E15](../docs/项目资料/学长新增NPU与ROS包评估_2026-10-25.md)评估§4.1、[E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)构建声明原文与[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)源码／模型哈希。选定现有SDK与部署路径由我落地。
- **已做验证及其范围：** RK指定目标构建及214文件一致性；不称历史EGO／Gazebo／SSD完整重建。
- **SHELL排查点：** 构建声明也影响实际启用代码区域，不能从本地文件存在推断二进制启用了哪个后端。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
103:   ready_ = false;
104:   nh_.param<std::string>("model_file", model_file_, "best.rknn");
105:   nh_.param<std::string>("labels_file", labels_file_, "labels.txt");
106:   nh_.param<std::string>("backend", backend_, "auto");
```

- [ ] SHELL排查结论：________；对应证据：________。

### V05 模型初始化或推理失败阻断／退出

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:42](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/detect.cpp:208](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect_main.cpp:18](../src/navigation_2d/src/detect_main.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect_main.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect_main.cpp)。
- **修改前／来源：** pack允许backend none／未编后端／模型失败后仍按固定点继续任务，检测独立节点只警告。
- **我实际修改：** 任务构造ready或startAsync失败抛异常退出2；独立检测初始化失败退出2、后端失败退出3；推理异常将ready／last_inference_ok置false。
- **为什么这样改／要解决什么：** SHELL明确“模型初始化失败必须阻断”，不让模型不可用被看作没有目标并触发旧兜底策略。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)授权与DP错误分支／当前diff。具体退出码、异常与就绪传播是我实现。
- **已做验证及其范围：** 真实缺模型自然退出及十项模拟；未把所有异常都当空目标。
- **SHELL排查点：** 运行中模型失败会通过C10暂停甚至影响全部任务推进，不仅影响DETECT阶段，需审查故障覆盖范围。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
41:   if (!detect_.ready())
42:     throw std::runtime_error("mission model initialization failed; refusing to start");
43:   if (!detect_.startAsync())
44:     throw std::runtime_error("mission asynchronous inference failed to start");
```

- [ ] SHELL排查结论：________；对应证据：________。

### V06 源图新鲜度、乱序拒收及完成推理年龄

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:344](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/include/detection_freshness.h:9](../src/navigation_2d/include/detection_freshness.h)；[src/navigation_2d/src/detect.cpp:230](../src/navigation_2d/src/detect.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack可把零stamp图改now，hasFrame只看非空；慢推理完成不保留同样源年龄约束。
- **我实际修改：** 不接受零／未来／过期／重复／倒序源图；同时保存ROS时间及接收时年龄＋单调elapsed；源帧默认1s、结果默认3s，完成后仍不能刷新源图年龄。
- **为什么这样改／要解决什么：** 防止旧图／积压帧刚到或慢推理刚完成就被当成新证据。
- **依据与来源限制：** DP.imageCallback与当前freshDetectionSource；[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)“旧图和过期源推理阻断”用例。时限为我的工程配置。
- **已做验证及其范围：** gtest／真实模型模拟飞控；相机旧图与推理过期合成测试。
- **SHELL排查点：** 相机驱动stamp是成功取图后的主机时间，非传感器曝光硬件时刻；不能声称已得到曝光到控制的严格延迟。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
343:
344: void Detect::imageCallback(const sensor_msgs::ImageConstPtr &msg)
345: {
346:   const auto received = std::chrono::steady_clock::now();
```

- [ ] SHELL排查结论：________；对应证据：________。

### V07 异步推理及最新结果缓存

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/src/detect.cpp:244](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp:302](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/include/latest_result_cache.h:25](../src/navigation_2d/include/latest_result_cache.h)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack的任务DETECT调用同步infer，单次约0.66s会占任务tick；已有多线程spinner不能消除共享状态／推理耗时问题。
- **我实际修改：** 独立推理线程只处理新frame_generation；单槽最新结果＋递增序号，try_lock轮询不等待结果锁；启停join／clear并行生命周期。
- **为什么这样改／要解决什么：** 把慢推理与50Hz控制及10Hz任务状态推进分离，不重复推同一帧。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md) 12图耗时及任务测试；D／LatestResultCache为我新增。
- **已做验证及其范围：** 结果缓存gtest、真实模型＋模拟任务；地面控制定时器证据不等同真实发送节拍。
- **SHELL排查点：** 单槽会丢中间结果且推理线程失败可令inferenceReady失效；不能保证所有帧被推理或记录。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
243:
244: bool Detect::startAsync()
245: {
246:   std::lock_guard<std::mutex> lifecycle(worker_mutex_);
```

- [ ] SHELL排查结论：________；对应证据：________。

### V08 图像所有权和绘制副本

- **提交／部署边界：** `18cd805；e4882bd`。18cd805主体曾部署；e4882bd追加C++改动已回退，见E5。
- **当前位置：** [src/navigation_2d/src/detect.cpp:379](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp:1219](../src/navigation_2d/src/detect.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack接收／infer／显示多次clone；绘制和输入共享需要防止互相污染。
- **我实际修改：** 18cd805已用toCvCopy拥有图像，后续读路径用不可变引用计数快照，绘制前clone；e4882bd另外调整推理完成状态的锁范围以便在锁外队列化审计输出。
- **为什么这样改／要解决什么：** 减少锁内整幅复制，避免控制查readiness等待大图拷贝，同时不让标注改写推理源。
- **依据与来源限制：** D.imageCallback／inferImpl／publishCallback在18cd805和e4882bd的逐提交diff；[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)候选回归。
- **已做验证及其范围：** 编译／真实推理／回归；e4882bd C++部分已从RK回退，不能以当前本地实现认定事故运行了该版本。
- **SHELL排查点：** 需比较实际板端detect.cpp版本（本次已保存任务／定位来源，但detect需按历史部署哈希判断）；区分迁移时改动与后来的候选优化。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
378:   // thread from waiting on a full image clone while it checks readiness.
379:   frame_ = cv_ptr->image;
380:   ++frame_generation_;
381:   frame_received_ = received;
```

- [ ] SHELL排查结论：________；对应证据：________。

### V09 YOLOv5解码抽取及数值稳定sigmoid

- **提交／部署边界：** `18cd805`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/navigation_2d/include/yolo_tensor_layout.h:106](../src/navigation_2d/include/yolo_tensor_layout.h)；[src/navigation_2d/include/yolo_tensor_layout.h:113](../src/navigation_2d/include/yolo_tensor_layout.h)；[src/navigation_2d/src/detect.cpp:978](../src/navigation_2d/src/detect.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)。
- **修改前／来源：** pack已有YOLOv5的2*sigmoid-0.5网格、平方宽高、anchors、逐类NMS、letterbox还原和像素到机体投影。
- **我实际修改：** 将v5 box解码抽到无ROS helper，sigmoid分正负指数以减少溢出；公式保持原样，不是新创网格算法。
- **为什么这样改／要解决什么：** 让同一真实解码公式可单测，并提高极端logits数值稳定性。
- **依据与来源限制：** DP原654附近与当前decodeV5Box；13项张量测试。
- **已做验证及其范围：** 参考解码与固定图对拍；未更改pack已有pixelToBody、camera_yaw=-90等业务投影。
- **SHELL排查点：** 六类离线评估纠错是另一套工具（T04），不能误写为这里从学长代码补入-0.5。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
105:
106: inline float sigmoid(float value)
107: {
108:   if (value >= 0.0f) return 1.0f / (1.0f + std::exp(-value));
```

- [ ] SHELL排查结论：________；对应证据：________。

## 7. 相机、监看、启动、解锁与记录

### R01 usb_cam上游导入后的退出／清理修复

- **提交／部署边界：** `31a9aa0`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/usb_cam/src/usb_cam.cpp:381](../src/usb_cam/src/usb_cam.cpp)；[src/usb_cam/src/usb_cam.cpp:1180](../src/usb_cam/src/usb_cam.cpp)。
- **修改前／来源：** 来源为ros-drivers/usb_cam0.3.7，非学长pack。旧系统同名驱动在停止时出现异常。
- **我实际修改：** 初始化FFmpeg／缓冲指针，集中幂等shutdown，释放后置空，避免析构重复释放及fd泄漏；纳入项目覆盖系统包。
- **为什么这样改／要解决什么：** 真实相机四轮启停失败及退出异常需要可复现修复。
- **依据与来源限制：** [E3](../docs/验证记录/RK外设修复_2026-10-24/README.md)、src/usb_cam/UPSTREAM.md锁定addab4a...；项目修改是我写，整套上游采集算法不是我创作。
- **已做验证及其范围：** 无设备重复shutdown单测，MJPEG／YUYV真实出图后SIGINT多轮正常退出。
- **SHELL排查点：** 生命周期修复和R02后端改写分开审查，不能整文件都归我或整文件都归第三方。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
380:
381: UsbCam::UsbCam()
382:   : is_capturing_(false), pixelformat_(0), monochrome_(false),
383:     io_(IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0), avframe_camera_(NULL),
```

- [ ] SHELL排查结论：________；对应证据：________。

### R02 相机新增MPP硬解＋RGA色彩转换

- **提交／部署边界：** `8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/usb_cam/src/rockchip_mjpeg.cpp:46](../src/usb_cam/src/rockchip_mjpeg.cpp)；[src/usb_cam/src/rockchip_mjpeg.cpp:69](../src/usb_cam/src/rockchip_mjpeg.cpp)；[src/usb_cam/launch/rk_usb_camera.launch:8](../src/usb_cam/launch/rk_usb_camera.launch)。
- **修改前／来源：** 原usb_cam只走FFmpeg软件解码；pack没有该驱动。
- **我实际修改：** 默认RK入口mpp_rga，MJPEG高级任务队列→NV12→RGA RGB8，按实际stride／尺寸／颜色范围，DMA同步及有界500ms等待；FFmpeg仍可显式切换。
- **为什么这样改／要解决什么：** 当时SHELL无线显示反馈彩条／高负载，我加入RK媒体硬件路径减轻解码CPU，并核对同帧颜色。
- **依据与来源限制：** [E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)同一JPEG的60次对照和媒体库锁定版本、此新增cpp；不是学长算法，也不是此次10月28日晚CPU清理才新增。
- **已做验证及其范围：** 同帧颜色／连续解码60次及真实显示／生命周期；不能归因全部历史彩条，也无飞行时硬件忙竞争测量。
- **SHELL排查点：** 超时、NV12 stride／CSC／DMA／队列回收均是新增依赖；相机失败可经V06／C10暂停任务。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
45: }
46: RockchipMjpeg::RockchipMjpeg(int width, int height): state_(new State(width,height)) {
47:   try {
48:     if (width%4 || height%2) throw std::runtime_error("MPP/RGA requires width multiple of 4, even height");
```

- [ ] SHELL排查结论：________；对应证据：________。

### R03 FFmpeg备用路径包所有权、stride和颜色范围修正

- **提交／部署边界：** `8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/usb_cam/src/usb_cam.cpp:500](../src/usb_cam/src/usb_cam.cpp)。
- **修改前／来源：** 原驱动把借用V4L2缓冲交av_packet_from_data，按部分假定像素格式和对齐复制。
- **我实际修改：** 创建自己拥有且补零的AVPacket；取实际解码frame格式／planes／stride／色域，sws缓存转换到连续RGB，验证尺寸；失败返回false。
- **为什么这样改／要解决什么：** 避免缓冲所有权和错误行距／颜色解释导致坏帧与图像异常，保留可用软件回退。
- **依据与来源限制：** 8aa597c的mjpeg2rgb差异与[E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)同帧软件／硬件对照；我的实现，依据上游API及现场问题。
- **已做验证及其范围：** 同帧图片比对、软件路径及相机退出检查；长期运行未全面验证。
- **SHELL排查点：** 正式默认硬件路径，但备用软件仍是可运行算法区域。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
499:
500: bool UsbCam::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
501: {
502:   if (rockchip_) return rockchip_->decode(MJPEG, len, RGB);
```

- [ ] SHELL排查结论：________；对应证据：________。

### R04 坏帧不再换新时间戳发布

- **提交／部署边界：** `8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/usb_cam/src/usb_cam.cpp:1206](../src/usb_cam/src/usb_cam.cpp)；[src/usb_cam/nodes/usb_cam_node.cpp:246](../src/usb_cam/nodes/usb_cam_node.cpp)。
- **修改前／来源：** 原process／grab为void，失败可能仍将旧图当新图发布。
- **我实际修改：** process_image→read_frame→grab_image→take_and_send_image传递成功bool；失败丢帧，成功才stamp=now并发布。
- **为什么这样改／要解决什么：** 防止空／解码失败的旧缓冲被V06当成新采样。
- **依据与来源限制：** 8aa597c调用链diff、[E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)相机测试；由我新增。
- **已做验证及其范围：** 真实出图／失败返回路径和编译；没有硬件曝光时钟对齐。
- **SHELL排查点：** 丢帧可能触发相机新鲜度门；驱动成功后stamp仍是主机接收后时间而非曝光真时刻。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
1205:
1206: bool UsbCam::grab_image(sensor_msgs::Image* msg)
1207: {
1208:   // grab the image
```

- [ ] SHELL排查结论：________；对应证据：________。

### R05 JPEG监看转发与降尺寸／帧率／陈旧拒发

- **提交／部署边界：** `fd7821a；8aa597c；e4882bd；17f2b16`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/scripts/monitor_jpeg.py:12](../src/location/scripts/monitor_jpeg.py)；[src/location/scripts/monitor_jpeg.py:59](../src/location/scripts/monitor_jpeg.py)。
- **修改前／来源：** 旧Nano监看依赖压缩插件；RK缺对应插件，原包无该脚本。
- **我实际修改：** 订阅raw或标注图，单槽latest，OpenCV转BGR、宽≤960等比缩放、JPEG；初版5Hz质量65，后续15Hz质量75，保留源header并拒绝年龄>1s。零stamp／未来>.05s的显式检查在e4882bd候选加入、回退后于17f2b16随整份脚本重新部署。
- **为什么这样改／要解决什么：** 让现有ROS1／Foxglove/Lichtblick在带宽可控时显示图像，避免用now掩盖旧帧。
- **依据与来源限制：** [E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)及Lichtblick教学报告；逐提交脚本历史显示数值变化。这是我写的可选显示进程。
- **已做验证及其范围：** 真实UI图像／地图看到、源header及陈旧帧测试；显示成功不验证导航轴向。
- **SHELL排查点：** 初版无人查看仍订阅整幅RGB，是我引入的多余负载，详见R07；实际编码时仍占CPU。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
11:
12: class Relay(object):
13:     def __init__(self, source, destination, fps, width, quality, archive_destination=None):
14:         self.bridge = CvBridge()
```

- [ ] SHELL排查结论：________；对应证据：________。

### R06 JPEG存档2Hz分流与复用编码

- **提交／部署边界：** `e4882bd；17f2b16`。16:40前已回退；17f2b16整文件部署重新包含此功能，当前依已保存E8证据存在。
- **当前位置：** [src/location/scripts/monitor_jpeg.py:49](../src/location/scripts/monitor_jpeg.py)；[src/location/scripts/monitor_jpeg.py:85](../src/location/scripts/monitor_jpeg.py)。
- **修改前／来源：** 初始记录器订阅显示JPEG激活15FPS编码，探索轮CPU90.65%。
- **我实际修改：** 增加独立log压缩话题；只有存档客户端时≤2Hz编码，有查看者时复用已编码JPEG给存档；保留原header。
- **为什么这样改／要解决什么：** 避免记录本身激活高帧率预览，把自动存档负载收敛。
- **依据与来源限制：** [E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)探索失败及完整／核心CPU对照；e4882bd diff。
- **已做验证及其范围：** 候选记录测试后，此JPEG改动也从RK回退。[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)板端回退核对的旧脚本SHA49cb2d59...；[E8](../docs/验证记录/隔墙WiFi禁动续验_2026-10-28/README.md)保存的monitor_jpeg_before.py确无archive参数。17f2b16以本地候选为基础部署，重新带入2Hz分流并通过archive-only测试。
- **SHELL排查点：** 这是“首次候选→回退→事故后随整文件再部署”，不是一直保留。日志仍禁用且无archive客户端时不编码存档，但代码／话题已存在；不能仅用订阅回收来概括本次文件差异。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
48:         now = rospy.Time.now().to_sec()
49:         archive_due = archive and now-self.archive_last >= .5
50:         # A recorder alone must not activate the 15fps display encoder.
51:         # With a live viewer, reuse the same encoded JPEG at <=2fps.
```

- [ ] SHELL排查结论：________；对应证据：________。

### R07 最后查看者离开后回收整幅图像订阅

- **提交／部署边界：** `17f2b16`。19点续验已备份整文件部署，SHA e47db190...；不是16:40前版本。
- **当前位置：** [src/location/scripts/monitor_jpeg.py:35](../src/location/scripts/monitor_jpeg.py)。
- **修改前／来源：** 我写的旧relay即使不编码仍持续反序列化RGB；无查看者实测relay约45.64%单核CPU。
- **我实际修改：** 无人查看／存档则unregister并clear latest；有客户端才订阅源；重连等待新帧。该补丁以本地含R06的候选文件为基础，整文件部署还同时恢复archive分流和R05严格时间检查，不能说板端仅变更了unregister。
- **为什么这样改／要解决什么：** 修正我新增的可选显示后台进程不必要开销，符合SHELL本轮只清多余负载、不优化主算法的要求。
- **依据与来源限制：** [E8](../docs/验证记录/隔墙WiFi禁动续验_2026-10-28/README.md)修前后分级负载：整机52.90→42.84%、relay45.64→2.96%单核；七项真实ROS生命周期探针。
- **已做验证及其范围：** 隔墙原生UI开／关监看及订阅清理真实通过。
- **SHELL排查点：** 发生于16:40事故取证之后，不能当成事故之前已存在的修复；有人查看时新relay仍约111.61%单核。存档分流重新部署这一点在本次审查补充披露。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
34:         # Only this timer owns the subscription; receive() only owns latest.
35:         if not live and not archive:
36:             if self.subscriber is not None:
37:                 self.subscriber.unregister()
```

- [ ] SHELL排查结论：________；对应证据：________。

### R08 有限MAVROS、只读桥与禁动感知入口

- **提交／部署边界：** `31a9aa0；fd7821a；a765be7；8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [src/location/launch/rk_mavros_bench.launch:17](../src/location/launch/rk_mavros_bench.launch)；[src/location/launch/rk_mavros_vision.launch:19](../src/location/launch/rk_mavros_vision.launch)；[src/location/launch/lichtblick_bridge_readonly.launch:12](../src/location/launch/lichtblick_bridge_readonly.launch)；[src/location/launch/rk_preflight.launch:27](../src/location/launch/rk_preflight.launch)。
- **修改前／来源：** pack主要为正式完整MAVROS＋定位／任务链，没有这些专用只读入口。
- **我实际修改：** bench只允许遥测；vision入口额外允许vision_pose_estimate／sys_time／RC但不含动作插件；preflight三门false；桥capabilities／客户端发布／服务／参数空白名单，显示话题有限。
- **为什么这样改／要解决什么：** 取得感知与融合证据时不开放动作；另给远程查看独立可选入口。
- **依据与来源限制：** [E3](../docs/验证记录/RK外设修复_2026-10-24/README.md)/[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)/[E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)及Lichtblick报告中的实际节点、空能力握手／零动作观测；XML由我写。
- **已做验证及其范围：** 真实未解锁融合／UI及隔离白名单验证；完整正式MAVROS仍由location.launch启动。
- **SHELL排查点：** 只读入口与正式入口插件集合不同，不能把前者通过当后者全部性能／通信保证。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
16:     <rosparam param="plugin_blacklist">['*']</rosparam>
17:     <rosparam param="plugin_whitelist">[sys_status, imu, global_position, local_position]</rosparam>
18:   </node>
19: </launch>
```

- [ ] SHELL排查结论：________；对应证据：________。

### R09 统一ROS环境与SSH路由自动选IP

- **提交／部署边界：** `8aa597c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/uav_env.sh:14](../tools/uav_env.sh)；[tools/install_uav_commands.sh:15](../tools/install_uav_commands.sh)。
- **修改前／来源：** 原来多个终端固定ROS_IP／HOSTNAME／setup行可能互相冲突；原生ROS1电脑回连名称不可解析。
- **我实际修改：** 每次读~/.config/uav/env.sh，统一workspace／ROS Python路径，按当前SSH对端路由src选ROS_IP／11311，清ROS_HOSTNAME／ROS_NAMESPACE；安装时备份并移除已知重复setup／固定IP行。
- **为什么这样改／要解决什么：** 让USB与Wi-Fi下启动／监看共享同一ROS图，避免图像／地图回连错误。
- **依据与来源限制：** [E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)现场原生连接问题及8aa597c脚本；路径／路由选择由我实现，非学长原包。
- **已做验证及其范围：** USB和Wi-Fi实际SSH及原生UI连接；不覆盖未来IP切换／多网卡所有情形。
- **SHELL排查点：** 可能改变接入的ROS master或节点可达地址；只看ssh通不能证明ROS反向数据连接正常。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
13: # Prefer the route to the current SSH peer; no saved wireless or USB address.
14: uav_peer="${SSH_CONNECTION%% *}"
15: uav_route="$(ip -4 route get "${uav_peer:-1.1.1.1}" 2>/dev/null)"
16: uav_ip="$(awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}' <<< "$uav_route")"
```

- [ ] SHELL排查结论：________；对应证据：________。

### R10 A／check就绪合取、设备选择及残留订阅修复

- **提交／部署边界：** `8aa597c；28a178e`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/uav_start.py:87](../tools/uav_start.py)；[tools/uav_start.py:65](../tools/uav_start.py)；[tools/uav_start.py:135](../tools/uav_start.py)。
- **修改前／来源：** pack无统一启动器；我初版wait_ready在返回后遗留rospy相机／地图／TF订阅。
- **我实际修改：** 按by-id唯一设备或环境变量选路径；拒绝已有ROS重叠；等待未解锁connected、IMU／scan／vision／camera、READY、非空地图和map→base_link；28a178e在finally注销所有临时订阅／listener。
- **为什么这样改／要解决什么：** 把实际依赖缺失尽早提示；后修我引入的启动门持续反序列化整幅图像的CPU泄漏。
- **依据与来源限制：** [E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md)独立11341对照：旧相机连接1／51.84%单核，新0／0.60%；真实Wi-Fi缺vision拒绝也释放订阅。
- **已做验证及其范围：** 8项arm测试另见R12；本项合成成功／超时清理及真实缺测路径通过。
- **SHELL排查点：** wait_ready有些门使用收到时间／TF存在性，非严谨源时钟验收；地图只要非空即可。它只能作启动筛查；修复是16:40后部署。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
86:
87: def wait_ready(timeout=65):
88:     import rospy
89:     import tf2_ros
```

- [ ] SHELL排查结论：________；对应证据：________。

### R11 统一启动器正式任务动作选择与子进程退出策略

- **提交／部署边界：** `8aa597c；e4882bd；f253060`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/uav_start.py:139](../tools/uav_start.py)；[tools/uav_start.py:239](../tools/uav_start.py)；[tools/uav_start.py:46](../tools/uav_start.py)。
- **修改前／来源：** pack手工roslaunch并默认自动动作；没有uav a/c/check/mission状态管理。
- **我实际修改：** A起定位／相机／可选JPEG；check/c三门false；mission显式enable_actuation=true与use_real_servos=true、auto_start=false；导航重复启动拒绝；自有子进程组逆序SIGINT→TERM→KILL。A中单子进程退出仅警告，避免显示故障连带停定位；其他模式退出则清整组。
- **为什么这样改／要解决什么：** 简化实际两终端流程、把飞行授权与禁动区分；不让相机／显示异常自动带走A中的定位。
- **依据与来源限制：** [E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)统一入口、[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)启动期写盘隔离、[E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)轻量入口部署与当前代码；策略由我设计。
- **已做验证及其范围：** 正式mission仅参数展开／隔离模拟，真实联查只check；并非正式组合实飞通过。
- **SHELL排查点：** mission会真实启动舵机节点且开始条件满足后飞行；当前已损坏不可执行。A父进程退出仍会停其来源，不能把“某个子进程失败时保留”说成任何SSH断连都保留。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
138:
139: def navigation(args, flight=False):
140:     if flight and args.target not in ('explosive','flammable','toxic'):
141:         raise RuntimeError('mission requires --target explosive|flammable|toxic')
```

- [ ] SHELL排查结论：________；对应证据：________。

### R12 新增uav arm正常解锁门与拒绝说明

- **提交／部署边界：** `f253060；e3a1d97；28a178e`。16:40前已部署初版；28a178e只细化拒绝说明、未放宽门限。
- **当前位置：** [tools/uav_arm.py:27](../tools/uav_arm.py)；[tools/uav_arm.py:99](../tools/uav_arm.py)；[tools/uav_arm.py:137](../tools/uav_arm.py)。
- **修改前／来源：** 学长原包无此脚本；任务已有OFFBOARD＋armed起飞触发及auto_start分支。
- **我实际修改：** 要求先OFFBOARD且未armed、phase0、无abort、三关键参数正确，6项readiness真；pose/vision源龄≤0.3s、state接收≤1.5s、readiness≤.5s；1.5s内≥30条setpoint且最新／最大间隔≤.15s；持续全绿2s；写盘后／发请求前重复核对。--check不发服务，正式只发一次普通CommandBool(true)。
- **为什么这样改／要解决什么：** 13点事故分析发现QGC Takeoff切AUTO.TAKEOFF无法启动此任务，我新增不发Takeoff的显式正常解锁入口，防止操作顺序混淆。
- **依据与来源限制：** [E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)授权／隔离正常解锁、e3a1d97最终IO后重验、28a178e具体拒绝项；上述额外门限全部来自我，非学长算法。
- **已做验证及其范围：** 8项纯逻辑＋独立11340模拟服务：Takeoff／set_mode0，正常一次arm；真实只执行--check拒绝路径。
- **SHELL排查点：** 这是“uav arm反复被拒绝”的直接代码区域，应逐项看拒绝键。location状态取最新锁存文字未单独查年龄，脚本也不验证每条setpoint的数值／来源身份；不能以门控通过宣称飞行安全。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
26:
27: def reasons(values, now, ros_now, parameters):
28:     pending = []
29:     def fresh(key, timeout):
```

- [ ] SHELL排查结论：________；对应证据：________。

### R13 重型全功能记录器及自动性能／完整性分析

- **提交／部署边界：** `e4882bd`。10月28日凌晨从RK回退；本地历史候选保留，见E5恢复SHA与复验。
- **当前位置：** [tools/uav_log.py:118](../tools/uav_log.py)；[tools/uav_perf.py:21](../tools/uav_perf.py)；[tools/uav_log_report.py:23](../tools/uav_log_report.py)。
- **修改前／来源：** pack和旧Nano没有此本地实现；原ROS文本／飞控ULog独立存在。
- **我实际修改：** 自动分轮、core＋media两个record进程、60s分段、低空间关图像／停止、自有进程身份、参数／版本快照、1s性能和落地报告；推理源图JPEG是有损证据。
- **为什么这样改／要解决什么：** SHELL要求事故后能关联输入、决策和控制，再用性能决定是否留机。
- **依据与来源限制：** [E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)六轮实测：关闭67.23%、核心74.95%、完整76.93%，都超预设+5个百分点预算；数据完整性一轮缺1源图保持false。
- **已做验证及其范围：** 安全回归／隔离写入故障和真实禁动CPU对照；已按条件授权回退。
- **SHELL排查点：** 这是我新增的后台负载；不能从HEAD源码存在推定16:40仍在运行。与R15轻量版分开，旧版本不应擅自恢复。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
117:
118: class Recorder(object):
119:     def __init__(self,directory):
120:         self.directory=os.path.abspath(directory); self.stopping=False
```

- [ ] SHELL排查结论：________；对应证据：________。

### R14 C++诊断元数据／推理源帧及额外audit线程

- **提交／部署边界：** `e4882bd`。C++诊断已从RK回退；本地保留。实际任务源SHA01eab407...、二进制e1c5e29d...见E7。
- **当前位置：** [src/navigation_2d/src/shoot_with_move_base.cpp:504](../src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/detect.cpp:751](../src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp:786](../src/navigation_2d/src/detect.cpp)；[src/location/src/location_with_ftmini.cpp:57](../src/location/src/location_with_ftmini.cpp)。
- **可逐行对照原件：** [src/navigation_2d/src/shoot_with_move_base.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/shoot_with_move_base.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/navigation_2d/src/detect.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/navigation_2d/src/detect.cpp)；[src/location/src/location_with_ftmini.cpp 原包](../docs/验证记录/学长新增包核查_2026-10-25/原始关键文本/pack/src/location/src/location_with_ftmini.cpp)。
- **修改前／来源：** 18cd805正式C++仅status/readiness，没有这些结构化audit与推理源帧输出。
- **我实际修改：** 任务记录phase／目标／年龄／控制最大间隔；推理独立单槽audit线程输出结果JSON与按需JPEG源帧；定位按订阅者输出原始range及接纳状态。
- **为什么这样改／要解决什么：** 将事故中“当时用哪帧／为什么拒收／控制是否掉拍”与位姿关联；诊断失败不应影响主推理。
- **依据与来源限制：** [E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)实现／失败隔离／CPU数据，tools/deployment-flight-logging.json候选SHA；此额外工作线程是我新增。
- **已做验证及其范围：** 候选51个C++用例／十项任务回归与性能测试；随后恢复原任务／定位可执行文件。
- **SHELL排查点：** 本地最新C++与板端不同，本项不能用于解释16:40架次的实际进程负载；需要按二进制／源码SHA定位。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
503:   readiness_pub_.publish(message);
504:   if (audit_pub_.getNumSubscribers())
505:   {
506:     const auto now = ros::Time::now();
```

- [ ] SHELL排查结论：________；对应证据：________。

### R15 轻量日志白名单、5秒性能和启动／解锁关联

- **提交／部署边界：** `f253060；e3a1d97`。曾部署；16:35按SHELL关闭，当前INACTIVE。唯一后取回轮缺关键飞行话题，见E7。
- **当前位置：** [tools/uav_log_lite.py:23](../tools/uav_log_lite.py)；[tools/uav_log_lite.py:87](../tools/uav_log_lite.py)；[tools/uav_start.py:184](../tools/uav_start.py)。
- **修改前／来源：** R13重型方案已回退；SHELL随后授权重新评估低负载关键证据方案。
- **我实际修改：** 单独native C++ rosbag record，关键pose／setpoint／模式／任务等白名单，无图像／地图／scan／全TF；16MiB缓冲、60s分包、5s性能；3GiB启动、约1.1GiB运行保留；A自动启动／C/arm关联，不修改C++。
- **为什么这样改／要解决什么：** 在较低开销下保留关键飞行证据，避免恢复失败的重型方案。
- **依据与来源限制：** [E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)四轮禁动对照关闭／开启60.53%／60.41%，差值仅波动；SHELL16:35实测反馈Wi-Fi高CPU后关闭。
- **已做验证及其范围：** 197条合成消息及模拟故障／解锁；真实禁动对照，非空中规划节拍。
- **SHELL排查点：** 16:35起env持久UAV_LOG_DISABLED=1，当前仍禁用。不能用早期CPU均值排除实际Wi-Fi负载，也不能凭closed判飞行数据齐全。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
22:
23: TOPICS = ['/mavros/vision_pose/pose', '/mavros/local_position/pose',
24:           '/mavros/local_position/velocity_local', '/mavros/setpoint_raw/local',
25:           '/mavros/setpoint_raw/target_local', '/mavros/state',
```

- [ ] SHELL排查结论：________；对应证据：________。

### R16 记录进程身份、故障隔离与封包判定修正

- **提交／部署边界：** `e4882bd；f253060；e3a1d97`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/uav_log_lite.py:60](../tools/uav_log_lite.py)；[tools/uav_log_lite.py:72](../tools/uav_log_lite.py)；[tools/uav_log_lite.py:272](../tools/uav_log_lite.py)。
- **修改前／来源：** 初版轻量封包跟踪Python rosbag外壳，它可先退出而原生写入子进程尚未完成索引。
- **我实际修改：** 持有原生record；PID＋start_ticks＋boot_id确认归属／防复用；只停自有writer／params；等待.active收尾，异常记录errors；来源／master丢失有界封包；写入失败不杀定位／任务。
- **为什么这样改／要解决什么：** 修复“封包完成”提前标记及误杀无关PID的风险，让日志失败与飞行来源隔离。
- **依据与来源限制：** [E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)第一轮失败原件、第二轮隔离通过；[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)故障隔离；进程策略由我实现。
- **已做验证及其范围：** 模拟writer退出／身份变化、194／197条控制记录封包；不保证全部飞行阶段都录到。
- **SHELL排查点：** ready／closed仅描述writer生命周期，缺topic仍要单独检查；当前logger禁用，这些代码不等于常驻运行。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
59:
60: def identity(pid):
61:     try:
62:         raw = read('/proc/%d/stat' % pid)
```

- [ ] SHELL排查结论：________；对应证据：________。

## 8. 板级NPU与实际飞控配置

### P01 NPU上电时序的Python实现及归属事务

- **提交／部署边界：** `b95932c；84da31c`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/rk_npu_power.py:212](../tools/rk_npu_power.py)；[tools/rk_npu_power.py:297](../tools/rk_npu_power.py)；[tools/rk_npu_power.py:251](../tools/rk_npu_power.py)。
- **修改前／来源：** 不是学长10月25日npu-fix脚本；此前RK官方／厂商参考有npu_powerctrl硬件时序。
- **我实际修改：** 按锁定ProD资料实现GPIO4/10/11/54/55/56、电源／RESET32／IRQ36／SLEEP35及24MHz/32k时钟；新增boot归属journal、flock、占用检查、已上电不reset、失败逆序回退且只释放自有资源。
- **为什么这样改／要解决什么：** 空系统没有完成NPU电源／时钟初始化，USB无设备；需要可重复、不会重置运行中NPU的独立工具。
- **依据与来源限制：** [E10](../docs/验证记录/RK_NPU上电脚本验证_2026-10-24/README.md)历史ProD原理图／Rockchip npu_powerctrl来源和27个硬件写入点失败测试；硬件序列参考厂商，我写包装和保护，不冒称学长。
- **已做验证及其范围：** 12项本机／RK模拟、实际上电／推理及后续重启；V1.2精确PCB资料限制仍保留。
- **SHELL排查点：** 这属于板级控制算法／后台基础设施，可能影响模型是否就绪，但不是航迹控制；默认check只读，不能在本轮再执行on/off。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
211:
212: class Controller:
213:     def __init__(self, system):
214:         self.io = system
```

- [ ] SHELL排查结论：________；对应证据：________。

### P02 NPU固件RAM启动与每阶段目标／哈希校验

- **提交／部署边界：** `84da31c；1aae3e7；ec4c6d7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/rk_npu_boot.sh:32](../tools/rk_npu_boot.sh)；[tools/rk_npu_boot.sh:13](../tools/rk_npu_boot.sh)。
- **修改前／来源：** 厂商npu_upgrade提供db→td→wl→rs流程；学长包后来同核心固件，但工具／服务方式不同。
- **我实际修改：** 锁定upgrade_tool SHA及固件Git8858114...和blob；每阶段核对唯一目标／mode／LocationID，超时30s；只做板内NPU RAM启动，不写主CPU eMMC。
- **为什么这样改／要解决什么：** 恢复NPU固件环境，并防止USB编号或目标错误导致误刷。
- **依据与来源限制：** 脚本头锁定厂商npu_upgrade参考2a532b...；[E11](../docs/验证记录/RK_NPU实板上电_2026-10-24/README.md)实际升级和核心固件／学长包哈希对照。顺序参考厂商，校验／护栏由我加。
- **已做验证及其范围：** 真实runtime USB／devices和单图推理，后续两次重启；不是任意固件版本兼容证明。
- **SHELL排查点：** 哈希／target不匹配会拒绝模型启动；工具输出位置不能硬编码成永久总线号。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
31:
32: run_stage() {
33:     guard_usb Loader "$location" >/dev/null
34:     printf '\nNPU stage: %s\n' "$1"
```

- [ ] SHELL排查结论：________；对应证据：________。

### P03 NPU有序systemd自启与实际就绪判定

- **提交／部署边界：** `1aae3e7；ec4c6d7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/rk_npu_autostart.py:47](../tools/rk_npu_autostart.py)；[tools/rk_npu_autostart.py:63](../tools/rk_npu_autostart.py)；[tools/shell-npu.target](../tools/shell-npu.target)。
- **修改前／来源：** 不是学长rknn.service；SHELL在学长恢复包到达前已要求现有NPU开机自动准备。
- **我实际修改：** power→firmware→proxy依赖顺序；等runtime USB0019真实非占位序列号，再等devices与其相同；有界等待，已runtime不重复reset／传固件；停止按归属清理。
- **为什么这样改／要解决什么：** 解决“USB刚出现但rknn_server未就绪”以及每天手动初始化；避免另启一套冲突服务。
- **依据与来源限制：** [E12](../docs/验证记录/RK_NPU自启_2026-10-24/README.md)开机时间10.74/16.03/26.39s记录、20项测试；[E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md)新增包评估确认不能叠加两套控制链。状态机由我实现。
- **已做验证及其范围：** 真实重启自动ready及推理三头一致；长期／多次冷断电可靠性未完整覆盖。
- **SHELL排查点：** 正式保留代理是业务必需，并不等于可任意清理后台进程；失败需看阶段状态而不是只看unit active。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
46:
47: def firmware(io):
48:     usb = owned_runtime(io)
49:     if len(usb) == 1 and list(usb.values()) == ['2207:0019']:
```

- [ ] SHELL排查结论：________；对应证据：________。

### P04 NPU由固定USB编号改为物理拓扑唯一目标

- **提交／部署边界：** `ec4c6d7`。迁移／部署记录见E2；本轮未连接RK。
- **当前位置：** [tools/rk_npu_usb.py:16](../tools/rk_npu_usb.py)；[tools/rk_npu_usb.py:52](../tools/rk_npu_usb.py)。
- **修改前／来源：** 我最初自启脚本固定某USB总线／端口编号；一次正常重启后编号变化而失效。
- **我实际修改：** 核对fe900000.dwc3控制器内置hub、2207设备唯一性、厂商hub型号和对应父子拓扑；绑定本次工具LocationID并每阶段复核，不猜其编码。
- **为什么这样改／要解决什么：** 修复我自己引入的固定USB编号假设，让外部USB插拔／重启不选错NPU。
- **依据与来源限制：** [E13](../docs/验证记录/NPU自启修复与相机复查_2026-10-25/README.md)实际自启失败→修改→再次重启约28s ready，34项本机／RK回归；脚本为我新增。
- **已做验证及其范围：** 真实重启和六类单图三头一致；不证明所有USB断连都被修复。
- **SHELL排查点：** 10月28日首轮Wi-Fi失联原因未明，不能因本项通过就把系统断链归为已解决。

当前代码摘录（仅定位所述分支，不是完整函数）：

```text
15:
16: def internal_target(root=USB_ROOT, allow_absent=False):
17:     devices = []
18:     for path in root.iterdir():
```

- [ ] SHELL排查结论：________；对应证据：________。

### P05 实际改过的PX4参数：GPS_1_CONFIG 0→201（补充审计）

- **提交／部署边界：** `31a9aa0（记录／工具）；参数写入不属于Git源码`。10月24日真实改参并持久化；本轮仅查历史证据。
- **当前位置：** [logs/2026-10-24_外设接入、NPU排查与YOLO验收.md:69](../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md)。
- **修改前／来源：** GPS1已由SHELL确认接线亮灯，但GPS驱动未运行，GPS_1_CONFIG=0。
- **我实际修改：** 按GPS1接口将GPS_1_CONFIG改201并重启验证u-blox NEO-M9N回传；未改SYS_HAS_GPS=0、EKF2_AID_MASK=24、EKF2_HGT_MODE=3等视觉融合策略。
- **为什么这样改／要解决什么：** 完成SHELL要求的GPS驱动与参数接入修复，不是要改变室内视觉导航算法。
- **依据与来源限制：** [E3](../docs/验证记录/RK外设修复_2026-10-24/README.md) GPS原因与处理、原始NSH／参数读回；这项是我执行，不应因没源码diff而在审查中隐去。
- **已做验证及其范围：** 约8.63Hz raw/fix、但0卫星未定位；只确认驱动／数据。
- **SHELL排查点：** 本轮没有连接飞控复核当前值；按已保存证据表述。不声称修改过PX4 yaw定义／EKF融合算法，也不以此排除ROS外部姿态影响。

- [ ] SHELL排查结论：________；对应证据：________。

## 9. 辅助测试、离线分析与部署工具

### T01 ROS／应用／MAVROS回环与测试入口

- **提交／部署边界：** `b1cbd46`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_ros_loopback_test.py](../tools/rk_ros_loopback_test.py)；[tools/rk_mavros_loopback_test.py](../tools/rk_mavros_loopback_test.py)；[tools/rk_application_loopback_test.py](../tools/rk_application_loopback_test.py)；[tools/rk_run_validation.sh](../tools/rk_run_validation.sh)；[src/navigation_2d/test/ego_path_tracking_test.cpp](../src/navigation_2d/test/ego_path_tracking_test.cpp)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 私有ROS master、模拟MAVLink／本地pose与TF输入，消息／跟随／退出判据；给原EGO gtest补main，未改变其跟随控制算法。
- **为什么这样改／要解决什么：** 在RK迁移初期证明通信、构建覆盖和应用退出，不依赖真实解锁／飞行。
- **依据与来源限制：** [E1](../docs/验证记录/RK迁移与通信核验_2026-10-23/README.md)回环原始输出、测试main差异；判据与合成输入由我写。
- **已做验证及其范围：** 当时七包编译、8项EGO测试及回环；后来EGO默认入口已退役。
- **SHELL排查点：** 模拟FCU行为不是PX4实际飞行模型；测试通过不能反推真机定位／动力正常。

- [ ] SHELL排查结论：________；对应证据：________。

### T02 PX4 USB诊断和NSH串口封装

- **提交／部署边界：** `31a9aa0；a765be7；8aa597c；e4882bd`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_px4_usb_diagnostics.cpp](../tools/rk_px4_usb_diagnostics.cpp)；[tools/rk_px4_serial_shell.py](../tools/rk_px4_serial_shell.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 临时libusb MAVLink解析／遥测统计和有界NSH命令采集；后续扩展只读版本／状态／日志命令及串口退出处理。
- **为什么这样改／要解决什么：** 最初cdc_acm缺失仍需读真实飞控身份／GPS状态；随后保存故障证据。
- **依据与来源限制：** [E3](../docs/验证记录/RK外设修复_2026-10-24/README.md)初始真实7380条MAVLink／解析零错及各提交工具diff；串口白名单不是学长实现。
- **已做验证及其范围：** 真实未解锁遥测及命令原文；GPS配置写入单独在P05披露。
- **SHELL排查点：** 它是有权限访问飞控的诊断工具，不可当任意NSH命令都无动作；本轮不运行。

- [ ] SHELL排查结论：________；对应证据：________。

### T03 相机／外设／六类NPU实板探针

- **提交／部署边界：** `31a9aa0；84da31c；8aa597c`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_camera_lifecycle_check.py](../tools/rk_camera_lifecycle_check.py)；[tools/rk_peripheral_bench_check.py](../tools/rk_peripheral_bench_check.py)；[tools/rk_sixclass_probe.cpp](../tools/rk_sixclass_probe.cpp)；[tools/verify_rk_sixclass.py](../tools/verify_rk_sixclass.py)；[tools/rk_media_compare.cpp](../tools/rk_media_compare.cpp)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 有界启动／取样／清理，相机帧率与退出判据；直接RKNN API set/run/get写三头raw，离线解码六类结果／汇总；另有同JPEG软件／硬件解码与色差对照。
- **为什么这样改／要解决什么：** 区分驱动可用、NPU真的推理和检测业务精度，不用devices列表代替模型执行。
- **依据与来源限制：** [E3](../docs/验证记录/RK外设修复_2026-10-24/README.md)相机原始失败与修后四轮；[E11](../docs/验证记录/RK_NPU实板上电_2026-10-24/README.md)六类36图真实输出；[E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)同帧媒体对照；代码为我的测试实现。
- **已做验证及其范围：** 真实外设／六类平均47.575ms、mAP50约0.7465但干扰误框仍存在。
- **SHELL排查点：** 六类是历史试验模型，后被四类替代；该成绩不能用于当前四类延迟或事故精度解释。

- [ ] SHELL排查结论：________；对应证据：________。

### T04 异机六类交付独立评估、解码和AP计算

- **提交／部署边界：** `fcbeec3`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/verify_yolo_delivery.py](../tools/verify_yolo_delivery.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 核验ZIP／清单／原始JPG与XML，独立RGB letterbox、YOLO raw解码、逐类NMS／IoU／AP、TorchScript与ONNX对拍；纠正交回评估中的RGB反转、漏阈值、网格漏-0.5。
- **为什么这样改／要解决什么：** 不盲用另一电脑返回的指标，判断训练产物是否可作业务部署基线。
- **依据与来源限制：** YOLO训练成果验收_2026-10-24报告、原始交回脚本与本工具137–238行；独立复测逻辑由我写，未改原交付脚本／模型。
- **已做验证及其范围：** 36图CPU复测／最大差3.05e-5；干扰3/12仍误框，业务未过。
- **SHELL排查点：** 仅离线评估，未进入当前飞行解码路径；六类与pack四类不同，不能混为一项算法。

- [ ] SHELL排查结论：________；对应证据：________。

### T05 四类模型元信息／CPU-NPU独立解码比较

- **提交／部署边界：** `18cd805`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_model_metadata.cpp](../tools/rk_model_metadata.cpp)；[tools/rk_fourclass_probe.cpp](../tools/rk_fourclass_probe.cpp)；[tools/compare_fourclass_probe.py](../tools/compare_fourclass_probe.py)；[tools/prepare_fourclass_cases.py](../tools/prepare_fourclass_cases.py)；[tools/rk_fourclass_camera_check.py](../tools/rk_fourclass_camera_check.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 查询真实shape／fmt／元素数、三头raw导出；相同固定图CPU/NPU比较类别／框／延迟；相机插入已知图和缺模型退出判据。
- **为什么这样改／要解决什么：** 给V01反序／转置提供可复核实板证据，避免只靠ONNX推断RKNN旧ABI。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)模型metadata、12图raw和四类CPU-RKNN comparison；比较算法及样本选择是我实现。
- **已做验证及其范围：** 12图两端各10框、8张业务图类别对应，两张干扰均误判flammable；非盲测。
- **SHELL排查点：** 比较工具可能与生产解码共享假设，不能替代独立业务数据集；约0.66s是此四类实测。

- [ ] SHELL排查结论：________；对应证据：________。

### T06 DWA合成动力学、碰撞几何和时间积分修正

- **提交／部署边界：** `18cd805；1a3aa1a；a965a7c`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/dwa_planner_probe.py](../tools/dwa_planner_probe.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 发布合成map／scan／odom、按cmd历史积分FLU速度、footprint矩形／障碍间距和ray-box；修复丢弃>50ms elapsed的问题，单调时钟完整分段、≤25ms子步；footprint字符串安全解析；取消只针对ACTIVE远目标。
- **为什么这样改／要解决什么：** 验证0°／90°导航和障碍／取消，原工具积分不一致会歪曲到点与停滞判断，必须先修再调C19。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md) DWA旧工具积分时间复核、本地7项回归、43–46号A/B及最终三场景；旧失败原件未改。
- **已做验证及其范围：** 真实RK运行生产Navfn/DWA、合成运动三场景通过；不是实际无人机动力学。
- **SHELL排查点：** 早期“停滞”不能全算DWA问题，工具自身曾有缺陷；候选0.32m轮廓未实测桨叶包络，5cm模拟净空不等于实机安全。

- [ ] SHELL排查结论：________；对应证据：________。

### T07 任务模拟飞控／动作服务器十场景

- **提交／部署边界：** `18cd805；e4882bd`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/dwa_mission_probe.py](../tools/dwa_mission_probe.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 在私有localhost master模拟pose／FCU／模式服务／arming／GPIO ACK与简单位置响应，注入掉图／旧帧／接管／不爬升／下降超时等；保存设定值和请求。
- **为什么这样改／要解决什么：** 可逆验证任务策略和动作门，避免在真机发飞行／投放来测逻辑。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)任务十项JSON及[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)候选十项回归；模拟动力学和判据由我写。
- **已做验证及其范围：** 真实NPU＋模拟飞控十项；下降约10.3016s仍投放为保留策略。
- **SHELL排查点：** 模拟服务响应快且坐标由我设定，无法证明MAVROS/PX4物理frame正确；不能用它排除C03/C04。

- [ ] SHELL排查结论：________；对应证据：________。

### T08 定位与坐标算法单测／合成桥输入

- **提交／部署边界：** `18cd805；a765be7`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [src/location/test/localization_bridge_probe.py](../src/location/test/localization_bridge_probe.py)；[src/location/test/localization_guard_test.cpp](../src/location/test/localization_guard_test.cpp)；[src/location/test/tfmini_uart_pty_test.cpp](../src/location/test/tfmini_uart_pty_test.cpp)；[src/location/test/tfmini_i2c_protocol_test.cpp](../src/location/test/tfmini_i2c_protocol_test.cpp)；[src/navigation_2d/test/dwa_control_math_test.cpp](../src/navigation_2d/test/dwa_control_math_test.cpp)；[src/navigation_2d/test/yolo_tensor_layout_test.cpp](../src/navigation_2d/test/yolo_tensor_layout_test.cpp)；[src/navigation_2d/test/gtest_main.cpp](../src/navigation_2d/test/gtest_main.cpp)；[src/location/launch/localization_guard_bench.launch](../src/location/launch/localization_guard_bench.launch)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 断流／时间／高度／姿态／跳变、UART真实PTY字节流、I²C协议mock、frame变换／任务授权及layout测试；隔离topic测距入口。
- **为什么这样改／要解决什么：** 把我新增的纯数学与状态边界做成可复现验证，并补齐catkin/gtest入口。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md) 42项独立C++与15项桥测试、[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)追加I²C用例；测试假设由我指定。
- **已做验证及其范围：** gtest／PTY／合成输入通过，数量以原XML为准，不将catkin父子汇总重复计数。
- **SHELL排查点：** 这些测试会验证代码符合所写假设，不能独立证明假设符合实际传感器安装和飞控轴向。

- [ ] SHELL排查结论：________；对应证据：________。

### T09 真实联合遥测判据与零点估计

- **提交／部署边界：** `18cd805；a765be7`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_dwa_readonly_stack_check.py](../tools/rk_dwa_readonly_stack_check.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 按话题源／收到年龄、连续readiness比例／最大缺口和进程资源／零动作判据观测；增加measured_ground_reference收集range＋IMU投影求零点，真实/合成模式分开；退出后再读推理日志避免缓冲漏计。
- **为什么这样改／要解决什么：** 将多模块同时可用与单个模块启动区分，并为0.12m配置提供测量。
- **依据与来源限制：** [E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)首轮原始false与后续严格200/200、[E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md) 80帧零点／17项联合；代码为我新增。
- **已做验证及其范围：** 真实静止／未解锁观测；最终资源清理和时间窗判据通过。
- **SHELL排查点：** 观察器本身有CPU开销，某些图景输入是合成；不能将脚本pass解释为实飞系统验收。

- [ ] SHELL排查结论：________；对应证据：________。

### T10 I²C原始探针与PCA全部关断

- **提交／部署边界：** `a765be7`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_i2c_acceptance.py](../tools/rk_i2c_acceptance.py)；[tools/rk_pca_disable.py](../tools/rk_pca_disable.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 按确定设备／地址做TF原始帧及PCA寄存器读写；关断工具给16路FULL_OFF并SLEEP、读回核对；采样坏帧如实计数。
- **为什么这样改／要解决什么：** 确认SMBus问题和OE改线前PWM关闭，避免用扫描到地址当功能通过。
- **依据与来源限制：** [E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)原始9字节帧、10_OE改线前PWM全部关断.json；实现为我写。
- **已做验证及其范围：** 当时真实通信／寄存器读回；当前舵机损坏，本文不执行。
- **SHELL排查点：** 这组含真实硬件写操作，不因“验收工具”名字而只读；原始执行范围须看当次SHELL授权。

- [ ] SHELL排查结论：________；对应证据：________。

### T11 舵机台架角度方案及净空参数

- **提交／部署边界：** `a765be7`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_servo_bench_check.py](../tools/rk_servo_bench_check.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 初版相机90→80→90碰限位；改为source模式读取实际launch的170／55、投放30／90／140，显式净空确认；小角度必须脱机构并显式指定；结束FULL_OFF。
- **为什么这样改／要解决什么：** 修正我最初未经机构范围核对就选临时90°的不当假设，用SHELL确认的源码角度复测。
- **依据与来源限制：** [E4](../docs/验证记录/接线后整机复检_2026-10-27/README.md)日志§8明确保留首次碰限位、之后SHELL垫高并确认动作；不是学长新调角度。
- **已做验证及其范围：** 当时空仓机械动作反馈正常；不证明带载投放。
- **SHELL排查点：** 我临时选择90°是实际错误，应单独可追溯；当前不调用此工具，不根据能检测到就尝试已坏舵机。

- [ ] SHELL排查结论：________；对应证据：________。

### T12 监看入口占用／进程归属和只读观测

- **提交／部署边界：** `fd7821a；9cd127c；8aa597c`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/start_lichtblick_monitor.sh](../tools/start_lichtblick_monitor.sh)；[tools/monitor_device_handles.py](../tools/monitor_device_handles.py)；[tools/lichtblick_readonly_observe.py](../tools/lichtblick_readonly_observe.py)；[tools/verify_lichtblick_runtime.py](../tools/verify_lichtblick_runtime.py)；[tools/foxglove_wire_check.py](../tools/foxglove_wire_check.py)；[tools/rk_wireless_observe.py](../tools/rk_wireless_observe.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** bridge／ground分流、已有graph和真实设备句柄占用拒绝；缺fuser后改只读/proc扫描，权限不足／路径缺失拒绝；WebSocket能力／原生ROS连通和帧率／年龄观测。
- **为什么这样改／要解决什么：** 避免教学监看重复起传感器、占串口或意外给客户端发布权限，分清原生ROS与WebSocket。
- **依据与来源限制：** Lichtblick教学报告7项集成、[E6](../docs/验证记录/无线地面排查_2026-10-27/README.md)真实原生UI及ROS_HOSTNAME问题；shell及观察判据为我写。
- **已做验证及其范围：** 实际UI／回放／只读握手和句柄测试通过；不覆盖无线飞行最坏延迟。
- **SHELL排查点：** 有的脚本是ground独立入口，会真实启动感知，不能在已有任务旁重复运行；native客户端的权限与只读桥权限也不同。

- [ ] SHELL排查结论：________；对应证据：________。

### T13 日志性能预算、统计与自动对照

- **提交／部署边界：** `e4882bd；f253060；e3a1d97`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/uav_logging_benchmark.py](../tools/uav_logging_benchmark.py)；[tools/uav_logging_compare.py](../tools/uav_logging_compare.py)；[tools/uav_lite_benchmark.py](../tools/uav_lite_benchmark.py)；[tools/uav_lite_compare.py](../tools/uav_lite_compare.py)；[tools/uav_lite_integration.py](../tools/uav_lite_integration.py)；[tools/uav_logging_fault_probe.py](../tools/uav_logging_fault_probe.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 同负载开关分轮、readiness预热、CPU／内存／iowait／定时器统计和预设预算；模拟writer异常隔离与轻量端到端控制消息；完整／核心／轻量分开。
- **为什么这样改／要解决什么：** 按SHELL负载决定留机，防止只看平均CPU就忽略关键时序／不完整数据。
- **依据与来源限制：** [E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)原始6轮及[E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)轻量四轮；+5个百分点、150MiB等预算是我提出的本任务工程筛查，不是PX4官方放行值。
- **已做验证及其范围：** 重型不通过而回退，轻量禁动通过后仍按SHELLWi-Fi反馈关闭；所有失败轮保留。
- **SHELL排查点：** 统计含观察器负载、静止导航无真实goal；不能宣称空中开销或零性能影响。

- [ ] SHELL排查结论：________；对应证据：________。

### T14 飞行记录完整性／频率报告算法

- **提交／部署边界：** `e4882bd；f253060`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/uav_log_report.py](../tools/uav_log_report.py)；[tools/uav_log_lite.py](../tools/uav_log_lite.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 按bag实际起止／话题／消息数／缺口和错误／active状态分析；完整候选将推理元数据与源帧配对，轻量明确不保存图像；缺数据保持missing而非零动作。
- **为什么这样改／要解决什么：** 避免封包成功被误读成所有飞行阶段已保存；降低事后漏证与错误归因。
- **依据与来源限制：** [E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)缺1张源图判false、E9active包拒扫、[E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md)唯一新增轮无关键topic；报告规则是我实现。
- **已做验证及其范围：** 纯逻辑测试及真实封包后分析；不扫描活动bag为完整结果。
- **SHELL排查点：** 报告仍依赖录到的证据，不能据话题未出现判断当时实际动作不存在。

- [ ] SHELL排查结论：________；对应证据：________。

### T15 PX4 ULog离线航向／分段／事件分析

- **提交／部署边界：** `68e6cba`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/analyze_px4_takeoff.py](../tools/analyze_px4_takeoff.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** pyulog读取各EKF实例／姿态／本地NED／setpoint／mode／event；四元数Euler、相对时间对齐、分段统计与图；输出保留原始NED，不改飞控数据。
- **为什么这样改／要解决什么：** 把锁桨前控制异常与落地／碰撞／锁桨后惯性推算分开，避免选择错架次或实例。
- **依据与来源限制：** [E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md) 229／230原文件SHA、ROS事件关联与早先13点事故报告；分析方法由我写。
- **已做验证及其范围：** 两次事故不同ULog读数与曲线核对；确认230锁桨前主EKF未见229同样的inertial-dead-reckoning。
- **SHELL排查点：** EKF位置不是地面真值，锁桨前也可能已碰墙；事件相关性不足以证明哪段ROS算法是根因。

- [ ] SHELL排查结论：________；对应证据：________。

### T16 原包／训练交付／仿真交接的清单和路径核查

- **提交／部署边界：** `bdd7955；2f1853b；f40ffb9`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/inspect_1025_packages.py](../tools/inspect_1025_packages.py)；[tools/package_sim_handoff.py](../tools/package_sim_handoff.py)；[tools/sim_handoff/verify_package.py](../tools/sim_handoff/verify_package.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 归档先枚举类型／路径、拒绝越界／链接／覆盖，提取后逐文件SHA；打包当前源码／素材，外包校验及接收机复核。
- **为什么这样改／要解决什么：** 保留原包与可复现交付，防止把已有build/devel或不明归档执行成开发源码。
- **依据与来源限制：** [E0](../docs/验证记录/学长新增包核查_2026-10-25/README.md) 140文件／21关键原文清单；全链仿真异机资源报告766载荷／753源码验证；算法由我写。
- **已做验证及其范围：** ZIP/CRC／SHA和路径验证，不构成仿真运行。
- **SHELL排查点：** 原件不改写；另电脑返回的训练代码虽由我提交归档，也不等于由我创作，见附录来源排除。

- [ ] SHELL排查结论：________；对应证据：________。

### T17 依赖／ABI核查、媒体构建和部署／回退工具

- **提交／部署边界：** `b1cbd46；31a9aa0；18cd805；8aa597c；e4882bd；e3a1d97`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_dependency_audit.sh](../tools/rk_dependency_audit.sh)；[tools/elf_dwarf_layout.py](../tools/elf_dwarf_layout.py)；[tools/rk_kernel_export_check.py](../tools/rk_kernel_export_check.py)；[tools/rk_build_cdc_acm.sh](../tools/rk_build_cdc_acm.sh)；[tools/fetch_dwa_deps.py](../tools/fetch_dwa_deps.py)；[tools/build_rk_media_runtime.sh](../tools/build_rk_media_runtime.sh)；[tools/finalize_uav_logging.py](../tools/finalize_uav_logging.py)；[tools/rollback_uav_logging.py](../tools/rollback_uav_logging.py)；[tools/finalize_uav_lite.py](../tools/finalize_uav_lite.py)；[tools/verify_uav_logging_rollback.sh](../tools/verify_uav_logging_rollback.sh)；[tools/run_uav_logging_verification.sh](../tools/run_uav_logging_verification.sh)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 读取系统／ELF／内核符号校验架构和ABI；精确版本依赖获取与媒体库构建；部署按清单哈希，回退核路径／备份SHA、原二进制恢复并检缺件。
- **为什么这样改／要解决什么：** 让目标版本可复现，不把只复制源码当成已启用；重型性能失败后精确恢复。
- **依据与来源限制：** [E1](../docs/验证记录/RK迁移与通信核验_2026-10-23/README.md)/[E3](../docs/验证记录/RK外设修复_2026-10-24/README.md)/[E2](../docs/验证记录/DWA四类部署_2026-10-25/README.md)/[E5](../docs/验证记录/机载飞行记录_2026-10-27/README.md)/[E9](../docs/验证记录/轻量机载日志_2026-10-28/README.md)构建、部署及回退原始输出；这些胶水逻辑为我写。
- **已做验证及其范围：** 指定模块构建和恢复哈希；未把所有历史模块跳过称完整编译。
- **SHELL排查点：** 此组有安装／写文件／服务操作，审查时看实际执行记录；本轮没有运行这些脚本或重连RK。

- [ ] SHELL排查结论：________；对应证据：________。

### T18 文档几何／链接与教程生成工具

- **提交／部署边界：** `fd7821a；7e287d3`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/render_monitor_guide.cjs](../tools/render_monitor_guide.cjs)；[tools/check_doc_links.ps1](../tools/check_doc_links.ps1)；[tools/doc_links_known_missing.json](../tools/doc_links_known_missing.json)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 教程截图排布／渲染及Markdown相对链接／页内锚点检查，历史缺件白名单；修脚本默认Root求值。
- **为什么这样改／要解决什么：** 维护可打开的证据与文档；不涉及飞行数值或控制。
- **依据与来源限制：** 原脚本及当天日志链接核查结果；属于我的工具实现。
- **已做验证及其范围：** 0新增断链／既有16历史缺件及13旧绝对引用单列。
- **SHELL排查点：** 白名单只允许历史缺件，不能用来掩盖本轮新证据缺失；此组不按故障嫌疑算法优先排查。

- [ ] SHELL排查结论：________；对应证据：________。

### T19 隔墙Wi-Fi分级禁动取样与有界清理

- **提交／部署边界：** `17f2b16`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/wifi_ground_probe.py](../tools/wifi_ground_probe.py)；[tools/monitor_jpeg_probe.py](../tools/monitor_jpeg_probe.py)；[tools/uav_start_gate_probe.py](../tools/uav_start_gate_probe.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 私有／限定master按定位→相机→禁动推理→JPEG显式分级，持久证据／信号处理／超时清理；合成图像或真实ROS生命周期探针确认订阅计数／header／重连／陈旧／CPU。
- **为什么这样改／要解决什么：** 首轮同时启动后失联且/tmp证据丢失，后续逐级定位额外负载并验证R07／R10修复。
- **依据与来源限制：** [E7](../docs/验证记录/再次起飞失败_2026-10-28/README.md)/[E8](../docs/验证记录/隔墙WiFi禁动续验_2026-10-28/README.md)原始结果，首次失联无旧boot内核日志，原因未明；脚本由我写。
- **已做验证及其范围：** 349＋445条真实禁动样本、7项relay、旧新启动器对照；未发飞行／舵机命令。
- **SHELL排查点：** 时间在16:40后，绝不能当作当时事故的新增飞行进程；静止网络成功不代表首轮失联根因已闭环。

- [ ] SHELL排查结论：________；对应证据：________。

### T20 电源／USB／解锁／记录器故障单测

- **提交／部署边界：** `b95932c；1aae3e7；ec4c6d7；e4882bd；f253060`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/test_rk_npu_power.py](../tools/test_rk_npu_power.py)；[tools/test_rk_npu_autostart.py](../tools/test_rk_npu_autostart.py)；[tools/test_rk_npu_usb.py](../tools/test_rk_npu_usb.py)；[tools/test_uav_logging.py](../tools/test_uav_logging.py)；[tools/test_uav_log_lite.py](../tools/test_uav_log_lite.py)；[tools/test_uav_arm.py](../tools/test_uav_arm.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 用mock注入硬件写失败、资源归属变化、USB多目标／编号变化、PID复用、磁盘不足、active包及arm状态／时间门的边界。
- **为什么这样改／要解决什么：** 证明辅助状态机符合预期且失败可控，不用真实故障／真实解锁验证这些分支。
- **依据与来源限制：** 对应P01–P04、R12–R16原始测试报告及测试源码；mock策略为我写。
- **已做验证及其范围：** 本机／RK解释器测试通过；不等同真实故障发生时全系统可靠性验收。
- **SHELL排查点：** 测试可与实现共享错误假设，尤其坐标与模式先后关系仍须独立实物证据。

- [ ] SHELL排查结论：________；对应证据：________。

### T21 RK固件归档／分区和目标架构离线核查

- **提交／部署边界：** `2f1853b`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/inspect_rk_firmware.py](../tools/inspect_rk_firmware.py)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 离线解析固件头、成员、分区／容量及ELF架构，计算完整性清单，不执行解包出的可执行文件。
- **为什么这样改／要解决什么：** RK刷写／分区容量修正阶段需要核对镜像与目标，保留可重复证据。
- **依据与来源限制：** 工具原文及RK分区容量修正／RK分区写入修正两份10月23日报告；本提交归档不代表当天才创作。
- **已做验证及其范围：** 已存报告的镜像结构／容量／SHA核验；本轮未刷机。
- **SHELL排查点：** 不在飞行控制环；若追究系统来源应看镜像版本，而不是当作yaw算法。

- [ ] SHELL排查结论：________；对应证据：________。

### T22 RK USB-SSH自启网络配置

- **提交／部署边界：** `2f1853b`。辅助／离线／隔离验证代码；是否曾临时上板见各报告，不属于默认飞行控制环。
- **当前位置：** [tools/rk_usb_ssh.sh](../tools/rk_usb_ssh.sh)；[tools/rk-usb-ssh.service](../tools/rk-usb-ssh.service)。
- **修改前／来源：** 学长pack无这些本任务辅助实现；测试所用既有ROS／MAVROS／模型接口不因此变成我原创。
- **我实际修改：** 创建／选择USB gadget网卡、指定地址与服务启动检查，提供独立USB SSH链路。
- **为什么这样改／要解决什么：** 在无人外网／无线未配置时保持板端维护通道。
- **依据与来源限制：** 10月23日SSH修复日志与脚本；系统配置由我完成，非学长ROS包。
- **已做验证及其范围：** USB真实登录与重启服务记录；不能从别名存在认定当前插的USB就是RK。
- **SHELL排查点：** 涉及网络路径和重连可达性，不是算法方向变化；10月28日晚电脑USB实际为手机已另记。

- [ ] SHELL排查结论：________；对应证据：________。

## 10. 明确继承、第三方或不应误算为我新算法的区域

| 区域 | 本轮逐段核对后的来源结论 |
|---|---|
| 从/mavros/local_position/pose读yaw、第一次pose设hold_yaw | 学长pack已有；我的新增是C04、C05、C08，不是“读取PX4 yaw”本身 |
| field_yaw_deg=-90、fieldToMap的R(-field_yaw) | 学长pack已有；我新增的map/local对齐是另外一层C03，不应把两者合称一个来源 |
| pixelToBody、camera_hfov/camera_yaw、视觉偏移bodyToLocal | pack已带对应公式和相机参数；当前几何函数主体没有在迁移时重造。真实安装是否符合仍需排查，但不是我10月23日后新公式 |
| flightControl默认构造委托、publisher存在性保护 | 学长pack已修；我的C17是数值／时间戳及服务响应补充 |
| 六线程MultiThreadedSpinner(6) | 学长pack已有；C16新增的是任务大锁／异步组织，不是线程数 |
| 20s起飞超时常数、2s最低等待 | 原包已有；C09把超时继续任务改成中止，并参数化，没有新调20s数值 |
| 10s下降超时仍投放、3s ACK超时仍爬升 | 原包已有，SHELL明确要求保留；我增加状态文字／权限保护，不将它们冒称我新增策略 |
| POSCTL→AUTO.LAND与阶段2救助语义 | 学长原流程；C13只是过渡权限与停止竞争输出等修正 |
| nextLeg恢复ControlMode::TRACK、航点表与返航路线 | pack原文已有（含提到2026-10-24故障的注释）；不能凭注释日期把作者归给我 |
| 危险品中文别名与person标签 | pack主要映射已有；我新增危险品三类权限限制、目标匹配与令牌，见C14 |
| YOLO letterbox、RGB预处理、逐类NMS、阈值0.5／0.45、v5网格-0.5 | pack四类检测主体已有；我适配ABI／五维转置／边界与时序，见V项。六类交回评估漏-0.5是T04另一套历史脚本问题 |
| pack四类best.onnx／best.rknn／labels顺序 | 学长原件，未重训／改权重；文件名／构建接入V04由我落地 |
| DWA速度0.4、合速度0.45、最小平移0.10、角速度0、acc_lim、goal容差0.10／0.20、sim_time2s、采样／path32／goal24等 | 原包原值；本次发现的唯一生产DWA数值调参是C19的occdist_scale0.15→0.02 |
| footprint±0.16方形、inflation0.25、cost_scaling3.0、costmap尺寸／resolution／频率 | 原包原值；我只注明未按实机包络测量，并未把它们验证成真实避障净空 |
| Cartographer主Lua、雷达yaw≈π、S2波特率／DenseBoost | Lua与旧Nano／pack／RK核对主链相同；相关launch简化来自pack。配置能工作不代表真实动态坐标已验，但不是我新增轴反转 |
| 雷达SDK2.1.0 | SHELL要求保留现用；未导入pack的2.0.0回退，没有此次新增雷达SDK算法 |
| servo_debug.py交互前端、舵机170／55和投放30／90／140 | pack已有并原样导入主要前端，角度沿用；我的I²C6默认和台架临时角度错误另列L13/T11 |
| usb_cam大部分采集／上游转换 | 第三方ros-drivers0.3.7 addab4a...，不是学长，也不是我原创；我的补丁R01–R04单列 |
| NPU固件／librknn_api／npu_transfer_proxy | 锁定官方原件；学长恢复包核心与之逐字节相同。我新增上电／校验／自启外壳P项，不改固件内部推理算法 |
| 另电脑返回的六类训练／导出脚本和模型 | SHELL交回的异机产物，曾由我提交归档但不是本线程创作；独立复核T04／T03属于我，不改写原返回脚本 |
| 10月22日基线中的EGO、旧任务／仿真和更早保护 | 本次时间范围外；后续未改的旧算法不伪列成10月23日后新增。迁移时重新引入／改写的保护在L/C项披露 |

“继承”只说明来源，不说明算法一定正确、适配当前机械或得到实飞验收。默认EGO→DWA、六类→学长四类是SHELL后续决定，算法迁移中的具体新增公式仍由我负责溯源。

## 11. 全提交清单与逐文件覆盖

下表来自本次实际Git读取。文件覆盖表包含src和tools中从10月22日基线到审查HEAD的全部净变化；此外逐提交扫描检查了后来被覆盖／回退的变化。Git中没有单独“学长作者”标记，因此必须看原包比较，不能按提交人筛掉导入文件。

### 11.1 全部提交

| 提交 | 真实北京时间 | 说明 |
|---|---|---|
| `b1cbd46` | 2026-10-24T03:31:15+08:00 | fix(rk): validate ROS migration and timer shutdown |
| `7920ca9` | 2026-10-24T04:08:24+08:00 | docs(vision): plan fresh YOLO training and offline RK deployment |
| `ad40b60` | 2026-10-24T04:50:10+08:00 | docs(vision): prepare portable YOLO training handoff |
| `31a9aa0` | 2026-10-24T22:50:36+08:00 | fix(rk): enable PX4 USB and repair camera shutdown |
| `fcbeec3` | 2026-10-25T01:17:29+08:00 | docs(vision): audit returned YOLO training artifacts |
| `b95932c` | 2026-10-25T02:26:57+08:00 | feat(rk): add guarded manual NPU power control |
| `84da31c` | 2026-10-25T03:20:17+08:00 | fix(rk): restore NPU boot and validate six-class YOLO on hardware |
| `1aae3e7` | 2026-10-25T03:36:07+08:00 | feat(rk): enable ordered NPU startup on boot |
| `64983ee` | 2026-10-25T03:59:01+08:00 | docs: record indoor flight scope and wiring blockers |
| `2f1853b` | 2026-10-25T04:05:11+08:00 | chore: archive pending project assets for 1024 milestone |
| `f40ffb9` | 2026-10-25T04:24:06+08:00 | feat(sim): package 1024 source and materials for cross-PC simulation |
| `2b787b5` | 2026-10-25T23:05:55+08:00 | docs: record 1025 assembly checks and NPU startup failure |
| `ec4c6d7` | 2026-10-25T23:29:01+08:00 | fix: bind NPU autostart to internal USB topology |
| `bdd7955` | 2026-10-25T23:56:28+08:00 | docs: assess incoming NPU and ROS packages for selective integration |
| `18cd805` | 2026-10-26T14:09:51+08:00 | 迁移四类RKNN与DWA并补齐任务保护 |
| `1a3aa1a` | 2026-10-26T14:31:02+08:00 | 修正DWA隔离测试时序并记录USB枚举故障 |
| `7b26a46` | 2026-10-26T14:33:44+08:00 | 记录RK连接持续受阻与待续验事项 |
| `a965a7c` | 2026-10-26T17:40:17+08:00 | 完成DWA障碍配置与严格联合验收并记录断连邮件提醒 |
| `5dc984c` | 2026-10-26T17:52:35+08:00 | 完成四类RKNN与DWA机上部署最终收尾 |
| `fd7821a` | 2026-10-27T01:08:43+08:00 | Add Lichtblick teaching guide and isolated ground preview |
| `9cd127c` | 2026-10-27T01:47:21+08:00 | Complete Lichtblick capture, replay and shutdown handoff |
| `301bb96` | 2026-10-27T01:49:22+08:00 | Record authorized post-delivery RK shutdown |
| `a765be7` | 2026-10-27T22:22:04+08:00 | Fix TFmini I2C frames and verify RK preflight fusion |
| `30368a4` | 2026-10-27T23:50:36+08:00 | Add current RK takeoff guide with checked commands |
| `8aa597c` | 2026-10-28T03:39:47+08:00 | Fix RK wireless monitoring and accelerate MJPEG with MPP/RGA |
| `e4882bd` | 2026-10-28T05:45:36+08:00 | Add local flight logging and USB performance evidence; roll back RK deployment |
| `81aefa6` | 2026-10-28T05:46:53+08:00 | Record authorized RK shutdown after logging rollback |
| `68e6cba` | 2026-10-28T13:50:23+08:00 | Analyze takeoff vibration, vision gaps and AUTO.TAKEOFF control path |
| `f253060` | 2026-10-28T14:34:00+08:00 | Add lightweight flight logging and guarded OFFBOARD arming; RK validation pending repair |
| `e3a1d97` | 2026-10-28T15:11:52+08:00 | Validate RK lightweight logging and add AGENTS field diagnostics overview |
| `7e287d3` | 2026-10-28T17:25:14+08:00 | Archive current UAV project, organize logs and document paused flight logging |
| `66db2d0` | 2026-10-28T17:27:52+08:00 | Record verified initial GitHub publication and current collaboration state |
| `28a178e` | 2026-10-28T17:52:25+08:00 | Fix startup gate subscription lifetime; verify USB no-actuation checks and document takeoff incidents |
| `c223f8b` | 2026-10-28T17:52:57+08:00 | Record Gmail notification and final verification scope |
| `17f2b16` | 2026-10-28T19:58:57+08:00 | Release idle JPEG source subscriptions and document wall-separated Wi-Fi ground checks |
| `ccbb548` | 2026-10-28T19:59:55+08:00 | Record Wi-Fi follow-up delivery and notification |

共36条提交；git rev-list --all --not HEAD为空，当前所有本地分支可达提交未另藏一条未审历史。已比对主要功能提交b1cbd46、31a9aa0、18cd805、a965a7c、a765be7、8aa597c、e4882bd、f253060、e3a1d97、28a178e、17f2b16及NPU系列，而非只看最终diff。

### 11.2 src／tools全部变化文件

“有实质文本／注释差异”只说明不是纯空白，不能自动等同改变算法；具体责任看条目。“pack无同路径”也可能是第三方／测试，不自动算我原创。未在表中出现的已跟踪src/tools文件没有此范围的净变化；历史中间版本仍看11.1。

| 文件 | 条目或不纳入算法的原因 | 相对学长pack |
|---|---|---|
| [src/location/CMakeLists.txt](../src/location/CMakeLists.txt) | V04／R08／T17；目标／依赖／测试构建适配，非新增控制律 | 有实质文本／注释差异，按条目细分 |
| [src/location/README.md](../src/location/README.md) | 说明文档／来源与边界，无可执行算法 | 有实质文本／注释差异，按条目细分 |
| [src/location/include/TFplus.h](../src/location/include/TFplus.h) | L09 | 有实质文本／注释差异，按条目细分 |
| [src/location/include/i2c.h](../src/location/include/i2c.h) | L10–L12；我的协议接口／边界／资源适配 | 有实质文本／注释差异，按条目细分 |
| [src/location/include/localization_guard.h](../src/location/include/localization_guard.h) | L02／L03／L04／L05／L08 | pack无同路径 |
| [src/location/include/tf_mini_i2c.h](../src/location/include/tf_mini_i2c.h) | L10–L12；我的协议接口／边界／资源适配 | 有实质文本／注释差异，按条目细分 |
| [src/location/launch/cartographer_2d.launch](../src/location/launch/cartographer_2d.launch) | 学长pack入口导入；主SLAM配置／雷达方向未新调，见继承清单 | 与pack字节相同 |
| [src/location/launch/lichtblick_bridge_readonly.launch](../src/location/launch/lichtblick_bridge_readonly.launch) | R08 | pack无同路径 |
| [src/location/launch/lichtblick_ground_preview.launch](../src/location/launch/lichtblick_ground_preview.launch) | R08／T12；我的独立禁动感知监看组合 | pack无同路径 |
| [src/location/launch/localization_guard_bench.launch](../src/location/launch/localization_guard_bench.launch) | T08 | pack无同路径 |
| [src/location/launch/location.launch](../src/location/launch/location.launch) | L13／L14 | 有实质文本／注释差异，按条目细分 |
| [src/location/launch/rk_mavros_bench.launch](../src/location/launch/rk_mavros_bench.launch) | R08 | pack无同路径 |
| [src/location/launch/rk_mavros_vision.launch](../src/location/launch/rk_mavros_vision.launch) | R08 | pack无同路径 |
| [src/location/launch/rk_preflight.launch](../src/location/launch/rk_preflight.launch) | R08 | pack无同路径 |
| [src/location/launch/rplidar.launch](../src/location/launch/rplidar.launch) | 学长pack入口导入；主SLAM配置／雷达方向未新调，见继承清单 | 与pack字节相同 |
| [src/location/package.xml](../src/location/package.xml) | V04／R08／T17；目标／依赖／测试构建适配，非新增控制律 | 有实质文本／注释差异，按条目细分 |
| [src/location/param/rk_height_reference_20261027.yaml](../src/location/param/rk_height_reference_20261027.yaml) | L14 | pack无同路径 |
| [src/location/rviz/ros_navigation.rviz](../src/location/rviz/ros_navigation.rviz) | 学长pack显示配置导入，无机载控制算法 | 与pack字节相同 |
| [src/location/scripts/monitor_jpeg.py](../src/location/scripts/monitor_jpeg.py) | R05／R05／R06／R06／R07 | pack无同路径 |
| [src/location/src/TFplus.cpp](../src/location/src/TFplus.cpp) | L09 | 有实质文本／注释差异，按条目细分 |
| [src/location/src/i2c.cpp](../src/location/src/i2c.cpp) | L10／L11／L11 | 有实质文本／注释差异，按条目细分 |
| [src/location/src/location_with_ftmini.cpp](../src/location/src/location_with_ftmini.cpp) | L01／L02／L03／L04／L05／L06／L07／L07／L08／L08／R14 | 有实质文本／注释差异，按条目细分 |
| [src/location/src/tf_mini_i2c.cpp](../src/location/src/tf_mini_i2c.cpp) | L10／L11／L12／L12 | 有实质文本／注释差异，按条目细分 |
| [src/location/src/tfmini_test.cpp](../src/location/src/tfmini_test.cpp) | L09–L13／T10；我的测距样例／错误处理／参数适配 | 有实质文本／注释差异，按条目细分 |
| [src/location/test/localization_bridge_probe.py](../src/location/test/localization_bridge_probe.py) | T08 | pack无同路径 |
| [src/location/test/localization_guard_test.cpp](../src/location/test/localization_guard_test.cpp) | T08 | pack无同路径 |
| [src/location/test/tfmini_i2c_protocol_test.cpp](../src/location/test/tfmini_i2c_protocol_test.cpp) | T08 | pack无同路径 |
| [src/location/test/tfmini_uart_pty_test.cpp](../src/location/test/tfmini_uart_pty_test.cpp) | T08 | pack无同路径 |
| [src/navigation_2d/CMakeLists.txt](../src/navigation_2d/CMakeLists.txt) | V04 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/README.md](../src/navigation_2d/README.md) | 说明文档／来源与边界，无可执行算法 | pack无同路径 |
| [src/navigation_2d/detect_mode/26/README.md](../src/navigation_2d/detect_mode/26/README.md) | 学长模型／标签原件；README/SHA为我的说明核验，V04 | pack无同路径 |
| [src/navigation_2d/detect_mode/26/SHA256SUMS](../src/navigation_2d/detect_mode/26/SHA256SUMS) | 学长模型／标签原件；README/SHA为我的说明核验，V04 | pack无同路径 |
| [src/navigation_2d/detect_mode/26/best.onnx](../src/navigation_2d/detect_mode/26/best.onnx) | 学长模型／标签原件；README/SHA为我的说明核验，V04 | 与pack字节相同 |
| [src/navigation_2d/detect_mode/26/best.rknn](../src/navigation_2d/detect_mode/26/best.rknn) | 学长模型／标签原件；README/SHA为我的说明核验，V04 | 与pack字节相同 |
| [src/navigation_2d/detect_mode/26/labels.txt](../src/navigation_2d/detect_mode/26/labels.txt) | 学长模型／标签原件；README/SHA为我的说明核验，V04 | 与pack字节相同 |
| [src/navigation_2d/docs/field_route_preview.png](../src/navigation_2d/docs/field_route_preview.png) | 学长包路线图导入，非我的算法 | 与pack字节相同 |
| [src/navigation_2d/include/audit_json.h](../src/navigation_2d/include/audit_json.h) | R14；我的JSON有限数／转义辅助，已从RK回退 | pack无同路径 |
| [src/navigation_2d/include/detect.h](../src/navigation_2d/include/detect.h) | V01–V09／R14；学长检测框架＋我的异步／新鲜度／审计接口 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/include/detection_freshness.h](../src/navigation_2d/include/detection_freshness.h) | V06 | pack无同路径 |
| [src/navigation_2d/include/dwa_control_math.h](../src/navigation_2d/include/dwa_control_math.h) | C03／C04／C06 | pack无同路径 |
| [src/navigation_2d/include/dwa_mission_policy.h](../src/navigation_2d/include/dwa_mission_policy.h) | C09／C12／C13／C14 | pack无同路径 |
| [src/navigation_2d/include/flight_control.h](../src/navigation_2d/include/flight_control.h) | C17；pack接口导入／旧EGO接口退役，frame默认语义未新造 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/include/latest_result_cache.h](../src/navigation_2d/include/latest_result_cache.h) | C15／V07 | pack无同路径 |
| [src/navigation_2d/include/mission_with_move_base.h](../src/navigation_2d/include/mission_with_move_base.h) | C01–C16；学长任务框架＋我的状态／门控声明 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/include/yolo_tensor_layout.h](../src/navigation_2d/include/yolo_tensor_layout.h) | V01／V02／V03／V09／V09 | pack无同路径 |
| [src/navigation_2d/launch/detect.launch](../src/navigation_2d/launch/detect.launch) | V04 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/launch/move_base.launch](../src/navigation_2d/launch/move_base.launch) | C18 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/launch/navigation_2d.launch](../src/navigation_2d/launch/navigation_2d.launch) | C01／C18 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/package.xml](../src/navigation_2d/package.xml) | V04／R08／T17；目标／依赖／测试构建适配，非新增控制律 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/param/costmap_common_params.yaml](../src/navigation_2d/param/costmap_common_params.yaml) | 学长pack数值保留；我新增“未量测实机包络”注释，非新轮廓算法 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/param/dwa_local_planner_params.yaml](../src/navigation_2d/param/dwa_local_planner_params.yaml) | C19 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/param/global_costmap_params.yaml](../src/navigation_2d/param/global_costmap_params.yaml) | 学长pack参数原值；仅空白归一，不是我调参 | 与pack仅空白差异 |
| [src/navigation_2d/param/local_costmap_params.yaml](../src/navigation_2d/param/local_costmap_params.yaml) | 学长pack参数原值；仅空白归一，不是我调参 | 与pack仅空白差异 |
| [src/navigation_2d/param/move_base_params.yaml](../src/navigation_2d/param/move_base_params.yaml) | 学长pack参数原值；仅空白归一，不是我调参 | 与pack仅空白差异 |
| [src/navigation_2d/src/detect.cpp](../src/navigation_2d/src/detect.cpp) | V01／V02／V02／V03／V03／V04／V05／V06／V06／V07／V07／V08／V08／V09／R14／R14 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/src/detect_main.cpp](../src/navigation_2d/src/detect_main.cpp) | V05 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/src/flight_control.cpp](../src/navigation_2d/src/flight_control.cpp) | C17／C17／C17 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/src/shoot_with_move_base.cpp](../src/navigation_2d/src/shoot_with_move_base.cpp) | C01／C02／C02／C02／C03／C04／C05／C05／C06／C07／C07／C08／C09／C10／C10／C11／C12／C12／C13／C14／C14／C15／C15／C16／C16／V05／R14 | 有实质文本／注释差异，按条目细分 |
| [src/navigation_2d/test/dwa_control_math_test.cpp](../src/navigation_2d/test/dwa_control_math_test.cpp) | T08 | pack无同路径 |
| [src/navigation_2d/test/ego_path_tracking_test.cpp](../src/navigation_2d/test/ego_path_tracking_test.cpp) | T01 | pack无同路径 |
| [src/navigation_2d/test/gtest_main.cpp](../src/navigation_2d/test/gtest_main.cpp) | T08 | pack无同路径 |
| [src/navigation_2d/test/yolo_tensor_layout_test.cpp](../src/navigation_2d/test/yolo_tensor_layout_test.cpp) | T08 | pack无同路径 |
| [src/servo_control/launch/servo_bench.launch](../src/servo_control/launch/servo_bench.launch) | L13 | 有实质文本／注释差异，按条目细分 |
| [src/servo_control/script/servo_debug.py](../src/servo_control/script/servo_debug.py) | 学长pack原脚本导入；非我的交互角度算法 | 与pack字节相同 |
| [src/servo_control/src/servo_controller_node.cpp](../src/servo_control/src/servo_controller_node.cpp) | L13 | 有实质文本／注释差异，按条目细分 |
| [src/usb_cam/.github/workflows/main.yml](../src/usb_cam/.github/workflows/main.yml) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/.gitignore](../src/usb_cam/.gitignore) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/.travis.yml](../src/usb_cam/.travis.yml) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/AUTHORS.md](../src/usb_cam/AUTHORS.md) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/CHANGELOG.rst](../src/usb_cam/CHANGELOG.rst) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/CMakeLists.txt](../src/usb_cam/CMakeLists.txt) | R01–R04；上游框架＋我的接口／媒体构建适配 | pack无同路径 |
| [src/usb_cam/LICENSE](../src/usb_cam/LICENSE) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/README.md](../src/usb_cam/README.md) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/UPSTREAM.md](../src/usb_cam/UPSTREAM.md) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/include/usb_cam/rockchip_mjpeg.h](../src/usb_cam/include/usb_cam/rockchip_mjpeg.h) | R01–R04；上游框架＋我的接口／媒体构建适配 | pack无同路径 |
| [src/usb_cam/include/usb_cam/usb_cam.h](../src/usb_cam/include/usb_cam/usb_cam.h) | R01–R04；上游框架＋我的接口／媒体构建适配 | pack无同路径 |
| [src/usb_cam/launch/rk_usb_camera.launch](../src/usb_cam/launch/rk_usb_camera.launch) | R02 | pack无同路径 |
| [src/usb_cam/launch/usb_cam-test.launch](../src/usb_cam/launch/usb_cam-test.launch) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/mainpage.dox](../src/usb_cam/mainpage.dox) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/nodes/usb_cam_node.cpp](../src/usb_cam/nodes/usb_cam_node.cpp) | R04 | pack无同路径 |
| [src/usb_cam/package.xml](../src/usb_cam/package.xml) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/src/LICENSE](../src/usb_cam/src/LICENSE) | 第三方usb_cam0.3.7原件／元数据，非我的算法；R01说明导入 | pack无同路径 |
| [src/usb_cam/src/rockchip_mjpeg.cpp](../src/usb_cam/src/rockchip_mjpeg.cpp) | R02／R02 | pack无同路径 |
| [src/usb_cam/src/usb_cam.cpp](../src/usb_cam/src/usb_cam.cpp) | R01／R01／R03／R04 | pack无同路径 |
| [src/usb_cam/test/lifecycle_test.cpp](../src/usb_cam/test/lifecycle_test.cpp) | R01／T03；我的重复shutdown测试 | pack无同路径 |
| [tools/70-uav-media.rules](../tools/70-uav-media.rules) | R02；媒体设备访问权限规则，无控制算法 | pack无同路径 |
| [tools/analyze_px4_takeoff.py](../tools/analyze_px4_takeoff.py) | T15 | pack无同路径 |
| [tools/build_rk_media_runtime.sh](../tools/build_rk_media_runtime.sh) | T17 | pack无同路径 |
| [tools/check_doc_links.ps1](../tools/check_doc_links.ps1) | T18 | pack无同路径 |
| [tools/compare_fourclass_probe.py](../tools/compare_fourclass_probe.py) | T05 | pack无同路径 |
| [tools/deployment-flight-logging.json](../tools/deployment-flight-logging.json) | R14／T17；候选版本清单，无控制算法 | pack无同路径 |
| [tools/doc_links_known_missing.json](../tools/doc_links_known_missing.json) | T18 | pack无同路径 |
| [tools/dwa_mission_probe.py](../tools/dwa_mission_probe.py) | T07 | pack无同路径 |
| [tools/dwa_planner_probe.py](../tools/dwa_planner_probe.py) | T06 | pack无同路径 |
| [tools/elf_dwarf_layout.py](../tools/elf_dwarf_layout.py) | T17 | pack无同路径 |
| [tools/fetch_dwa_deps.py](../tools/fetch_dwa_deps.py) | T17 | pack无同路径 |
| [tools/finalize_uav_lite.py](../tools/finalize_uav_lite.py) | T17 | pack无同路径 |
| [tools/finalize_uav_logging.py](../tools/finalize_uav_logging.py) | T17 | pack无同路径 |
| [tools/foxglove_wire_check.py](../tools/foxglove_wire_check.py) | T12 | pack无同路径 |
| [tools/inspect_1025_packages.py](../tools/inspect_1025_packages.py) | T16 | pack无同路径 |
| [tools/inspect_rk_firmware.py](../tools/inspect_rk_firmware.py) | T21 | pack无同路径 |
| [tools/install_uav_commands.sh](../tools/install_uav_commands.sh) | R09 | pack无同路径 |
| [tools/lichtblick_readonly_observe.py](../tools/lichtblick_readonly_observe.py) | T12 | pack无同路径 |
| [tools/monitor_device_handles.py](../tools/monitor_device_handles.py) | T12 | pack无同路径 |
| [tools/monitor_jpeg_probe.py](../tools/monitor_jpeg_probe.py) | T19 | pack无同路径 |
| [tools/package_sim_handoff.py](../tools/package_sim_handoff.py) | T16 | pack无同路径 |
| [tools/prepare_fourclass_cases.py](../tools/prepare_fourclass_cases.py) | T05 | pack无同路径 |
| [tools/render_monitor_guide.cjs](../tools/render_monitor_guide.cjs) | T18 | pack无同路径 |
| [tools/rk-usb-ssh.service](../tools/rk-usb-ssh.service) | T22 | pack无同路径 |
| [tools/rk_application_loopback_test.py](../tools/rk_application_loopback_test.py) | T01 | pack无同路径 |
| [tools/rk_build_cdc_acm.sh](../tools/rk_build_cdc_acm.sh) | T17 | pack无同路径 |
| [tools/rk_camera_lifecycle_check.py](../tools/rk_camera_lifecycle_check.py) | T03 | pack无同路径 |
| [tools/rk_dependency_audit.sh](../tools/rk_dependency_audit.sh) | T17 | pack无同路径 |
| [tools/rk_dwa_readonly_stack_check.py](../tools/rk_dwa_readonly_stack_check.py) | T09 | pack无同路径 |
| [tools/rk_fourclass_camera_check.py](../tools/rk_fourclass_camera_check.py) | T05 | pack无同路径 |
| [tools/rk_fourclass_probe.cpp](../tools/rk_fourclass_probe.cpp) | T05 | pack无同路径 |
| [tools/rk_i2c_acceptance.py](../tools/rk_i2c_acceptance.py) | T10 | pack无同路径 |
| [tools/rk_kernel_export_check.py](../tools/rk_kernel_export_check.py) | T17 | pack无同路径 |
| [tools/rk_mavros_loopback_test.py](../tools/rk_mavros_loopback_test.py) | T01 | pack无同路径 |
| [tools/rk_media_compare.cpp](../tools/rk_media_compare.cpp) | T03 | pack无同路径 |
| [tools/rk_model_metadata.cpp](../tools/rk_model_metadata.cpp) | T05 | pack无同路径 |
| [tools/rk_npu_autostart.py](../tools/rk_npu_autostart.py) | P03／P03 | pack无同路径 |
| [tools/rk_npu_boot.sh](../tools/rk_npu_boot.sh) | P02／P02 | pack无同路径 |
| [tools/rk_npu_power.py](../tools/rk_npu_power.py) | P01／P01／P01 | pack无同路径 |
| [tools/rk_npu_usb.py](../tools/rk_npu_usb.py) | P04／P04 | pack无同路径 |
| [tools/rk_pca_disable.py](../tools/rk_pca_disable.py) | T10 | pack无同路径 |
| [tools/rk_peripheral_bench_check.py](../tools/rk_peripheral_bench_check.py) | T03 | pack无同路径 |
| [tools/rk_px4_serial_shell.py](../tools/rk_px4_serial_shell.py) | T02 | pack无同路径 |
| [tools/rk_px4_usb_diagnostics.cpp](../tools/rk_px4_usb_diagnostics.cpp) | T02 | pack无同路径 |
| [tools/rk_ros_loopback_test.py](../tools/rk_ros_loopback_test.py) | T01 | pack无同路径 |
| [tools/rk_run_validation.sh](../tools/rk_run_validation.sh) | T01 | pack无同路径 |
| [tools/rk_servo_bench_check.py](../tools/rk_servo_bench_check.py) | T11 | pack无同路径 |
| [tools/rk_sixclass_probe.cpp](../tools/rk_sixclass_probe.cpp) | T03 | pack无同路径 |
| [tools/rk_usb_ssh.sh](../tools/rk_usb_ssh.sh) | T22 | pack无同路径 |
| [tools/rk_wireless_observe.py](../tools/rk_wireless_observe.py) | T12 | pack无同路径 |
| [tools/rollback_uav_logging.py](../tools/rollback_uav_logging.py) | T17 | pack无同路径 |
| [tools/run_uav_logging_verification.sh](../tools/run_uav_logging_verification.sh) | T17 | pack无同路径 |
| [tools/shell-npu-firmware.service](../tools/shell-npu-firmware.service) | P03；我的systemd时序／故障／停止策略 | pack无同路径 |
| [tools/shell-npu-power.service](../tools/shell-npu-power.service) | P03；我的systemd时序／故障／停止策略 | pack无同路径 |
| [tools/shell-npu-proxy.service](../tools/shell-npu-proxy.service) | P03；我的systemd时序／故障／停止策略 | pack无同路径 |
| [tools/shell-npu.target](../tools/shell-npu.target) | P03 | pack无同路径 |
| [tools/sim_handoff/Dockerfile.base](../tools/sim_handoff/Dockerfile.base) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/README_先读.md](../tools/sim_handoff/README_先读.md) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/apt-common.txt](../tools/sim_handoff/apt-common.txt) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/apt-ros-common.txt](../tools/sim_handoff/apt-ros-common.txt) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/apt-simulation.txt](../tools/sim_handoff/apt-simulation.txt) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/inspect_host.ps1](../tools/sim_handoff/inspect_host.ps1) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/inspect_linux.sh](../tools/sim_handoff/inspect_linux.sh) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/requirements-yolo-cpu.txt](../tools/sim_handoff/requirements-yolo-cpu.txt) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/verify_package.py](../tools/sim_handoff/verify_package.py) | T16 | pack无同路径 |
| [tools/sim_handoff/另一台电脑执行提示词.md](../tools/sim_handoff/另一台电脑执行提示词.md) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/sim_handoff/环境与外部依赖.md](../tools/sim_handoff/环境与外部依赖.md) | T16；异机环境／依赖／检查配置，不是机载算法 | pack无同路径 |
| [tools/start_lichtblick_monitor.sh](../tools/start_lichtblick_monitor.sh) | T12 | pack无同路径 |
| [tools/test_rk_npu_autostart.py](../tools/test_rk_npu_autostart.py) | T20 | pack无同路径 |
| [tools/test_rk_npu_power.py](../tools/test_rk_npu_power.py) | T20 | pack无同路径 |
| [tools/test_rk_npu_usb.py](../tools/test_rk_npu_usb.py) | T20 | pack无同路径 |
| [tools/test_uav_arm.py](../tools/test_uav_arm.py) | T20 | pack无同路径 |
| [tools/test_uav_log_lite.py](../tools/test_uav_log_lite.py) | T20 | pack无同路径 |
| [tools/test_uav_logging.py](../tools/test_uav_logging.py) | T20 | pack无同路径 |
| [tools/uav](../tools/uav) | R09／R11／R12／R15；入口／安装 | pack无同路径 |
| [tools/uav_arm.py](../tools/uav_arm.py) | R12／R12／R12 | pack无同路径 |
| [tools/uav_env.sh](../tools/uav_env.sh) | R09 | pack无同路径 |
| [tools/uav_lite_benchmark.py](../tools/uav_lite_benchmark.py) | T13 | pack无同路径 |
| [tools/uav_lite_compare.py](../tools/uav_lite_compare.py) | T13 | pack无同路径 |
| [tools/uav_lite_integration.py](../tools/uav_lite_integration.py) | T13 | pack无同路径 |
| [tools/uav_log.py](../tools/uav_log.py) | R13 | pack无同路径 |
| [tools/uav_log_lite.py](../tools/uav_log_lite.py) | R15／R15／R16／R16／R16／T14 | pack无同路径 |
| [tools/uav_log_report.py](../tools/uav_log_report.py) | R13／T14 | pack无同路径 |
| [tools/uav_logging_benchmark.py](../tools/uav_logging_benchmark.py) | T13 | pack无同路径 |
| [tools/uav_logging_compare.py](../tools/uav_logging_compare.py) | T13 | pack无同路径 |
| [tools/uav_logging_fault_probe.py](../tools/uav_logging_fault_probe.py) | T13 | pack无同路径 |
| [tools/uav_perf.py](../tools/uav_perf.py) | R13 | pack无同路径 |
| [tools/uav_start.py](../tools/uav_start.py) | L14／R10／R10／R10／R11／R11／R11／R15 | pack无同路径 |
| [tools/uav_start_gate_probe.py](../tools/uav_start_gate_probe.py) | T19 | pack无同路径 |
| [tools/verify_lichtblick_runtime.py](../tools/verify_lichtblick_runtime.py) | T12 | pack无同路径 |
| [tools/verify_rk_sixclass.py](../tools/verify_rk_sixclass.py) | T03 | pack无同路径 |
| [tools/verify_uav_logging_rollback.sh](../tools/verify_uav_logging_rollback.sh) | T17 | pack无同路径 |
| [tools/verify_yolo_delivery.py](../tools/verify_yolo_delivery.py) | T04 | pack无同路径 |
| [tools/wifi_ground_probe.py](../tools/wifi_ground_probe.py) | T19 | pack无同路径 |

覆盖179个净变化文件，全部有分类，未用“构建通过”替代算法来源判断。

### 11.3 src／tools以外的可执行算法来源

新增的yolov5training六类返回包脚本逐项与原交回ZIP比较，以下均字节相同；它们属于异机交回代码的归档，不归为我在此线程新写。下表保留文件和SHA，避免只因同一Git提交人而归错来源。这里不执行训练／转换脚本。

| 交回脚本 | 本地SHA256（与原ZIP相同） |
|---|---|
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/01_extract_audit.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/01_extract_audit.py) | `f19c9b0489e380ee0c1627cac74d5b63db6f94f6220e63737112b12a67146221` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/02_build_dataset.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/02_build_dataset.py) | `c54352787e54da47546fb8c770a99825421a4b105fc77d72d2dd7b75c2af6986` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/03_make_calib.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/03_make_calib.py) | `356eb4cce9eccaef62592f2e4a88f4b91ea0229953267af465b8a573bdf20548` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/05_eval_test.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/05_eval_test.py) | `9b35fa44be7e0dbe6365061a4d36d5be9aaa2ce9d8412d2cada77aeed9fb0841` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/06_infer_onnx.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/06_infer_onnx.py) | `8921976dfe026a2f2ab41c7ff1a8a806248f225957a032ac389ad4ea674ed052` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/07_rknn_convert.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/07_rknn_convert.py) | `08b4dea16217737f30fb47cbb936eb32a96135ab42df28ec4d79a935bafda209` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/08_rknn_compare.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/08_rknn_compare.py) | `c9d583ff3f9667f0fe80c4af7ee0a60e17bcf0d9a45104b48c4585ee4a343a35` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/09_assemble_deliverables.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/09_assemble_deliverables.py) | `be0256d9a67e2d6e188d3ff63d8b66f3fd2a463d93acb87cee18161725000d78` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/remote_zip_extract.py](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/remote_zip_extract.py) | `685dd8eb1c237b5822833821fc8e424d7f5710df96c9d94c530c9077ffa6f877` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp1_warm.bat](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp1_warm.bat) | `c90a95ebba6f8a9a8729fb80890f492ce3cb63d1237663136897f31b9866544a` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp2_resume.bat](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp2_resume.bat) | `535dc0a639c53c89a876d00e5be5f1b825b971601b0a71d687d069e6982fb80a` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp2_scratch.bat](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp2_scratch.bat) | `a1398613119257af0cd0390b9d49f6df0d4a3da402693fc5e3b41a5d224bbac2` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp3_rot.bat](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_exp3_rot.bat) | `2765b836b18a0e3dd19c2660dcb63f29ac8b7233580095681e5058e76b1cb15c` |
| [yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_smoke.bat](../yolov5training/YOLO异机训练交付_2026-10-24/YOLO异机训练交付_2026-10-24/scripts/run_smoke.bat) | `f8a4e2982c792dea7ea82402e5209c81c8f96873acc22e654eef7b784b80007a` |

该目录其余模型／数据划分／指标／配置为交回载荷；另两份NPU参考说明为来源文档。范围外根目录净变化为.gitignore、AGENTS.md以及文档入口，不含额外被遗漏的机载算法。恢复的docs/监看三份历史桥配置已按10月13日原SHA恢复，不是本日期范围新算法。docs/验证记录中的原始代码／测试脚本是证据副本或隔离工具，跟其对应E/T条目，不是正式部署入口。

## 12. 固定版本指纹与部署差异

### 12.1 关键当前文件／原包／已取回板端源码

这里只用已保存的该次事故后取证副本核对，**没有重新SSH读取当前板端**。记录为“取证包未含”不等于板端不存在。源码SHA相同也仍需正确可执行文件／入口才能说明运行版本。


**[src/navigation_2d/src/shoot_with_move_base.cpp](../src/navigation_2d/src/shoot_with_move_base.cpp)**

- 审查工作树：`990de208092c28214cc91b4d7ae7b029eb412fb26eb920508bf5fa20f1e7aa70`。
- 学长原包：`03315e7eaf14deef28203221fbe72a86f5cc3a2ab3939ca36f420bc99a1d2c2f`；有实质文本／注释差异，按条目细分。
- 16:40后取证板端副本：`01eab407cc619db1be18f5689dff54a35123c708528810462d7591d2e1c9577b`；与当前不同，不能拿当前整文件替代事故源码。

**[src/location/src/location_with_ftmini.cpp](../src/location/src/location_with_ftmini.cpp)**

- 审查工作树：`f46056759da3a5e1a107cdd67f6aeb4218b02e6c8c3732a186e283818d199e0b`。
- 学长原包：`0e76ccb75b5db05b6d5f2b48d8a941bc1e5f5a93455a8e5fa549c3aef8af47ff`；有实质文本／注释差异，按条目细分。
- 16:40后取证板端副本：`7017f0415f5395c5f15bf90f0f9124e47e5bd8d885e7c1d4bd8b19a3c9e61a2c`；与当前不同，不能拿当前整文件替代事故源码。

**[src/navigation_2d/include/dwa_control_math.h](../src/navigation_2d/include/dwa_control_math.h)**

- 审查工作树：`7788eb579fe00b54c10158da2e3bf7607b50da029549ca55461f58d62bca295f`。
- 16:40后取证板端副本：`7788eb579fe00b54c10158da2e3bf7607b50da029549ca55461f58d62bca295f`；与当前相同。

**[src/location/include/localization_guard.h](../src/location/include/localization_guard.h)**

- 审查工作树：`622bf50b53dc9d3c52318d538ab431763716faaccda1ea3b216e6f4cebbd4f8d`。
- 16:40后取证板端副本：`622bf50b53dc9d3c52318d538ab431763716faaccda1ea3b216e6f4cebbd4f8d`；与当前相同。

**[src/navigation_2d/include/dwa_mission_policy.h](../src/navigation_2d/include/dwa_mission_policy.h)**

- 审查工作树：`03edc6801365b86c358c6e58fa23638b21217c7f85b62a77d0040f10c36a437c`。
- 16:40后取证板端副本：`03edc6801365b86c358c6e58fa23638b21217c7f85b62a77d0040f10c36a437c`；与当前相同。

**[src/navigation_2d/src/flight_control.cpp](../src/navigation_2d/src/flight_control.cpp)**

- 审查工作树：`aa11b3f4eb4e8387630779e507f404136865a23c1e7ca33100d46e7e6b9418e0`。
- 学长原包：`59aeae04958df3332fd9ad7ad17201fb8cdd6ab9be2b14e41bf02ed7306a55a7`；有实质文本／注释差异，按条目细分。
- 16:40后取证板端副本：`aa11b3f4eb4e8387630779e507f404136865a23c1e7ca33100d46e7e6b9418e0`；与当前相同。

**[src/navigation_2d/src/detect.cpp](../src/navigation_2d/src/detect.cpp)**

- 审查工作树：`72348798d5cf374459cd3b63dd3dc45c886d71c64a80de7fa7b88b0c97fc66b1`。
- 学长原包：`1a030403b3537a0e53ee1cf34e21ee2fb99841850ddb40f6e590cc9f0199cf71`；有实质文本／注释差异，按条目细分。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[src/navigation_2d/include/yolo_tensor_layout.h](../src/navigation_2d/include/yolo_tensor_layout.h)**

- 审查工作树：`8ceaab7440c6eb5b8df351ca2351e9b197e971d5e818feb9a713abab65bb0b7e`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[src/navigation_2d/param/dwa_local_planner_params.yaml](../src/navigation_2d/param/dwa_local_planner_params.yaml)**

- 审查工作树：`d904470e9bde6f712866953fa49662c54dc1ce7b7df401aea88fc2deceb3ad9b`。
- 学长原包：`1a1e586cc9aa507bb3e9b86fe073e031842dd1e5659fe93dd64eb998f548bcab`；有实质文本／注释差异，按条目细分。
- 16:40后取证板端副本：`d904470e9bde6f712866953fa49662c54dc1ce7b7df401aea88fc2deceb3ad9b`；与当前相同。

**[src/location/param/rk_height_reference_20261027.yaml](../src/location/param/rk_height_reference_20261027.yaml)**

- 审查工作树：`30c0e1414577a624bafd4e1c8a180f9f945c42d692594ee64ea7b7d56e19e548`。
- 16:40后取证板端副本：`30c0e1414577a624bafd4e1c8a180f9f945c42d692594ee64ea7b7d56e19e548`；与当前相同。

**[src/location/src/tf_mini_i2c.cpp](../src/location/src/tf_mini_i2c.cpp)**

- 审查工作树：`83c1dd3dbdde01b285bd796876fba00e0c5cd201926ee14d99463941511b4ede`。
- 学长原包：`7d9141498d2f86ff1e559b751208e0c4b4f14538c4b9396a0e5e1e71ea366ffb`；有实质文本／注释差异，按条目细分。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[src/location/src/TFplus.cpp](../src/location/src/TFplus.cpp)**

- 审查工作树：`adc87f42c146d588636f4df319d347f2653cca5e557d63cba01a803373912f95`。
- 学长原包：`8ecd5b23082167e08a327e8528373c3acae458d0a01fdc7987f3b2b6ff568a0d`；有实质文本／注释差异，按条目细分。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[src/location/param/ROSSLAM/cartographer_2Dlidar_mapping_location_params.lua](../src/location/param/ROSSLAM/cartographer_2Dlidar_mapping_location_params.lua)**

- 审查工作树：`6c58aabb669ccee6cf3accc28c4f5b7860f90fb7bbdba7cf58c3ac233ce51ff9`。
- 学长原包：`6c58aabb669ccee6cf3accc28c4f5b7860f90fb7bbdba7cf58c3ac233ce51ff9`；与pack字节相同。
- 16:40后取证板端副本：`6c58aabb669ccee6cf3accc28c4f5b7860f90fb7bbdba7cf58c3ac233ce51ff9`；与当前相同。

**[src/location/scripts/monitor_jpeg.py](../src/location/scripts/monitor_jpeg.py)**

- 审查工作树：`e47db19089e04e0405cdfe40db3c3bb5081ef4563eb35d96d184cc41f7016c84`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[tools/uav_start.py](../tools/uav_start.py)**

- 审查工作树：`75f77b1b1b61a3c1d1267da2104988b3606fedf320b5a91dda12c2adbb8f4d50`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[tools/uav_arm.py](../tools/uav_arm.py)**

- 审查工作树：`b3695e0eac7d76c81a1f2d1ada17a5c1453fd767d425e76d8e2914777991b76a`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[tools/uav_log_lite.py](../tools/uav_log_lite.py)**

- 审查工作树：`ddc75a7a298082bae3fa16ea869c67925678b81bedf5eab3739cd4471f271f92`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

**[src/usb_cam/src/rockchip_mjpeg.cpp](../src/usb_cam/src/rockchip_mjpeg.cpp)**

- 审查工作树：`823d66bf6452887f44374d90d4209f759fefe14be1b96ac40ad4b9c498a0daea`。
- 此取证源码子包未包含该文件，部署依据另看对应E报告／回退清单。

### 12.2 原件和实际入口补充

- 学长任务原文与temp1025解压文件SHA均为`03315e7eaf14deef28203221fbe72a86f5cc3a2ab3939ca36f420bc99a1d2c2f`；不靠源文件里的Author注释定来源。
- 16:40后实际入口曾从`~/.local/share/uav/`而非仅`~/catkin_ws/tools/`执行；R10修复后start为`75f77b1b1b61a3c1d1267da2104988b3606fedf320b5a91dda12c2adbb8f4d50`、arm为`b3695e0eac7d76c81a1f2d1ada17a5c1453fd767d425e76d8e2914777991b76a`；已取证实际task二进制见§2。
- 原始ROS取证tar为`734690486d4a9a56fdc747f6b45dd5a86f80ace0d8fc773be96aeba7ccd54018`；部署源码tar为`08e084cd0f6b220e601d43119c5d498907b7c396df06a45a23ad2ee12a49694f`。原件都在recordings/incident_20261028_1640，Git忽略，不因此修改／重压它们。
- 229 ULog SHA `27bb702c79a0b6af96908c9373a82d731356b341f1b1a687ecce5daf3629dae8`；230 SHA `a1f9e0aa044f0482ade01fe9a72fe77a37c88567bfc0b23c71992882f1dd0b30`。板端日期错误，需按boot／事件／原路径关联，不按文件日期猜架次。

## 13. 复核方法、未完成证据与责任边界

下列仅为本地读取命令，不需要启动无人机。先看原始差异，再决定是否改回某项，不在本轮批量回滚。

```powershell
git show 18cd805 -- src/navigation_2d/src/shoot_with_move_base.cpp
git show a965a7c -- src/navigation_2d/param/dwa_local_planner_params.yaml
git show a765be7 -- src/location/src/tf_mini_i2c.cpp
git show e4882bd -- src/navigation_2d/src/detect.cpp
git show 28a178e -- tools/uav_start.py tools/uav_arm.py
git diff --no-index -- temp1025/inspection_20261025/pack/src/navigation_2d/src/shoot_with_move_base.cpp src/navigation_2d/src/shoot_with_move_base.cpp
```

`git diff --no-index`发现差异时退出1是正常差异状态，不是分析失败。最早b1cbd46“只改Timer生命期”的历史版本后来被迁移覆盖，应使用git show看当时diff。

仍缺实机定向平移／航向转动、动态高度真值、map与PX4 local物理轴闭合、真实桨叶包络与墙距、飞行时完整ROS控制输入输出。当前损坏条件不能补这些动态验收。10月28日晚首轮失联仍缺旧boot持久日志，未归因也未宣布修复。

我负责承认并逐项列出上面的新增／重写／调参及曾经的错误：不能把数学上看起来合理的工程修正说成学长原始实现，也不能用模拟通过排除它与事故有关；反过来也不把没有因果证据的改动直接判为撞墙根因。

本文按SHELL指定路径保存。temp1028原为Git忽略暂存区；本次仅将uav_suspect.md作为SHELL指定的人工审查记录单独保留版本，不解除整个temp目录忽略，也不把原始取证包／其它临时内容加入仓库。

**后续交付决定覆盖上段初始暂存规则：** SHELL随后明确要求直接更新main，且temp1028不要ignore、其它temp继续ignore。现以根目录`/temp*/`配合`!/temp1028/`执行，temp1028当前7份文件全部纳入；其它temp目录继续忽略。正文的36提交／179文件是截至ccbb548的算法审查范围，不包含本次文档交付与忽略规则调整。
