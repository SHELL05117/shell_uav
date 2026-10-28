# 2026-10-22 工程仓库与Docker／WSL配置

[返回项目目录](低空消防无人机工程日志.md)｜[任务状态](../docs/项目资料/任务状态.md)｜[执行计划](../docs/项目资料/执行计划.md)｜[前一日：2026-10-21](2026-10-21_RK板采购台账与迁移评估.md)

## 目标、日期与记录边界

SHELL于2026-10-22要求把[2026-10-21工程日志](2026-10-21_RK板采购台账与迁移评估.md)中原编号13及以后的内容移入本日志并重新编号。本日志因此承接该文件的原§13–§21，依次重编为§1–§9；正文、时间戳、证据链接和截图路径按原样保留（截图仍在 `附件/2026-10-21/`，原文件名与字节不变）。2026-10-21日志保留原§1–§12，其中§12为10月22日凌晨按08:00规则归入的内容。

本日志记录2026-10-22的版本控制基线、部署依赖分类、当前电脑Docker＋WSL配置与桌面启动修复。各项验收只以文中实际输出为准；文中未写明的编译、ROS运行、仿真和硬件操作均未执行。

后续新增的学长固件核查、Docker仿真答疑与RK板报销反馈亦记录于本日志§11–12；SHELL本轮要求主分支新工作统一归入2026-10-22。

## 1. 总技术指南更新到 1018 并建立项目仓库

SHELL要求：版本号更新为 1018，Git 范围扩大到 `uav1018` 全部内容，并做一次提交。

**文档：** [总技术指南](../docs/技术文档.md) 由 420 行重写到 613 行，整体对齐 version1018 —— 7 个包、11 个可执行目标（10 无条件＋1 条件）、11 类模型（32,193,674 字节、PyTorch 2.4.1、opset 17/IR 8、`scores[1,3000,11]`）、无 DWA/TEB 与 `planner_backend`、目标话题 `/mission/route_goal`（odom）、GStreamer 直采、唯一回传图 `/ssd_detection/image/compressed`、TFmini 门限与 `node_start_delay` 3 s，并新增第 14 节 Gazebo 仿真与设备分工。删除旧"六包/9 目标/5 类/`[1,1917,5]`/tf2onnx/opset11/usb_cam 默认链"与"Git 仅本地、另建分支标签"等失效表述。原文的学长起飞指南、WSL/Docker 评估、Lichtblick 与证据缺口章节保留并按 1018 更新。同时把各文档中"项目没有 Git 仓库"的表述改为"2026-10-22 起项目根已有仓库"。

**仓库：** 在项目根 `git init -b main`，局部身份 `SHELL Liz <shell@localhost>`；`.gitattributes` 设 `* -text` 关闭换行自动转换，保持文件原字节；`src/.git` 为空目录已清除；纳入 `docs/ logs/ src/ tools/ nanooutput_origin/` 与根目录文件共 381 条目、约 71 MB（含 30.7 MB ONNX 与 20.6 MB TensorRT engine，未设忽略规则）。完成首次提交；无远端、未推送。

**工具：** 本机 `git` 不在 PATH，使用便携版 `D:\Programs\Workbuddy\resources\vendor\PortableGit\cmd\git.exe`（2.55.0.windows.3）。

**边界：** 本轮只改文档与版本控制，未改动任何源码、模型或原始归档的字节，未编译、未运行、未连接设备。

## 2. 多人协作忽略规则与 uav_version1018 基线整理

**目标与授权：** SHELL询问为 push 和多人协作应排除哪些文件，随后明确要求按建议创建 ignore、重新提交并修改 `AGENTS.md`，并指定提交标签为 `uav_version1018`。本轮将提交说明和 Git 标签均设为该名称。检查时 `main` 只有首次提交 `13d2c6a662033de8453c53d1615321ed74ed2ccf`，工作区干净、无远端、无标签，尚未推送。

**操作：** 新增根目录 [`.gitignore`](../.gitignore)，覆盖 catkin/CMake 输出、Python 缓存及虚拟环境、TensorRT engine/plan、生成数据集与训练检查点、原始录制、完整备份/固件下载目录、个人配置、临时文件及本地凭据。将已跟踪的9个 `.pyc` 和1个 TensorRT engine 仅从索引移除，保留本地原文件；修订尚未共享的首次提交，避免上述产物仍随父提交历史上传。原先§1的381条目、未设忽略规则为整理前状态。

**保留范围：** 源码、launch/参数/消息、ONNX/标签/校验文件、URDF/Gazebo 场景及素材、`tools/assets/`、技术文档、人工 `logs/` 与必要截图，以及 `nanooutput_origin/` 当前跟踪的清单、环境文字报告和核验记录。未整体忽略 `logs/` 或 `nanooutput_origin/`；共享 VS Code 配置与不含凭据的环境示例允许提交。[AGENTS.md](../AGENTS.md) 已补充提交边界、跟踪取消方式及协作约定，并同步本日志索引、任务状态和开发规范的当前基线说明。

**核验与实际结果：** 37条应忽略路径与22条应保留路径的匹配检查全部通过；现有文件中只排除了上述10个本地产物，索引不再含被忽略的文件，共保留372个条目（381−10＋新增 `.gitignore`）。10个本地原文件的大小与SHA256均与操作前一致；`src/`、`tools/`、`nanooutput_origin/` 的暂存差异仅为这10项取消跟踪，源码、ONNX和归档记录无字节或权限变更。`.gitattributes` 未改动，`git diff --cached --check` 通过。

**提交方式：** 在 `main` 上修订唯一未推送的根提交，提交说明为 `uav_version1018`，并创建同名附注标签；完成后核对标签指向 `HEAD`、分支/标签可达历史不含 `.pyc` 或 engine，且工作区干净。旧提交可能仍由本地 reflog 保留用于恢复，本轮不清理本地 Git 对象。

**边界与下一步：** 本轮仅调整版本控制和文档；未修改源码、模型、原归档或缓存文件的字节，未编译、未运行、未连接设备，也没有配置远端或执行 push。SHELL提供远端目标后再接续成员权限和首次推送；Git 忽略规则不代表已经完成凭据或附件授权的全面审查。

## 3. Ubuntu18.04平台反馈与部署依赖分类

**目标与最新前提：** SHELL说明当前项目运行平台为RK3399ProD上的Ubuntu18.04，要求阅读文档后区分复制程序后可不安装、联网可自动处理、以及需要手动或apt安装的依赖。本次以该平台反馈更新接续前提；此前“待到货、优先核验Debian”的文字保留为当时快照，不继续覆盖最新说明。ROS发行版、板端已装软件、镜像来源及现场代码版本未取得只读输出。

**操作与结果：** 阅读项目导航、任务状态、总技术指南、迁移／RKNN与编译分析文档，逐项核对当前七包CMake、package.xml、默认launch及Python检测导入；查阅ROS／rosdep、MAVROS与Rockchip官方资料。新增[Ubuntu18.04部署依赖清单](../docs/项目资料/RK3399ProD_Ubuntu18.04部署依赖清单_2026-10-21.md)，同步导航、任务状态与总指南入口。

**主要结论：** 复制源码、联网和catkin构建不会自动安装外部依赖；apt／rosdep需主动执行，且包清单存在遗漏。RPLIDAR SDK、EGO／L-BFGS和PCA9685控制代码已随项目提供；Jetson栈、训练环境、PC仿真和未启用的T265／GUI项可不在机载安装。默认机载链仍需兼容ROS、MAVROS、Cartographer、OpenCV、libi2c及构建所需Eigen等；MAVROS地理数据、Python3检测环境或旧代RKNN配套另核对。

**源码边界：** 本地 `use_usb_cam=true` 仍选择Jetson后端；Python检测入口只订阅ROS图像。RK运行不能仅靠安装库修复该入口，CPU／NPU检测和相机源需要实际适配。未把SHELL的平台说明扩大为本地基线全部已在RK验收。

**验证与下一步：** 本轮仅静态核对和文档维护；安装命令均为参考，未登录板端、未安装、未构建、未启动节点或执行硬件测试。后续用板端系统／ROS版本、包状态、NPU配套及实际构建／推理结果收敛最终安装清单。

## 4. 当前电脑Docker＋WSL配置：实测、错误定位与准备

**目标与授权：** SHELL要求按10月21日工程日志指导当前电脑的Docker＋WSL配置。先读导航、任务状态、当日日志、总技术指南及另一电脑配置要求，保持一套Docker Desktop／WSL2后端、先基础容器再ROS编译与仿真的顺序。

**实际操作与结果（真实时间2026-10-22 14:34–14:39，Asia/Shanghai）：** 只读检查确认Windows 11专业版25H2、26200.9457、i5-12400F（6核12线程）、约15.8GB内存、RTX3060Ti 8GB／驱动616.92；C／D／R盘余量约96.7／234.5／296GB。Docker命令及常见安装／数据路径不存在。WSL应用包2.6.3.0存在，但wsl查询提示安装损坏和 `Wsl/CallMsi/Install/REGDB_E_CLASSNOTREG`；VirtualMachinePlatform可选功能未安装。HypervisorPresent=True与CPU虚拟化布尔字段False同时存在，未据此宣布BIOS关闭或硬件不支持。

**普通可逆准备：** 确认账户原无 `.wslconfig` 后，新建 `C:\Users\shell\.wslconfig`，设置memory=6GB、processors=4、swap=2GB；后续构建初始并行2。通过Microsoft官方GitHub发布API核对稳定版3.0.3，下载x64 MSI到项目外 `D:\uav-dev\installers\wsl.3.0.3.0.x64.msi`（367,575,040字节）；SHA256 `ae977286974255bd26bf925f00f40f5692c6b2084155cbc0563c09aea091f211` 与发布资产一致，Microsoft Authenticode签名Valid。取消助手自身查询触发的交互提示，未执行自动修复。

**交付与下一步：** 新增[当前电脑配置实录](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)，含实测表、错误原文、官方依据、安装包核验及逐步命令；同步任务状态和总索引。当前助手终端非管理员，下一步由SHELL在管理员窗口启用VirtualMachinePlatform、运行已核验MSI，成功后保存工作并手动重启，再提交WSL版本／状态／发行版列表的实际输出。WSL通过后安装Docker Desktop、选择WSL2／Linux容器并做hello-world验收，后续才核验旧ROS镜像、依赖及Dockerfile。

**源码覆盖校正：** 原提示词8通用目标清单未计当前gc_2026_sim的2个C++目标，后续按当前CMake的10个无条件可执行目标＋1个Jetson条件目标及4个EGO静态库分项记录；仿真包可单列，不能把8目标通过写成当前七包完整重建。

**完成边界：** 本轮只执行机器检查、配置文件准备和官方安装包下载核验；未启用Windows系统功能、运行MSI、重启、安装Docker或独立Ubuntu，未运行容器、ROS编译、Gazebo／PX4或真实硬件。没有更改src、模型或历史归档。工作树原有的部署依赖文档等未提交改动保留；本轮未提交或推送Git。

## 5. SHELL重启后的WSL复核与Docker接续

**目标与反馈：** SHELL报告“已重启”，继续原Docker＋WSL配置任务。

**实际结果（2026-10-22 15:02–15:03，Asia/Shanghai）：** 查询到本次Windows启动时间14:47:24；WSL版本3.0.3.0、内核6.18.54.1-1、WSLg 1.0.79，默认版本2，WSLService为Running／Auto，VirtualMachinePlatform的InstallState=1。此前 `Wsl/CallMsi/Install/REGDB_E_CLASSNOTREG` 不再出现。发行版列表明确提示尚未安装发行版；WSL1可选功能未安装，相关提示不阻塞本次WSL2路线。资源配置仍为6GB／4逻辑处理器／2GB swap。

**继续操作：** 核对Docker官方安装文档、AMD64更新源和固定build的checksums，准备4.96.2（244353）安装包、当前用户安装／WSL2后端；在项目外创建 `D:\uav-dev\docker-data` 与 `D:\uav-dev\logs`。详见[配置实录§7](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)。只读预查ROS官方tag的OCI索引及linux/amd64子manifest，元数据可访问；未将其写成镜像拉取、apt源或ROS编译已验收。

**当前边界：** WSL运行时／系统功能准备已核查，WSL2虚拟机实际启动、Docker安装与hello-world仍按后续真实结果补记。未安装独立Ubuntu、Gazebo或GPU／RKNN工具链，未启动ROS、编译或连接硬件；源码和既有未提交改动保持。

**后续安装与实际验收（15:30安装完成，15:34核验）：** Docker官方AMD64安装包4.96.2／244353下载完成（650,925,488字节），SHA256 `ceb589a9871b52971b24aa65915192854e5391a43d3df1b551db89aac50f24fa` 与固定build的官方checksums一致，Docker Inc Authenticode签名Valid。普通权限按 `install --user --quiet --backend=wsl-2 --wsl-default-data-root=D:\uav-dev\docker-data` 安装，15:30:11退出0；安装路径为当前账户LocalAppData\Programs\DockerDesktop。随后启动应用，未使用accept-license参数或代为处理首次条款界面。

**结果：** CLI与Engine均为29.9.2，context为desktop-linux，服务端linux／amd64；docker-desktop发行版Running／WSL2，NCPU=4，MemTotal=6,216,560,640字节（约5.79GiB），swap=2GiB。D盘生成disk\docker_data.vhdx与main\ext4.vhdx。hello-world首次从旧终端拉取因凭据助手不在该进程PATH而退出127；确认安装器已更新用户PATH，仅在当前执行进程补入resources\bin后重试，退出0并输出Hello from Docker，未改写登录配置或移除凭据助手。镜像为linux／amd64，digest为 `sha256:11a3eb1597e864c51b3f56733cb2e8a90bf0c284bfc05f9444d81c558bc848aa`。

**证据与接续：** 项目外 `D:\uav-dev\logs` 保存安装输出、首次失败／重试日志、WSL及Docker核验JSON；详见[实录§8](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)。WSL读swap时出现localhost代理未镜像到NAT的提示，本次Docker Hub拉取成功；后续容器apt联网另验，没有擅改代理或网络模式。当前只宣布“基础容器通过”；下一阶段拉取固定ROS镜像、检查旧源与签名、建立Dockerfile／Linux命名卷及启动恢复方案，再逐目标编译。尚未安装ROS、Gazebo、GPU或RKNN工具链，未创建项目工作卷、运行项目launch或接入硬件。源码、模型、归档及原有未提交改动保留，本轮未提交或推送Git。

## 6. Docker Desktop启动报错排查与恢复（18:26–18:28）

**目标与反馈：** SHELL报告Docker Desktop启动报错，要求修复。先保留既有镜像、容器和D盘虚拟磁盘，检查当前状态与本地日志。

**证据：** 检查时无Desktop／backend进程，Desktop状态查询失败，docker version仅返回客户端、服务端null并报dockerDesktopLinuxEngine管道不存在；docker-desktop为Stopped／WSL2，WSLService及vmcompute正常。历史日志存在15:40前端缓存拒绝访问、15:45空闲关闭引擎和AbortError，但本次未取得原始窗口错误，不把历史记录直接定为根因。

**实际操作：** 18:28:01从现有当前用户安装位置启动Docker Desktop，并将工作目录设为安装目录。新日志显示18:28:06引擎恢复running；SHELL明确反馈“已正常进入主界面”。未进行重装、重置、缓存或VHDX删除，也未修改系统、代理、资源或凭据配置。

**验证与结果：** 18:28:51，Desktop状态running，Client／Server均29.9.2，服务端linux／amd64；docker-desktop为Running／WSL2。用 `--pull=never` 运行现有hello-world镜像退出0，两个D盘VHDX及原镜像保留。当前启动及基础容器恢复，根因尚未定，不将恢复等同永久消除故障。

**记录与下一步：** 细节见[配置实录§9](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)，项目外 `D:\uav-dev\logs` 保存 `docker-startup-repair-20261022.json` 及hello-world复测日志。同步任务状态与总索引；若复发，按当次窗口文字和新日志定位。没有上传诊断资料、修改源码或原归档、运行ROS／仿真、连接硬件、提交或推送Git；原有未提交文档及SHELL新增的temp1022目录保持。

## 7. Docker磁盘分配与挂载性能说明（18:31）

**目标：** SHELL询问磁盘分配、后续容器位置、是否跨盘挂载及性能影响。

**实际核查：** C／D／R均在Disk 0，即ZHITAI Ti600 1TB NVMe SSD；不是三块物理硬盘。D盘可用约231.8GiB。安装配置wslDefaultDataRoot明确为 `D:\uav-dev\docker-data`；数据VHDX文件约1.59GiB，运行环境ext4.vhdx约0.09GiB。程序位于C盘LocalAppData\Programs\DockerDesktop，自身配置／日志也占C盘；Docker内部根目录/var/lib/docker、驱动overlayfs，不能按该Linux路径误判为Windows C盘。当前只有hello-world镜像，0容器、0本地卷、0构建缓存，没有项目自定义绑定挂载。

**解释与后续安排：** 同一引擎后续容器的默认可写数据、镜像和普通本地命名卷沿用D盘数据虚拟盘，VHDX随数据增长，受D盘实际余量约束。按已有要求，Windows源码只读输入，复制／同步到Linux命名卷工作副本，卷内编译，少量日志导出。C／D盘符差异不是本机主要I/O问题；Windows NTFS与Linux共享边界上的大量小文件编译才需要规避，即使两者都位于D盘也如此。未做性能对照，不承诺固定速度或倍率。

**记录与边界：** 已补[实录§10](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)、任务状态及索引，附Docker／Microsoft官方存储依据。本轮只读核查和文档维护，未迁移磁盘、改分区／配额、创建卷／挂载或项目容器、编译或改源码；没有提交或推送Git。

## 8. Quit后重开注册键报错：快捷方式修复及冷启动回归（18:45–18:50）

**后续校正：本节为未解决复发的历史尝试。** SHELL再次反馈后，Computer Use实际双击仍复现同一错误；本节程序调用和COM另启进程没有覆盖普通桌面环境的差异，撤回“快捷方式工作目录已解决”的结论。最新证据与真实点击验收见§9。

**SHELL新证据与范围校正：** 每次Quit Docker Desktop后打开都会报错，并提供[原始截图](附件/2026-10-21/codex-clipboard-9883d731-4727-4c73-964b-57a47740226a.png)：getting backend binary path，找不到SOFTWARE\Docker Inc.\Docker Desktop注册键。截图原文件名／字节保留，复制哈希一致。此证据说明§6的恢复进程并没有消除复发，本轮按退出后重开重新验收。

**检查与判断：** 当前用户安装卸载项存在，InstallLocation为现有安装目录；产品键在HKCU／HKLM、32／64视图缺失。通过Windows Shell另启原生Windows PowerShell复查，结果一致。桌面与开始菜单快捷方式目标正确，但WorkingDirectory均为空；助手此前成功启动时显式指定过安装目录，启动条件并不相同。产品键缺失是实际证据，不将HKLM单独缺项作为当前用户安装异常的充分条件。

**实际修改：** 备份两份原始.lnk，将两者WorkingDirectory补为 `C:\Users\shell\AppData\Local\Programs\DockerDesktop`；目标及参数保持。备份、对应路径及修复JSON保存在项目外 `D:\uav-dev\logs\docker-launcher-fix-20261022`。未手工构造AppPath等注册值，未改应用二进制、重装、重置或清空D盘数据。

**验证：** 18:48:40经Windows Shell打开修正后的桌面入口，冷启动及服务端版本查询成功。确认无运行中的容器后，执行正常Desktop stop，18:49:34退出0，并确认前后台进程全部结束。18:49:55直接经Windows Shell打开修正后的开始菜单入口，未预启backend，Desktop状态running、Client／Server均29.9.2、linux／amd64。18:50:04运行已有hello-world（--pull=never）退出0。SHELL随后确认“窗口正常，没有弹窗”。

**完成边界：** 本机两个快捷方式入口的退出重开回归通过；产品注册键仍缺失，解决方式是持久补全快捷方式工作目录，不能写成已重建厂商注册信息或已覆盖任意外部启动方式。详细证据与回退见[实录§11](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)，同步任务状态和索引。源码、模型、原始归档及temp1022保持；未构建ROS、运行项目／硬件、上传诊断资料、提交或推送Git。

## 9. Docker普通桌面注册／配置修复与Computer Use实测（19时至20:07）

**目标与新增约定：** SHELL再次反馈同一注册键报错，要求每次修复后使用Computer Use在桌面直接点击打开并测试。新[截图原件](附件/2026-10-21/codex-clipboard-d2a76aaa-dde5-46a7-9572-d05a7d2b0c97.png)保持原名，复制SHA256一致。助手在资源管理器桌面目录实际双击同一Docker图标，确实复现，推翻§8的修复结论。

**直接原因与证据：** 本机启动器优先从Windows卸载项读取InstallLocation；读取失败后才回退到报错中的旧产品键。独立的同账户、普通权限、交互登录任务检查发现：助手工具环境中的HKCU卸载项与原install-settings存在，但普通桌面用户环境缺失；普通用户设置亦缺少已处理的首次引导状态。核验前比对SID一致；没有把尚未证明的隔离机制或某项厂商缺陷写成事实。此前COM／Shell另启进程不足以独立排除此差异。

**实际修复：** 先保存原状，19:52:04恢复安装器已有的11个当前用户卸载注册值，按原名、类型及值写入正常桌面环境；未猜造旧HKLM产品键。实际点击后缺键报错消失，但首次引导提示配置仍不一致。正常停止Docker后，备份普通用户配置，19:59:00–19:59:01恢复原WSL2／D盘install-settings及原同版本7个设置字段，保留其他字段。沿用SHELL此前已处理的同版本引导状态；本轮未代点新条款或更改认证。

**实际验收：** 修复后实际双击桌面目录同一图标进入主界面；随后两轮均通过Computer Use点击Quit Docker Desktop，确认前后台进程全部结束，再实际双击打开。两次backend新启动时间分别20:03:03、20:06:22；两次界面Engine running且无缺键弹窗。20:05:16复用已有hello-world（--pull=never、linux/amd64）退出0，digest与原记录一致；引擎29.9.2、4CPU、约5.79GiB可见内存。GUI磁盘位置与独立普通用户Lxss注册位置均回到D:\uav-dev\docker-data，D盘数据保留。期间生成的C盘默认数据VHDX约1.55GiB保留，未擅自清理。

**记录及边界：** 详见[实录§12](../docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md)、[精简验收JSON](../docs/验证记录/DockerWSL_2026-10-21/启动修复验收_2026-10-22.json)。项目外备份／脚本／独立核验在D:\uav-dev\logs\docker-native-registration-fix-20261022；一次性任务完成后均移除，无常驻修复脚本。本轮结论限当前Windows登录会话内实际图标和两轮Quit重开，未重启Windows；未重置Docker、手动移动VHDX、注销发行版、运行ROS／仿真或连接硬件。同步任务状态、总索引及AGENTS.md中的点击验收规则；源码、模型、原始归档、temp1022及其他已有改动保持，未提交或推送Git。

## 10. 日志按日期拆分与引用同步（2026-10-22）

**要求：** SHELL要求把2026-10-21工程日志中编号13及以后的内容移入本日志并重新编号。经SHELL确认，原§18、§19、§21不作删除，一并迁入并按原顺序连续编号。

**操作：** 原§13–§21整段迁入本文件，重编为§1–§9；正文段落、时间戳、哈希／版本号和证据链接按原样保留，仅去掉标题中已无意义的“按SHELL要求归本日”字样，并把节内互引改为新编号（原§13→§1、§18→§6、§20→§8、§21→§9）。“原先§13的381条目”随之改为“原先§1的381条目”。2026-10-21日志保留§1–§12，末尾新增§13指向本日志。截图仍存放在 `附件/2026-10-21/`，文件名与字节不变。

**引用同步：** 总索引 `logs/低空消防无人机工程日志.md` 的Docker、依赖、版本控制条目改指本日志§2–§9并新增10月22日索引行；`docs/项目资料/任务状态.md` 的对应条目改为§2–§5、§6、§7、§9；`docs/项目资料/项目隔离与开发规范.md` 改指§2；`docs/项目资料/当前电脑_Docker与WSL2配置实录_2026-10-21.md` 的返回链接改指本日志；`AGENTS.md` 的日期入口条款按新归属更新。

**核验：** 以已提交版本比对，迁移后§1–§2与提交中§13–§14正文逐字符一致；本日志与10-21日志、总索引、任务状态、开发规范、AGENTS.md 的本地Markdown链接全部可解析（0处缺失），两文件章节编号连续无重号。

**边界：** 本轮只调整日志与文档引用；未修改源码、模型、模型配置或原始归档字节，未编译、未运行、未连接设备，也未提交或推送Git。

## 11. 学长原文、三个RK镜像与Docker仿真答疑（2026-10-22）

**目标与分支：** SHELL要求详细了解项目后回答[学长原文](../temp1022/学长原文.md)中的疑问，并补充要求新的工作结果也记录日志，主分支记录在2026-10-22日志中。本轮核对当前分支为`main`；保留工作树已有改动，不打包提交或改写既有基线。

**项目复核：** 读取AGENTS、总索引、任务状态、当天日志、总技术指南与部署／仿真分析；回到`navigation_2d`、`location`、`ssd_detection`、`gc_2026_sim`相关构建声明、launch和脚本核对。项目已有七包、EGO任务链及仿真材料，当前真机视觉入口仍为Jetson后端，不把Ubuntu固件或电脑仿真理解为RK视觉迁移完成。

**只读操作与证据：** 读取两个ZIP的文件清单，未运行其中exe；新增[核查脚本](../tools/inspect_rk_firmware.py)，以Python只读方式解析三个img的RKFW/RKAF封装及ext家族文件系统，计算完整输入SHA256、包内boot／rootfs哈希和选定系统文件哈希。初次读取boot时发现其使用传统块映射，脚本增加该格式读取后，三镜像核查执行退出0；未挂载、写入或全盘解包。输出[核查JSON](../docs/验证记录/RK固件核查_2026-10-22/镜像只读核查.json)，真实UTC时间戳保留于文件。只读取选定公开系统元数据，未读取账号凭据或Wi-Fi密码。

**实际结果：**

- 旧`boot_linux.img`64MiB，含内核Image、extlinux配置和Toybrick设备树；旧`rootfs.img`6GiB，为Ubuntu18.04／ARM64。
- 新`TB-RK3399ProD-Ubuntu-18.04-update.img`约1.28GiB，外层RKFW／内层RKAF，清单含Loader、GPT参数、trust、uboot、boot_linux、rootfs。统一包替代分项刷写组合，不是仅替代原两个文件中的某一个，也不是Docker镜像。
- 新包内boot与旧boot SHA256一致；内嵌rootfs为1,297,825,792字节（约1.21GiB），文件系统容量同值。两版rootfs中所查os／lsb／debian版本、dpkg状态、bash及Python3文件哈希一致；没有对整个根文件树做一致性认证。
- 两版rootfs的`/opt`为空，`/opt/ros`不存在，dpkg无`ros-*`包；未发现常规预装ROS，不能把准备好的Ubuntu写成ROS已部署。镜像内有`netplan.io`、`openssh-server`的安装记录及`/etc/netplan/01-toybrick.yaml`；未核验开机服务。
- 标准Ubuntu18.04 apt路线配Melodic，原Noetic MAVROS命令需按实际环境纠正。说明wget下载、chmod授权执行、sudo执行数据脚本及GeographicLib／geoid用途，给出有前提的Melodic参考，不执行安装。
- 当前电脑Docker／WSL2可另建amd64 ROS容器；RK ARM64固件不能直接启动为该容器。已有Gazebo材料仍需外部PX4、插件、图形链、TF和Python依赖；`start_px4_slam.sh`默认Noetic，与Melodic构建方案需显式对齐。保留两种候选的差异，未代替SHELL作ROS平台迁移决定。

**交付：** 新增[完整答疑](../docs/项目资料/学长固件说明与Docker仿真答疑_2026-10-22.md)，同步任务状态、总索引与总技术指南最新提示。核对ROS REP-3、Docker、MAVROS、Netplan、Toybrick和Gazebo官方资料，引用保存在答疑中。

**交付核验（实际2026-10-22 21:44，Asia/Shanghai）：** 三个输入用PowerShell `Get-FileHash`独立复算，SHA256与Python证据一致；五份本轮涉及的Markdown文档相对路径链接全部可解析，日志§1–§12连续；核查脚本语法解析通过、三镜像实际运行退出0，输入字节数、包内boot一致性及所查rootfs文件哈希一致性检查通过。`git diff --check`无错误，`src/`、`.gitignore`、`.gitattributes`无本轮差异；未暂存、提交或推送，既有未提交改动保留。

**范围与下一步：** 本轮未安装驱动、刷机、改Windows／Docker配置、创建ROS容器、构建项目、启动仿真或连接硬件；`src/`、模型、原始镜像、学长原文及Nano历史归档未改写。包可解析与哈希相符不表示来源认证、文件系统全面检查或实机启动兼容性已通过。后续先明确容器ROS版本并完成分目标构建，再推进Gazebo/PX4；板到货后单独核验固件、启动、磁盘／网络、ROS和RKNN。

## 12. RK3399ProD学校报销完成反馈（2026-10-22）

**[已确认／SHELL反馈]** SHELL补充“rk3399proD已得到学校报销，感谢伟大的志龙老师”。据此将当前状态更新为**RK3399ProD已获学校报销**，记录对志龙老师的感谢。

此前10月21日“自费采购、希望争取报销”保留为当时事实，以本次反馈更新其后续结果；未提供金额、票据、报销流程细节或其他配件的报销范围，不作延伸。此反馈不等于板已到货；SHELL本次原文仍说明待到货，未刷机或验收。

## 13. SHELL澄清学长要求联网后手动安装ROS与MAVROS（2026-10-22）

**[已确认／SHELL反馈]** SHELL补充：“学长的原话确实是配置好系统后要联网后手动装ros 然后装mavros。”据此明确交付与接续顺序：配置／刷写并启动Ubuntu→联网→手动安装匹配版本ROS→安装MAVROS及其地理数据→处理项目依赖和构建。

**记录修正：** §11的镜像核查事实保留；未发现常规预装ROS与学长说明一致，属于预期状态，不作为漏装、交付矛盾或学长曾承诺预装的证据。更新[答疑第1、5节](../docs/项目资料/学长固件说明与Docker仿真答疑_2026-10-22.md)、任务状态及总索引。Ubuntu18.04标准Melodic路线与原Noetic包名的版本配套问题仍需按实际环境处理。

**边界：** 本轮只补充澄清与文档，未改写学长原文、镜像、核查JSON或源码，未安装、构建或运行；当前仍在`main`工作树记录，未暂存、提交或推送。
