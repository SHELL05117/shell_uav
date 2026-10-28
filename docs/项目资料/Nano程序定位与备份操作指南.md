# Nano 程序定位与备份操作指南

编号日期：2026-10-11。本文保留程序定位与拷贝的历史操作方式。当前程序材料、外部依赖分包与已发现的权限补档均已拷贝校验完成；稳定版本来源仍为分析中的待确认信息。最新需求为完整项目分析、环境缺口、电脑 WSL＋Docker 编译准备和详细技术指南，本轮先只做文档对齐，见 [当前需求](当前需求与分析交付计划_2026-10-11.md)。不再继续整盘备份或在 Nano 独立再编译。

## 当前结果

电脑副本 SHA256 与 Nano 端校验值一致；六个 ROS 包、默认 ONNX/标签/engine、构建资料和主要外部依赖已保存，38 项权限缺失已另补。证据见 [补档核验](Nano补档下载与完整性核验_2026-10-11.md)。既有目录/入口分析见 [归档分析报告](Nano程序归档检查与项目分析_2026-10-11.md)，本轮需求修订见 [2026-10-11 日志末尾](../../logs/2026-10-11_程序导出、补档与项目隔离基线.md)。环境采集不再作为新待办。

下方排障与拷贝步骤保留历史阶段状态，其中“尚未下载/验证”和“下一步”描述当时情况；最新任务范围以本节和当前需求文档为准。

## 已知信息与边界

- SHELL本次确认：电脑没有网线接口；Nano 可以连接显示器、进入 Ubuntu；账户是 `jcfh`。
- [2026-10-01 的连接日志](../../logs/2026-10-01_Nano连接排障与双舵机空载测试.md)记录：Micro-USB 虚拟网卡曾经连接成功，Nano 地址 `192.168.55.1`，电脑 USB 端 `192.168.55.100/24`，SSH 登录成功。它是历史验证结果，下次连接仍需复查。
- 同一日志记录过舵机程序路径 `/home/jcfh/catkin_ws/src/servo_control/script/servo_controller.py`。因此 `/home/jcfh/catkin_ws` 是优先检查的工作空间，但不能据此认定定位、导航和所有依赖都在这里。
- 2026-10-11 本次 Windows 检查：已有 `ssh.exe`、`scp.exe`，WLAN 正在使用；网卡列表没有显示 Nano USB 网络接口。
- 学长截图给出 `location`、`navigation_2d` 两个 ROS 包名，以及 Cartographer、IMU、TensorRT、Offboard、QGC 等线索。包的具体功能、依赖和实际启动行为需要读取程序确认。

### 2026-10-11 新增本地检查结果（SHELL报告）

- SHELL确认账户与路径均为 `jcfh`，先前写成 `jch` 是输入错误。
- `rospack find location` 返回 `/home/jcfh/catkin_ws/src/location`；`rospack find navigation_2d` 返回 `/home/jcfh/catkin_ws/src/navigation_2d`。两个包都位于同一个工作空间，完整源码及稳定版本仍需查看文件验证。
- `du` 报告 `/home/jcfh/catkin_ws` 大小约 `73M`；对该目录所在文件系统的 `df` 查询显示 `/dev/mmcblk1p1`，容量约 `116G`、已用 `17G`、可用 `93G`。足够先打包项目，SHELL转录遗漏了挂载点，不能据此认定它挂载于 `/`。
- `lsblk` 转录显示约 `14.7G` 的 `mmcblk0`，包含多个分区及 `boot0`/`boot1`/`rpmb`；另有约 `119.4G` 的 `mmcblk1`，其 `p1` 是 ext4。前者的名称和特殊区域提示 eMMC，后者可能是约 128 GB 的 SD 卡，需要 sysfs 的 `type` 输出确认。
- `mmcblk0p1` 据转录挂载于 `/media/jcfh/09f02f1d-fd23-423c-af64-e72e01af7ced`。这不代表它是当前系统根目录，也不证明其内容属于当前稳定版本。
- 下一步：先打包现有工作空间；并用 `findmnt -n -o SOURCE,TARGET -T /`、`findmnt -n -o SOURCE,TARGET -T /home/jcfh/catkin_ws` 及 `/sys/block/mmcblk0/device/type`、`/sys/block/mmcblk1/device/type` 核对存储分工。整机镜像方案需包含实际根文件系统与启动相关存储，不能因 SD 卡容量大而只备份它。

### 同日后续结果：工作空间归档与存储分工（SHELL报告）

- SHELL报告已生成 `/home/jcfh/nano_catkin_ws_20251024_201602.tar.gz`，`ls -lh` 显示大小约 `37M`。归档已生成，不等于已经完成可读性、内容完整性或稳定版本验证。
- SHELL报告 SHA256 为 `5764ea536dcb4d6b4d18d564e7ea430b21c2f3f015e0d3198628d17c0d0b3eb8`。本机检查确认转录值是 64 位十六进制字符串，尚未收到归档文件，未验证其实际哈希。
- `findmnt` 对 `/` 和 `/home/jcfh/catkin_ws` 均返回 `/dev/mmcblk1p1 /`：当前系统根文件系统和该工作空间位于同一分区。
- `mmcblk0` 的 sysfs `type` 返回 `MMC`，结合前述 boot0/boot1/rpmb 区域，判断为 eMMC；`mmcblk1` 返回 `SD`，确认 SD 卡承载当前根文件系统。eMMC 是否参与实际启动链路仍需进一步核实，不能仅因根目录位于 SD 卡就认为整卡镜像包含全部启动环境。
- 文件名日期 `20251024` 与电脑当前日期 `2026-10-11` 不一致；需要用 Nano 的 `date -Iseconds` 核对，可能是设备时钟或转录问题。先按真实文件名导出，不自动调整 Nano 时钟。
- 助手已在电脑创建接收目录 `D:\SHELL\低空消防\ChatGPTworkingenviroment\Nano导出\2026-10-11`；随后 Windows 网卡复查仍未显示 Nano USB 网卡。尚未进行 SSH 下载。
- 下一步：通过已验证的 USB SSH 链路或 U 盘把这份归档带到电脑，检查 SHA256 和归档可读性，之后查看源码并补齐工作空间外部文件及依赖环境资料。

### 同日 USB 排障结果（助手在电脑实测）

- SHELL接上 USB 后，Windows 出现 `E:`，标签 `L4T-README`，容量约 16 MB；实际目录只有驱动及 README 等说明。读取本次 `E:\README-usb-dev-mode.txt` 确认它是只读说明盘，不是 Nano 的系统盘或项目导出盘。
- Nano 的 USB 复合设备已识别，包括 RNDIS、NCM、串口 COM5 和说明盘。RNDIS 当前是“以太网 3”、接口索引 `11`，地址为 `169.254.1.199/16`；NCM 是“以太网 2”、接口索引 `4`，地址为 `169.254.207.46/16`。这些是当前枚举值，之后重插可能变化。
- 两个 USB 接口都没有得到预期的 `192.168.55.100/24`，因此本次 IPv4 访问尚未连通。电脑执行环境不是管理员，没有擅自提升权限或修改 WLAN。
- 依据本次说明盘提供的 IPv6 方法，实测 `fe80::1%11` 的 TCP 22 端口连通；`fe80::1%4` 在 3 秒内未连通。
- 使用 `HostKeyAlias=192.168.55.1` 和 `StrictHostKeyChecking=yes`，SSH 将当前设备公钥与电脑既有记录核对通过。`BatchMode=yes` 测试止于 `Permission denied (publickey,password)`：无交互认证未成功，未尝试SHELL密码，未读取 Nano 项目文件。
- 当前可在电脑 PowerShell 用下方 IPv6 SCP 方法下载，密码由SHELL在本机终端输入。只有完成下载、哈希对比和归档检查后才可确认电脑副本可用。

```powershell
Set-Location -LiteralPath 'D:\SHELL\低空消防\ChatGPTworkingenviroment\Nano导出\2026-10-11'
scp -o ConnectTimeout=10 -o StrictHostKeyChecking=yes -o HostKeyAlias=192.168.55.1 'jcfh@[fe80::1%11]:/home/jcfh/nano_catkin_ws_20251024_201602.tar.gz' .
```

IPv6 方括号是 SCP 地址语法的一部分，`%11` 是当前 RNDIS 接口索引。若拔插后索引改变，先重新查网卡和端口，再替换此数字。

### 同日再次核查：FinalShell 的 IPv4 连接

SHELL反馈 FinalShell 用 `192.168.55.1` 仍连不上。电脑复测显示：RNDIS“以太网 3”仍为接口索引 `11`，IPv4 是 `169.254.1.199/16`；`fe80::1%11:22` 连通，`192.168.55.1:22` 超时。严格公钥核验的 IPv6 SSH 请求到达认证阶段，因未提供密码返回 `Permission denied (publickey,password)`。本轮没有用SHELL密码登录，也没有修改网卡。

此前成功下载使用的是 IPv6，命令里的 `HostKeyAlias=192.168.55.1` 只用于查找既有公钥记录，不是连接地址。USB 提供本地网络，SSH/SCP 在这条网络上传输。

**直接在 Windows PowerShell 使用已验证的 IPv6 地址登录：**

```powershell
ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=yes -o HostKeyAlias=192.168.55.1 'jcfh@fe80::1%11'
```

密码在本机输入。`%11` 选择当前 RNDIS 网卡；SSH 命令的地址不需要 SCP 那样的方括号。

**要在 FinalShell 使用 IPv4：**

1. Windows 按 `Win+R`，输入 `ncpa.cpl`。
2. 找到当前 RNDIS 接口“以太网 3”，右键“属性”，双击“Internet 协议版本 4 (TCP/IPv4)”。这一步配置 USB 网卡，电脑上网仍由 WLAN 提供。
3. 设置电脑 USB 端 IP `192.168.55.100`，子网掩码 `255.255.255.0`，默认网关留空，DNS 不填；应用设置。Windows 可能要求管理员权限，助手当前执行终端无管理员权限。
4. PowerShell 执行 `Test-NetConnection 192.168.55.1 -Port 22`。`TcpTestSucceeded=True` 后，在 FinalShell 的 SSH 连接中填写主机 `192.168.55.1`、端口 `22`、用户名 `jcfh`、原登录密码。

这是本设备说明盘 NVIDIA README 给出的静态 IPv4 配置。FinalShell 是否接受带作用域的 IPv6 主机名未得到官方文档确认，因此不把它作为已经验证的连接方式。若 IPv4 配好后端口仍失败，进一步读取 Nano 的实际 USB IPv4/桥接状态，不用更改 ROS 程序。

### 完整备份的验收范围

| 范围 | 当前状态 | 后续验收 |
|---|---|---|
| 当前 `catkin_ws` | 电脑已取得归档，SHA256 一致，gzip/tar 可读；79 个链接目标均已归档 | 保留原始包；稳定版本来源、Linux 恢复及运行验证待做 |
| 工作空间外的程序、模型、地图和配置 | 默认启动引用已静态分析；外部 ROS/NVIDIA 依赖与系统配置未备份，默认模型在包内 | 下次采集实际 ROS 环境、外部目录与启动配置，再补文件 |
| 已安装依赖和环境配置 | 尚未完整备份 | 保存环境清单与关键配置，并纳入系统存储备份 |
| 当前系统 SD 卡 | 根分区位置已确认；尚未制作系统镜像 | 选择一致的离线备份方式，保存完整相关分区并校验 |
| eMMC 及启动相关存储 | 设备已发现；尚未确认其启动职责、尚未备份 | 核对启动路径和适用备份工具，覆盖必要区域及可能存在的旧项目文件 |

不得把工作空间归档、依赖清单或 USB 说明盘称作完整整机备份。若验收目标包括整架无人机的恢复，PX4 飞控自身的固件与参数还需单独备份。

先保留原系统和程序。本轮只检查、备份，不运行 `roslaunch`、舵机脚本或飞行任务，不切换 Offboard，不升级或重新编译现有环境。

## 1. 推荐连接：电脑 Wi-Fi 上网，USB 单独访问 Nano

```text
互联网 ── Wi-Fi ── 电脑 ── Micro-USB 数据线 ── Nano
                    │                          │
                Codex 在线协助              原有独立供电
```

这是两条独立的通信链路。SSH 和文件下载经过 USB 的局域网络，电脑的上网连接继续走 Wi-Fi。Nano 本身不需要访问互联网。

1. 电脑保持连接当前能上网的 Wi-Fi；现场没有可用 Wi-Fi 时，可让电脑连接 iPhone 的 Wi-Fi 个人热点。
2. Nano 使用原先确认正确的独立供电并正常开机。用支持数据传输的 Micro-USB 线连接电脑和载板之前成功使用的 Micro-USB 口。保持正常启动状态，不进入刷机 Recovery 模式。
3. 在**电脑 Windows PowerShell**检查：

   ```powershell
   Get-NetAdapter | Format-Table ifIndex,Name,InterfaceDescription,Status
   Get-NetIPAddress -AddressFamily IPv4 | Format-Table InterfaceAlias,IPAddress,PrefixLength
   ```

   寻找新增的 RNDIS/NCM/USB 网络接口。旧日志里的“以太网 3”和接口索引只是当时的值，不能直接套用。

4. 如果实际 USB 接口已有 `192.168.55.100/24`，直接测试 SSH。若没有 IPv4 或只有 `169.254.x.x`，先确认该接口确实属于 Nano，再配置电脑 USB 端：IP `192.168.55.100`、掩码 `255.255.255.0`、默认网关和 DNS 留空。保留 WLAN 的原有设置。USB 地址仍以 Nano 当前配置为准。

   旧日志验证过的临时设置方法如下，只有确认接口后才在**管理员 PowerShell**使用；把占位文字替换为当前 USB 网卡的实际名称：

   ```powershell
   netsh interface ipv4 set address name="实际USB网卡名称" source=static address=192.168.55.100 mask=255.255.255.0 gateway=none store=active
   ```

   `store=active` 在电脑重启后失效。此配置仅用于确认的 USB 接口，不用于 WLAN。

5. 在**电脑 Windows PowerShell**测试和登录：

   ```powershell
   Test-NetConnection 192.168.55.1 -Port 22 -InformationLevel Detailed
   ssh -o ConnectTimeout=10 jcfh@192.168.55.1
   ```

   端口测试应显示 `TcpTestSucceeded : True`，访问接口应是实际 USB 网卡。密码在本机输入，不发送给助手。若首次连接询问主机身份，核对目标设备；出现主机密钥变化时先确认设备或系统是否发生变化。

连接失败时先区分：USB 网卡未识别、IPv4 未配置、22 端口不可达、账户认证失败。保留完整报错即可，不重复修改密码、系统或 ROS 程序。

## 2. 在 Nano 上定位实际使用的程序

以下命令在**Nano 的 Ubuntu 终端**执行；可以用显示器上的终端，也可以用已经 SSH 登录成功的终端。Ubuntu 桌面通常可用 `Ctrl+Alt+T` 打开终端。

先运行最少的检查：

```bash
whoami
ls -ld /home/jcfh/catkin_ws
printenv ROS_DISTRO ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
rospack find location
rospack find navigation_2d
```

`rospack find` 返回当前 ROS 环境选中的包路径。若存在多个工作空间，它不保证已经证明这个路径就是学长认定的稳定版本；还要与原启动环境、启动文件和版本记录对照。

如果显示 `rospack: command not found` 或找不到包，保存报错。可能是终端没有加载原 ROS 环境，不能据此认定程序不在 Nano 上。先检查环境加载线索：

```bash
ls /opt/ros
grep -nE '(^|[[:space:]])(source|\.)[[:space:]]|ROS_' /home/jcfh/.bashrc /home/jcfh/.profile
find /home/jcfh -maxdepth 6 -type f \( -name location.launch -o -name navigation_2d.launch -o -name setup.bash \) -print
```

这次搜索只覆盖SHELL主目录一定深度；没有搜索结果不能证明磁盘其他位置没有程序。后续根据环境配置、挂载盘和启动文件扩大搜索，不直接从整个 `/` 开始。

需要离线保存结果时，将下面整段粘贴进 Nano 终端。它只检查信息并新建一个带时间戳的文本报告；不会启动 ROS 节点或修改项目文件。`rospack` 自身可能更新其包索引缓存。

```bash
report="$HOME/nano_inventory_$(date +%Y%m%d_%H%M%S).txt"
{
    printf '\n=== Identity and versions ===\n'
    date -Iseconds
    whoami
    hostname
    uname -m
    cat /etc/os-release
    cat /etc/nv_tegra_release 2>/dev/null
    python3 --version
    printf '\n=== ROS environment ===\n'
    ls /opt/ros
    printenv ROS_DISTRO ROS_PACKAGE_PATH CMAKE_PREFIX_PATH ROS_MASTER_URI ROS_IP ROS_HOSTNAME
    command -v rospack
    printf '\n=== Home and candidate workspace ===\n'
    ls -la /home/jcfh
    ls -ld /home/jcfh/catkin_ws
    printf '\n=== Environment loading clues ===\n'
    grep -nE '(^|[[:space:]])(source|\.)[[:space:]]|ROS_' /home/jcfh/.bashrc /home/jcfh/.profile
    printf '\n=== Selected ROS package paths ===\n'
    rospack find location
    rospack find navigation_2d
} > "$report" 2>&1
printf 'Report saved: %s\n' "$report"
```

记录最后显示的报告完整路径。若包定位失败，再把上述 `find` 的输出保存或提供给助手。报告未包含 shell 历史和 SSH 私钥；分享前仍检查是否有自己写在配置中的敏感信息。

## 3. 先打包，再导出到电脑

确认 `/home/jcfh/catkin_ws` 存在后，可以先备份这个已知工作空间。若定位和导航包落在别的工作空间，另外备份那些实际目录。

在**Nano 终端**先检查大小与存储空间：

```bash
du -sh /home/jcfh/catkin_ws
df -h /home/jcfh
```

确认剩余空间足够后打包：

```bash
archive="$HOME/nano_catkin_ws_$(date +%Y%m%d_%H%M%S).tar.gz"
tar -czf "$archive" -C /home/jcfh catkin_ws
```

只有 `tar` 成功退出后，再执行检查并保存校验值：

```bash
tar -tzf "$archive" > /dev/null
sha256sum "$archive" > "$archive.sha256"
printf 'Archive: %s\n' "$archive"
printf 'Checksum: %s\n' "$archive.sha256"
```

保留原始 `.tar.gz` 文件，它保存目录结构、Linux 权限和符号链接。符号链接指向工作空间外部时，默认只保存链接，外部目标必须另行备份。工作空间归档也不等同于整张系统盘的可恢复镜像。

### 路线 A：通过 SSH 下载

在**电脑 Windows PowerShell**创建本地保存位置：

```powershell
New-Item -ItemType Directory -Force -Path 'D:\SHELL\低空消防\ChatGPTworkingenviroment\Nano导出\2026-10-11'
Set-Location -LiteralPath 'D:\SHELL\低空消防\ChatGPTworkingenviroment\Nano导出\2026-10-11'
```

然后下载。以下文件名是示例占位文字，必须替换成 Nano 上最后打印的实际文件名：

```powershell
scp "jcfh@192.168.55.1:/home/jcfh/实际归档文件名.tar.gz" .
scp "jcfh@192.168.55.1:/home/jcfh/实际归档文件名.tar.gz.sha256" .
scp "jcfh@192.168.55.1:/home/jcfh/实际报告文件名.txt" .
Get-FileHash -Algorithm SHA256 -LiteralPath '.\实际归档文件名.tar.gz'
Get-Content -LiteralPath '.\实际归档文件名.tar.gz.sha256'
```

比较两边 SHA256；字母大小写可以不同，字符内容应相同。若改用路由器局域网，把 `192.168.55.1` 换成确认属于 Nano 的实际局域网地址。Nano USB 地址与路由器分配的地址不混用。

SSH 是远程终端连接；`scp`/SFTP 是经 SSH 传输文件。电脑已有相应命令，无需为这一步另装软件。

### 路线 B：使用 U 盘，不依赖 SSH

1. 按上面步骤在 Nano 本机完成报告和打包。
2. 将 U 盘插入 Nano，在 Ubuntu 的“文件”中打开“主目录”（`/home/jcfh`）。
3. 把刚生成的报告、`.tar.gz` 和 `.sha256` 三个文件**复制**到 U 盘，确认复制完成后弹出 U 盘。
4. 把 U 盘插入电脑，复制到本项目的 `../../nanooutput_origin/Nano导出/2026-10-11` 文件夹。用 PowerShell 的 `Get-FileHash` 对照校验值。

选择电脑和 Ubuntu 都能读写的 U 盘格式。FAT32 单文件上限约 4 GiB；归档超过上限时换用已确认两端可读写且支持大文件的介质，或改用 SSH。不要为临时导出而格式化装有其他文件的 U 盘。

文件复制到电脑后，电脑恢复或保持互联网连接，即可继续分析本地文件。Nano 当时是否联网不会妨碍对备份的静态分析。

## 4. 导出时各目录的作用

下面是 ROS 1/catkin 项目常见约定，实际项目需以文件内容为准。

| 内容 | 通常的作用 | 初次备份策略 |
|---|---|---|
| `src/` | 项目包、C++/Python 源码等 | 优先保留；目录名叫 src 不保证里面只有自写代码 |
| `launch/`、`.launch` | 指定启动节点、参数及其他启动文件 | 核心分析入口 |
| `config/`、YAML、标定文件 | 参数、传感器标定、控制或定位配置 | 保留 |
| 地图、模型权重、ONNX、TensorRT engine | 建图/定位或视觉推理的数据 | 保留；可能位于工作空间外 |
| `package.xml`、`CMakeLists.txt` | ROS 依赖与编译配置 | 保留 |
| `build/`、`devel/`、`install/` | 编译产物、可执行文件和环境入口 | 第一次备份先保留，之后再判断哪些可重建 |
| `/opt/ros`、CUDA、系统库 | ROS 和 NVIDIA 的运行环境/依赖 | 记录版本及安装来源；不能只复制目录就认为环境已迁移 |
| `.bashrc`、启动服务、设备权限配置 | 环境选择、自动启动、硬件访问 | 导出有关配置，检查敏感信息 |
| 日志、ROS bag、录像 | 故障证据或实验数据 | 先确认作用及大小，再决定是否额外导出 |

“稳定程序能在 Nano 上运行”不证明完整源码仍保存在板上，也不证明复制到 Windows 就能运行。TensorRT 引擎通常受到操作系统、CPU/GPU 和 TensorRT 版本限制。先完成备份与代码分析，之后再决定电脑上的开发或仿真环境。

## 5. 学长截图的解读与待核实项

| 截图线索 | 当前可用的理解 | 需要从程序确认的内容 |
|---|---|---|
| `roslaunch location location.launch` | 定位/建图的启动入口 | 实际节点、传感器、TF、地图路径、是否启动 MAVROS |
| `roslaunch navigation_2d navigation_2d.launch` | 导航与任务执行的启动入口 | 是否自动发送飞行/舵机命令、任务状态机、模型路径 |
| Cartographer 报错可重启、IMU 未就绪 | 学长记录的一种历史现象 | 实际错误、时间戳、传感器状态与就绪条件，不能仅凭截图归因 |
| TRT 模型加载完成后再 Offboard | 学长要求先完成推理初始化 | 程序中的就绪标志和允许起飞的其他条件 |
| 机头朝地图 `+x` | 初始朝向相关操作要求 | 坐标系定义、地图原点、定位初始化以及转换关系 |
| QGC 切换 Offboard | 使用地面站选择飞行模式 | 控制设定值、失联/失败行为，不能据此认为进入模式就一定起飞 |
| `nvpmodel`、`jetson_clocks` | NVIDIA 电源模式和频率管理 | 板卡型号、原配置及实际性能需求，本轮不更改 |
| 两个 SSH 窗口、不能同时启动 | 可能是在不同窗口按顺序启动，或涉及阶段互斥 | 定位启动完成后是否保持运行，还是必须退出后再启动导航 |

最后一项不能从截图直接确定。分析两个 launch 文件及其引用的脚本后，再补齐启动顺序，暂不照截图试飞。截图中的拼写、名称和简写也以 Nano 实际文件为准。

## 6. 接下来对备份做什么

2026-10-11 后续已完成工作空间下载与环境清单核验。电脑 USB 网卡已由SHELL设为 `192.168.55.100/24`，FinalShell 可登录 Nano `192.168.55.1`。补备份初版已执行但中断，发现重跑时会重新归档旧输出目录；请停用初版，使用完成上传核验的 v2。详细范围、真实版本、进度说明和剩余恢复缺口见 [环境核验与补备份](Nano环境核验与补备份_2026-10-11.md)。新增补档保存于 `Nano补充导出`，避免改变原 `Nano导出` 固定基线。

先确定两个 ROS 包的实际路径与所用工作空间，再沿 launch 文件查依赖节点、参数、地图和模型；建立“定位—导航—视觉—MAVROS/PX4—舵机”之间的关系，并标出哪些属于现有实现、哪些仍未确认。随后整理环境版本和一份可重复执行的启动说明。

分析阶段主要交付：目录用途表、程序调用关系、依赖与外部文件清单、启动顺序中的歧义、稳定版本的证据与尚缺材料。拿到完整文件前，不把旧的目标架构图当作现有程序的实现。

## 参考依据

- [既有项目连接日志](../../logs/2026-10-01_Nano连接排障与双舵机空载测试.md)：本设备历史 USB 接入和程序路径，当前仍需复查。
- [NVIDIA JetBot USB 设备网络说明](https://raw.githubusercontent.com/NVIDIA-AI-IOT/jetbot/master/docs/software_setup/wifi_setup.md)：支持 USB 虚拟以太网的机制说明，不保证任意载板/镜像均支持。
- [Microsoft OpenSSH](https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_overview)：SSH、SFTP、SCP 的用途。
- [ROS 官方命令速查](https://github.com/ros/cheatsheet/blob/master/ROScheatsheet.tex)：`rospack find` 定位包。
- [NVIDIA TensorRT 的兼容性说明](https://docs.nvidia.com/deeplearning/tensorrt/latest/architecture/how-trt-works.html)：引擎对运行环境的依赖。
- [Apple 个人热点说明](https://support.apple.com/zh-cn/111785)：iPhone 可经 Wi-Fi 或 USB 分享蜂窝网络；Windows USB 方式要求 Apple 设备或 iTunes，并按提示信任电脑。若使用“电脑 Wi-Fi 连无人机路由器＋iPhone USB 上网”，还需检查默认路由；本设备尚未验证这条组合。
- [OpenAI Docs 故障排查](https://learn.chatgpt.com/docs/reference/troubleshooting)：在线协助访问本机文件/工具时应保持电脑在线。电脑上网与 Nano 的局域网连接可以分别安排。
