# RK NPU设备枚举为空排查

[当天日志](../../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md)｜[SSH原始核验](SSH核验.txt)

**后续详细排查，2026-10-25 01:58–02:05（工程归10月24日）：已发现更早的明确阻塞——NPU上电初始化没有落实。** 对照ProD历史原理图及Rockchip署名控制源码，六路电源使能GPIO均为输入／低电平，CPU_RST_NPU也为输入／低电平；NPU参考时钟的实际硬件门控关闭，不只是软件计数为0。板内USB集线器正常枚举、各端口有供电但无下游连接。详细证据和版本边界见本文后半部分。本轮只读，不改变控制脚、时钟或固件；这不是硬件损坏判定，也未完成修复验收。

实际核验时间：2026-10-25凌晨，Asia/Shanghai；按08:00前规则归入2026-10-24。板端时钟明显落后，原始输出保留，不用作本轮工程日期。

## 已确认结果

SHELL反馈在当前RK板下载官方RK3399Pro_npu仓库后，npu_transfer_proxy devices没有设备项。本轮经既有rk-uav别名SSH连接实测：

- 系统为Ubuntu18.04、AArch64、4.4.194；设备树compatible为rockchip,rk3399pro-toybrick-prod-linux及rockchip,rk3399pro。
- 仓库为/home/jcfh/RK3399Pro_npu，origin为https://github.com/airockchip/RK3399Pro_npu.git，commit为8858114555f1700e41e185b14ec3d266342f8059，提交说明为更新NPU驱动至1.7.5。
- 仓库内linux-aarch64/npu_transfer_proxy架构正确、能够执行。普通账户和sudo均只输出List of ntb devices attached，无设备行。
- 原系统无运行中的代理，PATH也未安装代理。以sudo限时运行仓库代理时，打印Transfer version 2.1.0；在代理运行期间再次查询，设备列表仍为空。检查后代理已退出，无常驻测试进程。
- USB仅列出根集线器、Genesys集线器和Realtek无线设备，没有可见NPU设备；/sys/bus/pci/devices为空。
- 已检查的系统可执行目录、启动服务目录、固件目录及/opt等路径未找到NPU配套组件；npu_powerctrl、npu_upgrade、npu-image.sh、系统级npu_transfer_proxy均不在PATH。仓库中的文件不等于已安装的启动链。
- 内核启动存在PCIe link training gen1 timeout及probe failed with error -110。

## 判断与边界

**[已实测] 当前阻塞位于NPU设备发现／初始化阶段。** sudo及单独启动代理均未恢复枚举，不是单纯普通账户权限或漏启动代理即可解释的问题。尚未运行YOLO、模型转换或RKNN推理，不能归因模型文件。

**[推断／优先排查] 当前Ubuntu系统缺少与ProD配套的NPU上电、复位及固件装载启动组件，或该启动链尚未接通。** 现有证据支持优先核对这条链路；尚未直接测量NPU电源、参考时钟或复位，不能将缺组件推断扩大为已确认NPU断电，更不能认定硬件损坏。

PCIe报错是实际证据，但官方说明RK3399Pro板卡可以采用USB或PCIe连接NPU。本轮尚未确认当前实板NPU的实际通道，因此**不把PCIe超时直接认定为NPU根因**，也不根据空列表选择USB／PCIe固件。

代理启动日志里的devid=0123456789ABCDEF不能替代devices返回的设备行；本轮启动后devices仍空，不算发现NPU。主CPU侧没有/dev/galcore也不单独作为NPU驱动缺失结论，RK3399Pro的NPU侧驱动封装在NPU固件中。

## 下一步

1. 取得与TB-RK3399ProD及实际PCB配套的厂商NPU初始化组件／服务，核对上电、复位、参考时钟和固件加载流程；电脑获取后SSH传板，不要求RK联网。
2. 根据本板图纸及厂商组件确认实际NPU通信通道；恢复启动链后先验USB／PCIe实际枚举，再验npu_transfer_proxy devices。
3. 设备可见后，再核对NPU固件、librknn_api和模型转换工具的配套版本，执行独立YOLO推理验证。

本轮仅查询系统与限时代理验证，未修改GPIO、内核、设备树、启动服务、NPU固件或板端项目源码，未重启／刷机。未知实际通道前没有覆盖任何boot.img。

## 官方依据

- [RK3399Pro_npu说明](https://github.com/airockchip/RK3399Pro_npu/blob/main/README.md)：区分USB／PCIe固件、解释devices及代理服务，指出驱动位于NPU固件中；该仓库并不自动完成本板的上电初始化。
- [Toybrick ProD Linux设备树](https://github.com/rockchip-toybrick/kernel/blob/stable/arch/arm64/boot/dts/rockchip/rk3399pro-toybrick-prod-linux.dts)：compatible与本板运行值对应；名称对应不等于全部NPU组件已配置。
- [Radxa RK3399Pro NPU启动过程](https://wiki.radxa.com/RockpiN10/dev/NPU-booting)及[系统说明](https://wiki.radxa.com/RockpiN10/Ubuntu)：作为同芯片上电／装载过程的旁证，不能将另一板型的GPIO命令直接用于ProD。

## 详细排查：上电控制与时钟的寄存器证据

### SHELL反馈与核查范围

SHELL确认本轮前重启过一次，不清楚GPIO／上电脚本含义；刷Ubuntu前没有实际测试原厂NPU。因此不将已有GPIO导出归因为SHELL手动操作，也没有原厂NPU正常的验收基准。

GPIO是处理器用于控制电源开关、复位及通信握手的引脚；上电脚本按板卡时序操作它们。RK3399Pro的主CPU能运行Ubuntu，不代表同封装的NPU已完成上电和固件启动。

采用既有[Rockchip署名ProD原理图](../RK板资料核查_2026-10-21/TB-RK3399ProD_原理图_20181217.pdf)，视觉核对PDF第4、7、10、17页，分别为框图、控制／时钟、NPU USB、NPU电源。**图纸版本为V1.0／2018-12-17，现有实物照片为V1.2；本轮不把历史图纸当作V1.2逐网验收，因此没有依图试写控制脚。** Rockchip署名的两份控制源码与历史图纸的六路使能及复位映射一致；官方Toybrick同版内核源码用来核对寄存器偏移和时钟门控含义。

### 1. NPU通道与集线器

[框图](原理图_P04_框图.png)及[NPU接口页](原理图_P10_NPU接口.png)显示，此ProD历史设计的NPU USB2／USB3信号接到板内GL3520集线器；CPU PCIe则引至扩展插槽，NPU PCIe专用剩余线路未接。**按此设计应先走USB启动路线，不再把CPU PCIe训练超时作为NPU枚举失败的直接解释。** 实物V1.2的逐网确认仍保留。

[USB集线器实测](USB集线器与启动组件.txt)：xHCI与USB2／USB3的05e3:0610、05e3:0616集线器均已绑定。USB2四个端口均仅power，无connection；USB3均为power、Rx.Detect，无连接。证明主CPU能够管理板内集线器，当前没有NPU侧USB设备接入的可见状态；不能扩大为USB链路所有电气指标合格。

### 2. 电源使能、复位尚未驱动

先用运行设备树验证GPIO0／GPIO1地址为0xff720000／0xff730000，再以/dev/mem的**O_RDONLY、PROT_READ**读取DR、DDR、EXT_PORT；未导出或修改GPIO。偏移0x00、0x04、0x50来自同版pinctrl-rockchip.c。

| 历史ProD信号 | 对应Linux GPIO号 | 当前方向 | 读回电平 | 历史图纸作用 |
|---|---:|---|---:|---|
| NPU_PWREN_1 | 4 | 输入 | 0 | NPU 0.8V使能 |
| NPU_PWREN_2 | 10 | 输入 | 0 | NPU逻辑电源使能 |
| NPU_PWREN_3 | 11 | 输入 | 0 | NPU 1.8V及DDR电源使能 |
| NPU_PWREN_4 | 54 | 输入 | 0 | NPU CPU电源使能 |
| NPU_PWREN_5 | 55 | 输入 | 0 | NPU 3.3V IO电源使能 |
| NPU_PWREN_6 | 56 | 输入 | 0 | NPU核心电源使能 |
| CPU_RST_NPU | 32 | 输入 | 0 | NPU低有效复位，未见释放为高 |

pinmux状态均为MUX／GPIO UNCLAIMED；这些控制脚没有成为由软件驱动的输出。现有gpio5／6／41／42与上述六路使能不相同；按官方40pin表，它们对应外部GPIO，不应当作NPU上电成功证据。本轮未改它们。

**这是CPU侧方向和逻辑电平的实测，不是万用表测出的实际电源轨电压。** 结合使能电路可判断与正常NPU上电状态不符；未直接测量六路电压、复位波形或时序，不能仅据寄存器判定硬件健康。

### 3. 24MHz参考时钟确实被门控

软件接口clk_wifi_pmu的enable_count=0、prepare_count=0、rate=24000000。单看rate只能证明配置值，单看计数也不足以判断实际门控，因此进一步只读读取PMUCRU：

- PMUCRU_CLKSEL_CON1（0xff750084）=0x2086，bit13=1选择xin24m，bit14=0选择clk_wifi_div；说明频率源配置为24MHz。
- PMUCRU_CLKGATE_CON0（0xff750100）=0x0ffc，bit8=1。同版clk-rk3399.c将clk_wifi_div映射至此位，GFLAGS含CLK_GATE_SET_TO_DISABLE，因此**该路时钟关闭**。
- GPIO0_A2已被运行设备树复用为npu-ref-clk；历史图纸将REFCLK_OUT接至NPU输入并采用外部24MHz模式。这是已有引脚复用但缺少时钟使能的组合。

证据：[寄存器输出](GPIO与时钟只读寄存器核验.txt)、[复用与接口输出](引脚复用与时钟接口核验.txt)、[本轮实际执行的只读脚本](只读寄存器核查.py)。脚本经标准输入在RK运行，无板端落盘文件；返回0，未写寄存器。

### 4. 缺少主机初始化与固件装载组件

前序及本轮系统搜索未找到安装的npu_powerctrl、npu_upgrade、NPU固件目录或相关启动服务；本轮补查/etc与/usr/lib/systemd中对应命令／npu_fw引用也未发现。仓库仅提供代理、固件及API，克隆不会执行ProD板级上电时序；代理无法自行替代电源、复位、时钟和固件加载。

**当前最具体的软件原因：这套Ubuntu系统没有完成ProD NPU的板级上电初始化。** 六路使能未驱动、复位未释放、参考时钟关闭，已经先于模型／运行库版本阻塞设备发现；即使补起代理，也没有可供其连接的运行设备。固件装载组件还缺失，属于修复上电后仍需继续补齐的下一层。

### 5. 恢复顺序与未执行边界

1. 核对实物V1.2配套控制定义和电源时序，采用匹配ProD的初始化组件；保留当前Ubuntu、ROS和外设已验收配置，先做独立NPU修复。
2. 按板级时序建立六路电源、参考时钟并释放复位，先观察板内USB出现Rockchip启动设备。此阶段不以devices已有运行设备为唯一要求，因为NPU可能尚在等待固件的启动阶段。
3. 再用配套加载工具和**USB接口NPU固件**启动NPU，确认最终USB枚举、代理运行和devices的USB_DEVICE设备行；固件路径与工具必须核对，不将NPU boot.img写入主CPU的boot_linux分区。
4. 完成后核对API／NPU固件版本，再做独立图片推理和重启自启验收。当前未执行步骤1–4，不宣称NPU已恢复；也没有重刷系统、改变内核／DTB或安装galcore模块。

上电控制源码中的顺序为0.8V→逻辑→1.8V／DDR→24MHz→NPU CPU→3.3V IO→核心→延时后释放复位，**仅作本轮源码审阅结果，不是对未知PCB可直接复制的接线或写GPIO命令**。现有内核已经提供参考时钟复用及可写debugfs时钟控制接口，暂不以重编整个内核作为第一步。

### 参考源码与可复现性

- [Rockchip署名Linux电源控制实现](https://github.com/friendlyarm/linuxsdk-rockchip-buildroot/blob/master/package/rockchip/npu_powerctrl/npu_powerctrl.c)，保存在[参考源码](参考源码/npu_powerctrl_friendlyarm.c)，保留SPDX／版权。主机名含wifi的clk_wifi_pmu在本设计承担NPU参考时钟，不能仅凭名称误判为无线网卡设置。
- [TinkerBoard保存的Rockchip同源实现](https://github.com/TinkerBoard-Android/rockchip-android-hardware-rk29-npu_powerctrl/blob/bd3a5355f34af2e83b41fbd507dda2c8cf9b05a8/npu_powerctrl.c)，保存为[npu_powerctrl_tinkerboard.c](参考源码/npu_powerctrl_tinkerboard.c)。用于交叉验证定义，不将Android工具直接安装进Ubuntu。
- Toybrick内核固定commit为a80be5749ac552821967eff313df53f9e0cd1e01，来自本机已核对的原压缩包；本轮仅提取[时钟定义](参考源码/clk-rk3399.c)、[时钟宏](参考源码/rockchip-clk.h)、[GPIO驱动](参考源码/pinctrl-rockchip.c)、[时钟debugfs实现](参考源码/clk.c)及[板级复用](参考源码/rk3399pro-toybrick.dtsi)供只读解释，未编译或加载模块。
- 所有保存文件的本地校验值见[SHA256清单](详细排查文件SHA256.json)。工具调用的初次fitz依赖缺失已改用现有pypdf及Poppler；一次标准输入脚本末行CRLF导致head参数报错，已统一去CR重新核验，无板端配置写入。官方Toybrick历史独立powerctrl仓库URL当前返回404，没有将其记为已取得的可安装ProD软件包。
