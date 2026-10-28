# boot_linux容量为零：FULL-v2修正及真实界面载入核验

[当天日志§8](../../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md)｜[快速清单](../../项目资料/RK板到货后接续操作清单_2026-10-23.md)

## 原因

SHELL提供[弹窗截图](../../../logs/附件/2026-10-23/codex-clipboard-96b41643-25be-4556-a351-80962e7cec50.png)，显示“boot_linux分区不够存放镜像”。[实际日志](Log2026-10-23.txt)于18:49:35／18:50:03两次记录：

```text
ERROR:CheckPartitionSize-->boot_linux part_size=0,0;file_size=67108864,4000000
```

上一版FULL修正了RKAF条目的目的地址，却仍保留原包中boot_linux和rootfs的零容量字段，故工具在载入时拒绝。包内parameter实际给boot_linux分配`0x30000`个512字节扇区，即96MiB，镜像64MiB可放入；rootfs为末尾grow分区。此弹窗不是本板eMMC总容量不足的证据。前次核验漏查了RKAF的part_size，现已补齐。

## 本次修正

从上版修正负载生成新副本，按parameter补齐boot_linux容量`0x30000`、rootfs grow容量标志`0xFFFFFFFF`，目的地址继续为`0x6000`和`0x36000`。重算RKAF CRC为`0xf666d68c`，再用配套RKImageMaker2.23以`-RK330C`及保持原值的OS字段封装，输出成功、退出0。原文件及此前证据均保留。

当前文件：[TB-RK3399ProD-Ubuntu-18.04-RK330C-FULL-v2-update.img](../../../firmware/TB-RK3399ProD-Ubuntu-18.04-RK330C-FULL-v2-update.img)，1,374,261,720字节。

SHA256：`bf91231fd19cc1843e0d8613a5cd9a5222e6d2e418b28d1ab9d13c73610ba90d`，同目录另存校验文件。本体与临时件沿用firmware忽略规则，不进入Git。

[核查JSON](容量与界面核验.json)已逐项确认：芯片ID匹配、所有可写镜像容量足够、所有分区目的地址和容量与parameter一致、外层Loader及每个分区文件SHA256不变。只修改封装元数据，未修改文件系统内容。

## 实际Computer Use验证

使用[Computer Use技能](C:/Users/alexh/.codex/plugins/cache/openai-bundled/computer-use/26.1016.51333/skills/computer-use/SKILL.md)及其sky API，在SHELL已打开的RKDevTool3.30中点击“固件”，通过文件选择框载入FULL-v2。嵌套文件对话框的元素索引操作未准确聚焦，改用截图定位及Alt+N明确文件名焦点后输入完整路径，按Enter打开。

实际载入后返回主界面：固件8.1.00、Loader1.21、芯片RK330C、固件字段为FULL-v2完整路径，底部仍识别一个LOADER设备，没有容量弹窗。验证停在载入状态，助手未点击“升级”、未执行设备擦除／写入／复位，未接管认证或修改系统设置。

下一步SHELL可直接点击当前已选好文件的“升级”，观察boot_linux和rootfs两项下载，再核对Ubuntu启动与根目录空间。载入通过不等于设备写入及运行验收通过。
