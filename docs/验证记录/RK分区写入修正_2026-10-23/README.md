# 升级后仍为Debian：系统分区被跳过及FULL包修正

> **后续修正：本页FULL包仍有boot_linux／rootfs容量为零的缺陷，已由[FULL-v2容量修正版](../RK分区容量修正_2026-10-23/README.md)替代。** 新版已在RKDevTool实际载入且没有容量弹窗；本页保留地址修正历史，不再推荐本页旧FULL文件。

[当天日志§7](../../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md)｜[快速清单](../../项目资料/RK板到货后接续操作清单_2026-10-23.md)

## 结论

SHELL报告使用上一版RK330C包升级后仍是Debian10。[最新工具日志副本](Log2026-10-23.txt)记录18:31:56通过芯片校验并识别eMMC，但下载总量仅8,422,912字节，下载条目只有trust和uboot；18:31:57即显示成功，没有boot_linux或rootfs。

逐项读取RKAF表确认：原包及上一版只改芯片ID的包中，boot_linux与rootfs的`flash_offset`均为`0xFFFFFFFF`。结合实际下载日志，可确定这两项在本次升级被跳过，不能以“下载固件成功”认定Ubuntu已写入。上一轮只修正芯片字段、保留整个RKAF负载时也保留了这一缺陷，前次核查范围不足。

## 修正依据与动作

包内parameter已声明：

```text
0x00002000@0x00002000(uboot)
0x00002000@0x00004000(trust)
0x00030000@0x00006000(boot_linux:bootable)
-@0x00036000(rootfs:grow)
```

因此boot_linux应写LBA `0x6000`，rootfs应写LBA `0x36000`，单位512字节扇区；boot实际64MiB小于分区96MiB。包内extlinux默认启动项、rootfs的fstab与parameter所设rootfs PARTUUID一致，均为`614e0000-0000-4b53-8000-1d28000054a9`。rootfs的os-release及lsb-release标为Ubuntu18.04；debian_version为buster/sid不单独作为发行版判断依据。

先用配套AFPTool2.2解包确认原包及原内容。尝试其pack功能3次均以`-1073740791`退出，未得到可用负载；失败的零字节临时产物保留在已忽略的firmware目录，不用于刷机。随后按parameter明确地址，仅修正RKAF头内这两个目的地址字段，重算RK专用CRC（多项式`0x04c10db7`、初值0）；算法先在原完整负载上独立复现原CRC `0xd2e078bc`，新CRC为`0x0240e56d`。CRC格式参照[原作者实现](https://github.com/dayongxie/rk2918_tools/blob/master/rkcrc.h)，未使用普通zlib CRC代替。

再用配套AFPTool解包修正后的RKAF，实际输出`Check file... OK`和`Unpack firmware OK!`、退出0。最后用配套RKImageMaker2.23的`-RK330C`及保持原值的`-os_type:androidos`重新生成外层包，摘要及新镜像生成成功、退出0。未修改任何系统分区文件，也未向设备执行写入。

## 当前应使用的文件

[TB-RK3399ProD-Ubuntu-18.04-RK330C-FULL-update.img](../../../firmware/TB-RK3399ProD-Ubuntu-18.04-RK330C-FULL-update.img)

- 大小：1,374,261,720字节。
- SHA256：`294051f603008f3fe7082cbf87b7015d0bce526bbbef184018b47b2b5f4952ca`。
- 同目录有`.img.sha256`；本体和临时件按既有`/firmware/`规则排除Git。
- 上一版无FULL的包只解决了芯片校验，已被本版替代，不再推荐刷写；所有原件保留。

[独立核查JSON](完整分区封装修正核查.json)确认：外层Loader和每一个分区成员的大小／SHA256均与上一版一致；RKAF头之后至尾CRC前的所有字节一致。变化仅为两个分区目的地址、RKAF尾CRC及外层封装时间／摘要；芯片ID继续为实板日志对应的`0x33333043`。根文件系统仍包含此前发现的扩容服务链接问题，刷后保留一次空间确认。

## 重刷与最低确认

重新进入LOADER，在“升级固件”中选择FULL文件并升级。此次日志必须出现`Start to download boot_linux`和`Start to download rootfs`，再最终成功；只见trust／uboot仍不算Ubuntu写入。进入系统后用`cat /etc/os-release`确认`ID=ubuntu`、`VERSION_ID="18.04"`，再用`df -h /`确认安装空间。若仍不一致，再查看`findmnt /`／`lsblk`与实际启动介质，不提前要求重做整套采集。

当前只完成离线修正和核验；尚无FULL包在实板写入、启动或驱动／NPU运行的结果。未改源码、原镜像、工具配置和此前证据。
