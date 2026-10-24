# RK统一固件芯片标识不匹配及重封装核验

> **后续修正：本页的无FULL副本已由[完整分区写入修正版](../RK分区写入修正_2026-10-23/README.md)替代，不再用于重刷。** 18:31实际升级虽通过芯片校验，但后续查明原RKAF把boot_linux／rootfs地址标为跳过，只写了trust／uboot；本页保留芯片修正历史及核查边界。

[当天日志§6](../../../logs/2026-10-23_RK板到货与原系统接续.md)｜[快速操作清单](../../项目资料/RK板到货后接续操作清单_2026-10-23.md)

## 结论与证据

SHELL在step1使用RKDevTool3.30升级时报“校验芯片失败，固件中芯片标志不正确”。[原始截图](../../../logs/附件/2026-10-23/codex-clipboard-6133e15e-3469-4220-a924-411b4dc04177.png)显示已找到一个LOADER设备并通过设备测试；[工具日志副本](Log2026-10-23.txt)于18:23:42给出明确比较：

```text
ERROR:CheckChip-->Chip is not match,firmware(0x33333939),device(0x33333043)
```

本地原固件RKFW头的offset21字段确为`0x33333939`，包内外层Loader的同类字段却为`0x33333043`，与日志中的实板返回一致。**已确认故障点是外层升级包芯片标识不匹配**；截图里的“芯片信息RK3399”显示名称不能代替这个数值字段。打包时可能误用了`-RK3399`，但未取得学长实际打包命令，此项只作推断。

Rockchip的[RK3399配套打包脚本](https://github.com/rockchip-android/RKTools/blob/rk3399-box-7.1/linux/Linux_Pack_Firmware/rockdev/mkupdate.sh)使用`-RK330C`；[官方RKFW结构定义](https://github.com/rockchip-linux/rkdeveloptool/blob/master/RKImage.h)将该字段列为`emSupportChip`。前次只读核查确认了格式及所查内容，没有比较实板ID；本次补齐了这项兼容判断。

## 本地修正与核验

从原包按其声明边界提取外层Loader和完整RKAF负载，保存在根目录已忽略的`firmware/repack_RK330C_2026-10-23/`。使用SHELL已解压工具中的`bin/RKImageMaker.exe`（自报2.23）重新封装，未更改原文件：

```text
RKImageMaker.exe -RK330C MiniLoaderAll.bin original_payload.rkaf TB-RK3399ProD-Ubuntu-18.04-RK330C-update.img -os_type:androidos
```

`androidos`保留原包的OS封装字段值1，不代表rootfs变为Android；根文件系统负载没有变动。工具实际输出`MD5 data generated successfully!`、`New image generated successfully!`，进程退出0。

修正版：[TB-RK3399ProD-Ubuntu-18.04-RK330C-update.img](../../../firmware/TB-RK3399ProD-Ubuntu-18.04-RK330C-update.img)，1,374,261,720字节；SHA256为`e9a628585200ca104306f4759f68963cf25491aa29a6857f11b002aaa8af5173`。同目录另存`.img.sha256`。固件本体和临时提取件按既有`.gitignore`放在`firmware/`，不纳入Git；本文保留版本、来源和校验值。

独立核查结果见[封装修正核查JSON](封装修正核查.json)：

| 项目 | 结果 |
|---|---|
| 原包SHA256对照10月22日证据 | 一致，原包保持不变 |
| 修正版外层芯片ID | `0x33333043`，与设备日志及Loader一致 |
| 外层Loader，465,230字节 | SHA256与原包一致 |
| 整个RKAF，1,373,796,356字节 | SHA256与原包一致，包含分区清单、参数、引导、boot和rootfs |
| OS封装字段 | 保持1 |
| 外层变化 | 正确芯片ID、新封装时间及由工具重新生成的摘要 |

## 继续操作

保持板卡现有LOADER连接，在“升级固件”页点击“固件”，重新选择上面的RK330C修正版，待载入完成后点击“升级”。本次所见日志没有进入下载／擦除记录；保持当前连接重试即可。以实际日志通过芯片校验并最终升级成功为下一结果，再核验系统启动及安装空间。

这里只完成本地重封装与内容一致性核对，助手没有向设备下发擦除、写入或复位命令；实板升级、启动及驱动／NPU验收尚未取得结果。若后续在下载Boot、分区写入或启动阶段失败，按新的首个错误另行定位，不将本次外层标识修正扩大为系统完整兼容认证。
