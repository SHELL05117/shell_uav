# NPU开机自动启动与重启验收

[任务状态](../../项目资料/任务状态.md)｜[日常操作说明](../../项目资料/RK_NPU上电脚本使用说明_2026-10-24.md#6-开机自动启动)｜[当天日志§16](../../../logs/2026-10-24_外设接入、NPU排查与YOLO验收.md#16-shell要求npu开机自动启动)｜[前轮NPU恢复及36图测试](../RK_NPU实板上电_2026-10-24/README.md)

实际核验2026-10-25凌晨，按08:00规则归2026-10-24；板端日期仍有偏差，保留原始值，PC capture为电脑时间，启动耗时使用journal单调时钟。

**结论：已启用shell-npu.target开机启动，实际主机重启一次验证通过。** 从本次内核启动起约26.4秒，NPU上电、固件RAM启动、USB真实序列号和代理设备就绪全部完成；重启后普通账户可推理交回六类模型，同图三头输出与重启前SHA256逐一一致。自动控制链不启动YOLO／ROS／无人机任务；业务精度结论沿用前轮，未宣称业务验收通过。

## 1. 实际安装

| systemd对象 | 作用与依赖 |
|---|---|
| [shell-npu.target](../../../tools/shell-npu.target) | 唯一开机入口，WantedBy=multi-user.target；Requires并等待代理，已enabled |
| [shell-npu-power.service](../../../tools/shell-npu-power.service) | oneshot、RemainAfterExit；等待本地文件系统、udev trigger、debugfs mount，原上电脚本on；停止时off |
| [shell-npu-firmware.service](../../../tools/shell-npu-firmware.service) | Requires／After电源；锁定工具及四个固件，完成USB NPU RAM启动；等待运行态及真实序列号，再等待内部系统10秒 |
| [shell-npu-proxy.service](../../../tools/shell-npu-proxy.service) | Requires／After固件，启动代理；ExecStartPost有界等待devices并核对内部USB序列号；Restart=no |

三个service均PartOf=target，关闭时按反向依赖停止代理→固件服务→电源。停止target可能先于各PartOf服务收尾返回，应再查看power.service。上电仍由原脚本维护/run/shell-rk-npu-power.json和资源归属，已有健康运行态不重复加载／复位；错误状态拒绝继续。失败不进行无限重试、强制循环断电或重启主机。

[rk_npu_autostart.py](../../../tools/rk_npu_autostart.py)为Python3.6标准库；root副本安装在/usr/local/lib/shell-rk-npu，与rk_npu_power.py和rk_npu_boot.sh放在一起。电源调用显式profile=prod-20181217、USB等待15秒；固件命令190秒、运行USB60秒、代理查询60秒均有界。systemd阶段分别30／270／80秒超时。

启动不依赖网络下载或账户登录；依赖现存/home/jcfh/RK3399Pro_npu的官方固定提交8858114555f1700e41e185b14ec3d266342f8059、四个已核对固件，以及/home/jcfh/npu_runtime_175/upgrade_tool。更换固件／升级工具／系统内核后应重新核查，不能绕过固定哈希保护。

## 2. 验证过程与失败修正

1. 首次SSH已恢复，boot_id=1c302562-7ccf-4bc8-82f4-6bd95bf9f84c、uptime约2分钟，与前轮不同；此前重启来源未知，不归为本轮操作。冷启动预检为空、代理inactive、未运行ROS／NPU推理。[01部署记录](01_部署与静态验证.txt)含当时状态、13项原电源测试和7项新增启动测试；本机及RK Python3.6均20项通过，bash语法及systemd-analyze verify通过。
2. [02启用记录](02_启用开机目标.txt)：创建multi-user.target.wants/shell-npu.target链接并启动依赖链。首次固件阶段被Git dubious ownership拦截，见[03原始失败](03_首次服务启动Git归属拒绝.txt)。systemd root没有sudo调用者身份，不能依赖交互sudo下的仓库归属特例；已将rk_npu_boot.sh的只读Git命令限定为-c safe.directory=这个确切仓库路径，没有添加全局／通配信任，也未放松工具哈希、仓库提交、固件blob或USB唯一目标检查。
3. 修正后[04首次完整启动与推理](04_启动链与六类模型单图验证.txt)：target及三服务均active，devices返回14ade952d732d316／a0096b1e／USB_DEVICE；交回六类模型IMG_2706推理完成，API计时55.3741ms。
4. [05反向停止验证](05_反向停止与资源释放.txt)：代理先停止，电源off返回owned_resources_powered_off_and_released；归属记录清空，GPIO无导出／低电平、24MHz及32k引用均0，冷启动预检无阻塞。05的最初服务快照曾显示deactivating，随后同一证据的journal明确记录power已Stopped；[06](06_主机重启请求.txt)执行重启前再次要求power已inactive。
5. [06主机重启](06_主机重启请求.txt)是本轮实际执行的一次sudo systemctl reboot。之后没有手动on、装载固件或start target。新boot_id=e4f2e041-2c28-4e31-9d65-9b813b909f3d；[07早期快照](07_重启后自动启动状态.txt)显示固件还在等待内部系统，代理未启动，符合顺序；[08最终就绪与模型](08_重启后自动就绪与六类推理.txt)及[10完整本次启动日志](10_重启后完整NPU启动日志.txt)确认：固件命令10.74秒完成，运行USB／真实序列号16.03秒，固件阶段26.07秒完成，代理26.39秒ready。
6. 重启后普通账户运行同一六类模型、同一IMG_2706，三头[1,33,80,80]／40／20，API／DRV1.7.5，退出0，推理56.3862ms。08保存重启前后三个float输出SHA256，逐一完全相同；这是单图恢复一致性验证，未重复36图精度或相机端到端测试。

新增7项测试见[test_rk_npu_autostart.py](../../../tools/test_rk_npu_autostart.py)：未知USB拒绝传输、已有运行态不重载、Maskrom仅调用受保护helper一次、传输失败不进入就绪、占位序列号拒绝、代理错误序列号拒绝、匹配设备成功。系统verify输出仅有原有udevd的内核不支持BPF防火墙提示，退出0；不是NPU服务失败。

## 3. 最终状态与校验

[09最终状态](09_最终状态与部署校验值.txt)：target enabled／active，power及firmware active(exited)、proxy active(running)，所有ExecMainStatus=0／Result=success，普通账户devices有设备。服务路径均为/etc/systemd/system，脚本root所有0644，unit root所有0644；原手动代理unit备份于/home/jcfh/npu_runtime_175/autostart_install/shell-npu-proxy.service.before-autostart。

| 文件 | 本地与部署SHA256 |
|---|---|
| rk_npu_power.py（未改） | d9f151d62a7a049d53bf68f4c28c47858389379a319ae8ff3f1dfd18904bc2ba |
| rk_npu_autostart.py | ad56be93af9158ed1d059b9bf5c3a7358de5c690481fc48729f2c40d345e3a6a |
| rk_npu_boot.sh（本轮Git归属修正后） | 6e543f107c1efec89fb11b7ff3b4699e0bb8f55a0b0b2049c79cd611524afe47 |
| shell-npu-power.service | 2fc9d0a2d6d9fb25b334290a03bb9d0b51fd8386ced241b0743416b2daceeb9a |
| shell-npu-firmware.service | b1ac95f087ab358e5f757350edf43f17aafee67cf2e9c38695d9c03516697211 |
| shell-npu-proxy.service | f632164e4cc8bcdaa86fb1f61877ee72d687e29e093f3ce2d92cc9e435669d0f |
| shell-npu.target | 5f3498370d580ad7caffb5504560e61780f03b7a02e7dcc118543925ea810d43 |

后续正常开机无需手动执行GPIO或固件命令。常用systemctl／journalctl及取消自启方法见[操作说明第6节](../../项目资料/RK_NPU上电脚本使用说明_2026-10-24.md#6-开机自动启动)。首次主机重启恢复通过，不据一次结果宣称反复冷断电／长期负载稳定；未改视觉模型、训练成果、ROS源码或飞控参数，未启动飞行任务。
