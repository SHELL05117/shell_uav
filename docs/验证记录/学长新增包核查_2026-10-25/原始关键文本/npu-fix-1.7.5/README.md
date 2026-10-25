# RK3399Pro NPU 修复包 1.7.5

适用：已刷好本工程 Ubuntu 镜像、NPU 检测不到的 TB-RK3399ProD。
不需要重刷任何分区。

## 用法

```bash
scp npu-fix-1.7.5.tar.gz toybrick@<板子IP>:~/
ssh toybrick@<板子IP>
tar xzf npu-fix-1.7.5.tar.gz
cd npu-fix-1.7.5
sudo ./install.sh            # 安装并立即加载 NPU
sudo ./install.sh --no-start # 只安装，下次开机自动加载
```

## 装了什么

- `/usr/lib/firmware/npu_fw/`：NPU（RK1808）固件，版本 1.7.5
- `/usr/bin/`：npu_powerctrl、npu_upgrade、npu_transfer_proxy、upgrade_tool
- `/usr/lib/aarch64-linux-gnu/librknn_api.so`
- `rknn.service`：开机把固件经 USB3 推给 NPU，然后常驻 npu_transfer_proxy

RK3399 内核里没有 NPU 驱动，驱动和 rknn_server 都在 NPU 的 boot.img 里，
所以每次上电都要重推一次固件，这部分由 rknn.service 负责。

## 验证

```bash
systemctl status rknn
lsusb -d 2207:
npu_transfer_proxy devices   # 应输出 USB_DEVICE
```

## 回滚

install.sh 会把被覆盖的文件备份到 `/var/backups/npu-fix-<时间>/`：

```bash
sudo systemctl disable --now rknn
# 再把备份里的文件拷回原路径
```

## 内置检查

- SHA256 校验（校验失败不会改动系统）
- `CONFIG_GPIO_SYSFS`：缺失直接报错（只能重编内核）
- `CONFIG_DEBUG_FS`：缺失只警告
