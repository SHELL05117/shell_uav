# RK完整刷写及Ubuntu控制台启动记录

[当天日志§9](../../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md)｜[任务状态](../../项目资料/任务状态.md)

本次证据更新前序“仅载入通过”的状态，范围限定固件写入和控制台启动／登录，不代表ROS、网络、NPU或无人机系统已验收。

## 工具日志

[日志原名副本](Log2026-10-23.txt)记录：

- 19:00:18通过设备测试、芯片校验并识别eMMC，下载总量1,373,357,568字节。
- 19:00:19开始下载boot_linux，LBA `0x6000`，67,108,864字节。
- 19:00:23开始下载rootfs，LBA `0x36000`，1,297,825,792字节。
- 19:01:36.892显示“下载固件成功”。

这与前序只下载trust／uboot的8,422,912字节不同，确认本次工具实际执行了两个系统分区下载。助手未点击本次升级，操作来源为SHELL接续；工具日志不单独证明启动或所有驱动可用。

## SHELL提供的现场照片

[原始照片](../../../logs/附件/2026-10-23/codex-clipboard-55b88d39-dcf6-41d8-a172-8a754a241666.jpg)可读信息：Ubuntu18.04 LTS、GNU/Linux4.4.194、aarch64、tty1；登录用户名toybrick，提示符`toybrick@toybrick:~$`。照片确认本次已到Ubuntu文本控制台并登录，不把无图形界面认定为刷机失败，也不推断桌面组件的具体安装状态。

照片复制SHA256：`60628e707d60a44357f644fed79be28e732444dbbae5a9ae481e26eb1b4a2acd`，与SHELL提供文件一致；原文件名保留。

## 账户答疑及接续

当前toybrick是普通登录账户，屏幕root文字是sudo管理员使用说明。root为系统管理员账户名，建议保持；个人登录名称可另建账户并加入sudo组，例如：

```sh
sudo adduser shell
sudo adduser shell sudo
```

由SHELL在板端交互设置密码，其他个人信息可直接回车；之后重新登录新账户。依据[Ubuntu18.04 adduser手册](https://manpages.ubuntu.com/manpages/bionic/man8/adduser.8.html)。

仅改当前toybrick密码可用`passwd`；确需设置／修改root密码可用`sudo passwd root`，不等于改名或自动开启root SSH登录。以上只提供自操作说明，助手未执行账户／密码修改、未接管认证，也未索取或记录密码。

下一步沿既定快速路线临时联网、安装匹配的ROS／MAVROS与依赖，再推进正式DHCP／SSH和通信测试。图形桌面不作为前置；根分区容量／扩容结果、sudo及新账户操作、网络和依赖安装仍待真实反馈。
