# RK板Codex密钥与sudo配置

[返回项目目录](../../logs/低空消防无人机工程日志.md)｜[当天日志§33](../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md)

**最新实测：专用密钥与免密sudo已通过。** 10月24日约04:02通过rk-uav以BatchMode登录jcfh，并在sudo -k清缓存后执行sudo -n id -u返回0；未重新安装公钥或修改权限，也不推断安装时间。见[日志§35](../../logs/2026-10-23_RK刷写、SSH修复与七包构建.md#35-yolo独立重训决定素材核查及离线依赖路线)。下方“板端待首次授权／未验收”为此前准备过程，不覆盖本次结果。

## 方案与实际准备状态

SSH密钥用于登录，RK上的sudoers规则决定能否免密码提权。本次复用jcfh账户，新增完整免密sudo规则后，该账户所有登录方式均受此规则影响；它不是只能由指定密钥触发的权限。持有本机专用私钥者可据此完整管理RK。

电脑端已生成独立Ed25519密钥，文件位于Windows用户.ssh目录，私钥无口令、移除继承ACL并限定当前执行身份访问。文件未放项目、未读取或输出私钥，不需上传到任何服务。公钥指纹为SHA256:f/BiE8up717mY+FPHgR244M0zDpi4HAW4UhIqqRgTtc，注释SHELL-Codex-RK3399ProD。SSH config已实际解析核对：

| 别名 | 地址 | 账户 |
| --- | --- | --- |
| rk-uav | 192.168.77.1（USB） | jcfh |
| rk-uav-wifi | 192.168.1.102（正式无线） | jcfh |

IdentityFile为~/.ssh/id_ed25519_codex_rk，限制只使用指定身份、禁止agent转发，连接超时10秒。不要把私钥内容发到聊天、文档或Git。

本任务使用电脑上的命令工具调用OpenSSH，不要求RK另装Codex。若使用App的原生SSH远程项目，官方流程还需要远端安装／认证Codex及PATH配置；本次未设置该功能。[OpenAI Docs远程连接说明](https://learn.chatgpt.com/docs/remote-connections)

## 1. SHELL首次安装公钥

在电脑PowerShell执行，按提示输入现有jcfh密码；首次SSH主机校验按原有可信主机信息确认，不关闭主机密钥检查：

```powershell
Get-Content "$env:USERPROFILE\.ssh\id_ed25519_codex_rk.pub" | ssh jcfh@192.168.77.1 'umask 077; mkdir -p ~/.ssh; echo >> ~/.ssh/authorized_keys; cat >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys'
```

只发送.pub公钥。现有authorized_keys内容保留，追加前先换行以避免与旧末行粘连；重复执行可能产生重复公钥行，不会自动撤销既有密钥。密码只在本机终端输入。

## 2. SHELL首次安装sudoers规则

在现有板端FinalShell／SSH终端执行，须为jcfh账户；第一次sudo可能要求现有密码：

```bash
rk_sudo_rule=$(mktemp)
printf '%s\n' 'jcfh ALL=(ALL:ALL) NOPASSWD: ALL' > "$rk_sudo_rule"
sudo visudo -cf "$rk_sudo_rule" && sudo install -o root -g root -m 0440 "$rk_sudo_rule" /etc/sudoers.d/99-codex-jcfh
rm -f "$rk_sudo_rule"
```

规则先在临时文件校验，通过后才安装到sudoers.d；独立文件可直接撤销，不替换主sudoers，不启用root密码SSH登录。无需重启sshd或RK。如visudo或install报错，保留首个错误，不继续将免密提权记为通过。

## 3. 无交互验收

在电脑PowerShell执行：

```powershell
ssh -o BatchMode=yes rk-uav "sudo -k; sudo -n id -u"
```

输出0且命令成功，表示密钥登录及清缓存后的免密root身份调用均通过；sudo -n禁止等待密码，避免把旧sudo缓存当成规则成功。随后Codex可用同样方式执行授权范围内的管理命令。例如只读检查：

```powershell
ssh -o BatchMode=yes rk-uav "uname -m; sudo -n systemctl is-active ssh"
```

首次公钥／sudoers安装仍依赖现有登录授权，不通过聊天收集密码。SSH权限不自动扩大无人机任务范围；真实外设和可能启动舵机／飞行任务的命令仍遵守项目约定。

## 4. 撤销

使用现有SSH或本地终端，删除本次独立规则即可恢复原sudo授权方式：

```bash
sudo rm /etc/sudoers.d/99-codex-jcfh
```

需要同时撤销这把密钥登录时，编辑~/.ssh/authorized_keys，仅删除注释为SHELL-Codex-RK3399ProD且指纹对应的公钥行，保留其他密钥。电脑密钥及别名如后续不用再单独移除；本次不自动删除其他凭据。

## 当前验证边界

已实做电脑端密钥生成、私钥ACL限制和ssh -G配置解析；板端公钥、sudoers规则及BatchMode／sudo -n结果待首次授权后核验。不因本机文件准备完成而宣称板端免密管理已完成。
