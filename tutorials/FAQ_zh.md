# FAQ

[English Version](./FAQ_en.md) | 中文版

## System

### Q1. 配置自定义网络后，可能无法连接网络

```sh
# 移除默认路由
sudo ip route del default via 192.168.123.1 dev eth0
```


### Q2. colcon build 显示系统时钟警告

```
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
```

```sh
# 查看系统时间
date
```
显示1970年，可能是无法连接网络导致系统时钟无法自动更新

```sh
# 移除默认路由
sudo ip route del default via 192.168.123.1 dev eth0
```

关闭并重新打开终端
```sh
# 查看系统时间同步情况
timedatectl status
```
时间更新成功

删除 build install log 文件，重新编译
```sh
# 注意在正确的 ws 中运行以下命令
rm -rf build install log
colcon build
```
不再报错


### Q3. not found
```
ros2: command not found
Package 'XXX' not found
```
以上问题出现可能是没有 source 正确的配置文件



### Q4. 遇到任何 ROS 问题，可以尝试：

1. 关闭所有 Terminal，重新打开新的 Terminal
2. 删除 build install log 重新编译

### Q5. 网络配置
```sh
unitree@ubuntu:~$ sudo ifconfig wlan0 up 
# 报错 SIOCSIFFLAGS: Operation not possible due to RF-kill
```

解决:
```sh
# 查看
unitree@ubuntu:~$ rfkill list
# 0: phy0: Wireless LAN
#         Soft blocked: yes
#         Hard blocked: no

unitree@ubuntu:~$ sudo rfkill unblock wifi

# 再查看
unitree@ubuntu:~$ rfkill list
# 0: phy0: Wireless LAN
#         Soft blocked: no
#         Hard blocked: no

# 重新运行不再报错
unitree@ubuntu:~$ sudo ifconfig wlan0 up
```

### Q6. SSH 成功 但 VScode 连接远程失败
首先，Download Server 需联网，确认本地网络连接正常

如果多次要求重新输入密码，且显示
```
Failed to connect to the remote extension host server (Error: ExtensionInstallFailed(ExtensionInstallFailed("Error while installing extensions: getaddrinfo EAI_AGAIN marketplace.visualstudio.com\ngetaddrinfo EAI_AGAIN marketplace.visualstudio.com\n")
```
是 VS Code 远程 SSH 到 Linux 服务器时无法下载 VSCode 插件 的典型网络问题。DNS 解析失败或超时，即 Linux 服务器无法访问 marketplace.visualstudio.com

解决:
1. 通过 Windows Powershell 或 Linux 命令行 ssh 连接远程
```sh
ssh unitree@192.168.xx.xx
```

2. 在 Linux 服务器上运行：
```sh
unitree@ubuntu:~$ ping marketplace.visualstudio.com
```
如失败，则是 Linux 服务器的外网访问问题

3. 运行：
```sh
unitree@ubuntu:~$ cat /etc/resolv.conf
```
显示 
```
nameserver 127.0.0.53 
search Huawei.Local
``` 
这意味着 Ubuntu 正在使用 systemd-resolved 的本地 DNS 缓存代理（127.0.0.53），但它无法解析外部域名。

4. 运行：
```sh
unitree@ubuntu:~$ sudo vim /etc/systemd/resolved.conf
```

找到或添加如下几行（去掉 #）：
```
[Resolve] DNS=8.8.8.8 1.1.1.1 
FallbackDNS=114.114.114.114 
DNSStubListener=yes
```
保存

5. 重启 DNS 服务
```sh
unitree@ubuntu:~$ sudo systemctl restart systemd-resolved
```

6. 检查 DNS 是否生效
```sh
unitree@ubuntu:~$ systemd-resolve --status
```

如显示 
```
DNS Servers: 8.8.8.8 1.1.1.1
Fallback DNS Servers: 114.114.114.114 8.8.8.8
```
则已生效

按 `q` 退出查看

7. 再次测试网络

```sh
unitree@ubuntu:~$ ping 8.8.8.8
unitree@ubuntu:~$ ping marketplace.visualstudio.com
```
如果能 ping 通, 则重启 VScode 远程 SSH 不会再出现问题

## Control


### Q1 Client 扫描不到机器人设备

优先检查：
- `./launch_robot.sh` 以及 `./launch_trans.sh agent` 是否已启动，且输出正常
- 机器人与控制端是否在同一网络
- `46000` 和 `37020` 是否被占用

### Q2 平台看不到新增动作

优先检查：
- 是否重新编译
- 当前模式是否正确 `src/control_py/config/smach_config.yaml`
- 新动作是否加入 `active.children`
- 该编号是否被加入 `hidden_action_reg`
- 是否重启了 Agent 并重新扫描设备

### Q3 动作能触发，但不能切换下一步

优先检查：
- `active_expected_duration` 是否过长
- `allowed_sub_trans` 是否定义了不允许切换的状态
- 使用 `auto` 的动作是否正确调用了 `mark_active_action_done()`

### Q4 抓取为什么总是右手执行

当前代码中，部分抓取流程入口和中间步骤都存在固定 `right` 的逻辑。  
如果要支持左手或双手，需要同步修改 `on_xx_handler` 中的处理逻辑。

### Q5 识别物体后动作为什么不开始

常见原因：
- 物体没稳定识别（当前有稳定检测逻辑，如手的晃动可能导致手中的物体无法通过稳定检测判定）
- 物体检测边框离相机边缘过近

