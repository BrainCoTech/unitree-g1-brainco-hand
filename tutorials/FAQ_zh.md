# FAQ

[English Version](./FAQ_en.md) | 中文版

## 1. 配置自定义网络后，可能无法连接网络

```sh
# 移除默认路由
sudo ip route del default via 192.168.123.1 dev eth0
```


## 2. colcon build 显示系统时钟警告

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


## 3. not found
```
ros2: command not found
Package 'XXX' not found
```
以上问题出现可能是没有 source 正确的配置文件



## 4. 遇到任何 ROS 问题，可以尝试：

1. 关闭所有 Terminal，重新打开新的 Terminal
2. 删除 build install log 重新编译

## 5. 网络配置
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

## 6. SSH 成功 但 VScode 连接远程失败
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