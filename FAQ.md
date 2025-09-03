# FAQ


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