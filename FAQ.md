# FAQ

### 配置自定义网络后，无法连接网络

可以尝试移除默认路由
```sh
sudo ip route del default via 192.168.123.1 dev eth0
```

### colcon build 显示系统时钟警告

```
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
```

可能是无法连接网络导致系统时钟无法自动更新
```sh
# 查看系统时间，显示1970年
date
```

同样尝试移除默认路由
```sh
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

