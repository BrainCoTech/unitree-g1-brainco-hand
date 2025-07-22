# FAQ


## 1. 配置自定义网络后，无法连接网络

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

## 3. 在conda环境下如何编译
```sh
# 启动虚拟环境
conda activate env_name
# 安装所需依赖
(env_name) pip install rospkg
(env_name) pip install -U colcon-common-extensions
# 编译
(env_name) python -m colcon build
```

## 4. 运行节点报错 'GLIBCXX_3.4.XX' not found
```
ImportError: /aaa/bbb/libstdc++.so.6: version 'GLIBCXX_3.4.31' not found
```
（该问题解决来源于[这个网友](https://blog.csdn.net/L1481333167/article/details/137919464)的回答

1. 先分析一下问题，/aaa/bbb/libstdc++.so.6 中找不到'GLIBCXX_3.4.31'版本

```sh
# 在 /aaa/bbb/libstdc++.so.6 中查找字符串 GLIBCXX
strings /aaa/bbb/libstdc++.so.6 | grep GLIBCXX
```

查看查找结果，确实找不到 GLIBCXX_3.4.31，最高版本为 GLIBCXX_3.4.30  
记下报错的目录地址 `/aaa/bbb/`

2. 查看系统内其他的 libstdc++.so.6 是否有 GLIBCXX_3.4.31

```sh
# 列出所有的 libstdc++.so.6.X.XX
sudo find / -name "libstdc++.so.6*"
```

随机查看几个，可能找到 GLIBCXX_3.4.31，比如：

```sh
# 在不同目录下 libstdc++.so.6.X.XX 中查找字符串 GLIBCXX（找 XX 数字最大的，最可能出现）
strings /lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
strings /ccc/ddd/eee/libstdc++.so.6.X.XX | grep GLIBCXX
```
记下找到 GLIBCXX_3.4.31 的路径，如 `/ccc/ddd/eee/libstdc++.so.6.X.XX`  
如仍未找到，看下一条

3. 如果已找到 GLIBCXX_3.4.31，忽略这一步，如果找不到：

```sh
# conda 安装 libstdcxx-ng
conda install -c conda-forge libstdcxx-ng
# 再次列出所有的 libstdc++.so.6.X.XX
sudo find / -name "libstdc++.so.6*"
```

在刚刚打印的结果中找到刚刚 conda 安装的 libstdc++.so.6.X.XX，比如：  
`/home/anaconda3/fff/ggg/lib/libstdc++.so.6.X.XX`，查看这个新安装的路径里能否找到 GLIBCXX_3.4.31
```sh
# 在 conda 新安装的 libstdc++.so.6.X.XX 中查找字符串 GLIBCXX
strings /home/anaconda3/fff/ggg/lib/libstdc++.so.6.X.XX | grep GLIBCXX
```
这个打印结果里有 GLIBCXX_3.4.31，记下路径 `/home/anaconda3/fff/ggg/lib/libstdc++.so.6.X.XX`

4. 将记下的路径复制到报错的目录下
```sh
cp /ccc/ddd/eee/libstdc++.so.6.X.XX /aaa/bbb/
```

检查是否成功复制
```sh
sudo find /aaa/bbb/ -name "libstdc++.so.6*"
```
看到打印结果中出现 `/aaa/bbb/libstdc++.so.6.X.XX`，表示复制成功

5. 软链接 /aaa/bbb/libstdc++.so.6 → /aaa/bbb/libstdc++.so.6.X.XX
```sh
# 创建软链接
sudo ln -sf /aaa/bbb/libstdc++.so.6.X.XX /aaa/bbb/libstdc++.so.6
# 查看软链接是否成功创建
ls -l /aaa/bbb/libstdc++.so.6
```

6. 最后检查报错的 /aaa/bbb/libstdc++.so.6 中能否找到'GLIBCXX_3.4.31'
```sh
strings /aaa/bbb/libstdc++.so.6 | grep GLIBCXX
```
打印结果中有 GLIBCXX_3.4.31，重新编译运行，不再报错

## 0. 遇到任何 ROS 问题，可以尝试：

1. 关闭所有 Terminal，重新打开新的 Terminal
2. 删除 build install log 重新编译