
# 简介

[English Version](./README_en.md) | 中文版

本项目为 [unitreerobotics/brainco_hand_service]((https://github.com/unitreerobotics/brainco_hand_service))

## 环境准备
将 [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) 和 [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service) 克隆或上传到机器人开发单元.

```sh
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/brainco_hand_service.git
```

创建新的示例
```sh
cd ~/unitree_sdk2/example/g1
mkdir brainco_hand & cd brainco_hand
touch g1_arm7_brainco_example.cpp
```

复制并粘贴 `brainco_hand_service_extension/g1_arm7_brainco_example.cpp`


编辑 `CMakeLists.txt` 

1. 修改 `unitree_sdk2/example/g1/CMakeLists.txt`

    ```cmake
    # 添加在最前面
    add_executable(g1_arm7_brainco_example brainco_hand/g1_arm7_brainco_example.cpp)
    target_link_libraries(g1_arm7_brainco_example unitree_sdk2)
    ```

2. 修改  `unitree_sdk2/CMakeLists.txt` 替换成 `brainco_hand_service_extension/CMakeLists.txt`.

## 编译
确保系统时间正确
```sh
# 查看当前时间
date
```

如果系统时间未更新
```sh
# 删除默认路由
sudo ip route del default via 192.168.123.1 dev eth0
# 重启系统时间同步服务
sudo systemctl restart systemd-timesyncd
# 查看当前时间
date
```

编译 `brainco_hand_service`
```sh
cd ~/brainco_hand_service
mkdir build && cd build
cmake ..
make -j6
```

编译 `unitree_sdk2`
```sh
cd ~/unitree_sdk2
mkdir build & cd build
cmake ..
sudo make install
```

## 运行

### 终端 1 保持运行
```sh
cd ~/brainco_hand_service/bin
# 启动服务器 (eth0 或 wlan0)
sudo ./brainco_hand_server --network eth0
```

### 终端 2
#### 手部测试
```sh
cd ~/brainco_hand_service/bin
# test left hand
sudo ./test_brainco_hand_server left
# test right hand
sudo ./test_brainco_hand_server right
```

#### 臂手联合测试
```sh
cd ~/unitree_sdk2/build/bin
# 测试机械臂与手部（无论有线或无线，此处必须使用 eth0）
./g1_arm7_brainco_example eth0
```
按下 ENTER 手臂和手开始运动
按下 R 手臂和手复位