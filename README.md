# unitree-g1-brainco-hand

强脑灵巧手Revo2适配宇树G1(Edu标准版23自由度)教程及简单动作示例。

## 代码库说明

### brainco_ws

G1手臂IK计算基于宇树官方示例[Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py)。双臂双手控制基于ROS2。
- Main control [smach_action.py](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/brainco_ws/src/control_py/control_py/smach_action.py)
- State machine transition client [keyboard_call.py](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/brainco_ws/src/control_py/control_py/keyboard_call.py)


### ros2_stark_ws

本例中使用的灵巧手SDK与[原版SDK](https://github.com/BrainCoTech/stark-serialport-example/tree/revo2/ros2_stark_ws)区别: 
强脑灵巧手与宇树G1通过**双485**串口通信，即单ROS节点中左右手分别通过`/dev/ttyUSB0`和`/dev/ttyUSB1`串口同时传输信息。
- Brainco hands [stark_node.cpp](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/ros2_stark_ws/src/ros2_stark_controller/src/stark_node.cpp)


## 灵巧手适配教程

视频演示(待添加)

### 机器人启动
1. 宇树G1开机，具体可参照[宇树文档中心|操作指南](https://support.unitree.com/home/zh/G1_developer/quick_start)。接电时，灵巧手手背指示灯亮起，手指自动复位。
2. 等待（约1分钟）宇树G1进入**零力矩模式**，具体表现为随意活动关节无阻力。
3. 使用遥控器，按照说明按下对应按钮，使机器人依次进入**阻尼模式** → **锁定站立模式**。(注意：手臂开发**不进入**运动模式)

### 远程连接
参考[宇树文档中心|快速开发](https://support.unitree.com/home/zh/G1_developer/quick_development)。
1. 首次连接使用网线连接G1和计算机，将计算机以太网IP设置为与宇树G1同网段 `192.168.123.XXX`,如：
```
IP          192.168.123.222
Subnet      255.255.255.0
Gateway     192.168.1.1
DNS         192.168.1.1
```

2. 打开VSCode，安装拓展Remote SSH，点击New Remote，输入
```sh
ssh unitree@192.168.123.164
```
输入密码（默认`123`）。连接成功后打开文件夹`/home/unitree/`访问所需文件。

3. 打开新的终端，输入`1`(即选择ROS环境为Foxy)，按下回车。如需重新选择，可以输入`source ~/.bashrc`

4. 配置WIFI：[宇树文档中心|常见问题](https://support.unitree.com/home/zh/G1_developer/FAQ) → Jetson Orin Nx WIFI 配置方法 → STA模式 → nmcli配置WIFI方式 

如果网络`<SSID>`或密码包含**特殊字符、中文或空格**，需使用双引号，如
```sh
nmcli device wifi connect "我的WiFi" password "mypass@123!"
```
如果报错`Not authorized`则加`sudo`

5. 固定远程IP地址：
```sh
# 查看 wlan0 网络详细信息
nmcli device show wlan0

# 确认选择的IP是否被占用
nmap -sn 192.168.13.60
# 显示`Host seems down`则

# 如固定IP为 192.168.13.60
sudo nmcli connection modify <SSID> ipv4.method manual \
    ipv4.addresses 192.168.13.60/23 \
    ipv4.gateway 192.168.13.1 \
    ipv4.dns "192.168.13.1 8.8.8.8"

# 断开网络后重新连接
sudo nmcli connection down <SSID> && sudo nmcli connection up <SSID>
```


### 安装环境依赖
1. 安装Miniconda。进入[Miniconda官网](https://www.anaconda.com/docs/getting-started/miniconda/main)，选择系统：`Linux`，选择系统架构`ARM64`，按照官方提供的命令安装。
2. 创建conda环境，环境名为g1brainco，使用python3.8
```sh
conda create -n g1brainco python=3.8
conda activate g1brainco
```
3. 在新建的conda环境下安装依赖
```sh
# 用于手臂运动控制
conda install pinocchio -c conda-forge
pip install meshcat
pip install transitions
# 用于conda环境下编译ROS2
pip install rospkg
pip install -U colcon-common-extensions
# 其他依赖
pip install matplotlib
pip install empy==3.3.2
pip install lark-parser
```

### 安装宇树ROS
1. 参考[宇树文档中心|ROS2通信例程](https://support.unitree.com/home/zh/G1_developer/ros2_communication_routine)，安装并编译`unitree_ros2`

2. 打开`~/unitree_ros2/setup.sh`，修改`"enp3s0"`为`"eth0"`
```xml
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

### 安装强脑灵巧手SDK
1. 下载本仓库到G1
- 方法1：
```sh
cd ~
git clone https://github.com/BrainCoTech/unitree-g1-brainco-hand.git
```
- 方法2：
    下载到本地后上传
```sh
scp -r unitree-g1-brainco-hand unitree@192.168.XXX.XXX:/home/unitree/
```

2. 设置可执行权限
```sh
cd ~/unitree-g1-brainco-hand/brainco_ws 
chmod +x ./launch/launch_trans.sh
chmod +x ./launch/launch_robot.sh
```

### 配置灵巧手
打开`ros2_stark_ws/src/ros2_stark_controller/config/params_v2_double.yaml`，根据灵巧手配置修改参数，通常使用默认参数。  
- `port_l, port_r`: 左右手串口，分别对应USB-485板的`485_A, 485_B`信号端口
- `baudrate`: 波特率
- `slave_id_l, slave_id_r`: 左手默认`0x7e`右手默认`0x7f`。

### 编译

```sh
# 激活 conda 环境
conda activate g1brainco
# 编译 brainco_ws
cd ~/unitree-g1-brainco-hand/brainco_ws 
python -m colcon build    
# 编译 ros2_stark_ws
cd ~/unitree-g1-brainco-hand/ros2_stark_ws
python -m colcon build          
```


## 测试运行

同时开启两个终端

### 终端1: 启动主控制节点和灵巧手节点
```sh
conda activate g1brainco                    # 激活conda环境
cd ~/unitree-g1-brainco-hand/brainco_ws     # 进入工作空间      
./launch/launch_robot.sh                    # 运行 launch 文件
```

检查输出信息:
- 左右手 `Port`，`Baudrate`，`slave_id` 都正确
- 串口已打开 `"serial port opened"`
- 正在等待关节控制命令 `"Waiting for joint cmd ..."`
如果未出现上述信息，可能是config文件参数不正确

- IK初始化结束 `"IK initialization done."`
- 当显示 `"Request 'configure' to start"` 则可以发送状态转换请求


### 终端2: 启动状态转换 client 节点
```sh
conda activate g1brainco                    # 激活conda环境
cd ~/unitree-g1-brainco-hand/brainco_ws     # 进入工作空间      
./launch/launch_trans.sh                    # 运行 launch 文件
```

### 请求状态切换
**终端2**会提示当前状态、可使用的转换和对应的动作。输入`字符(串) + 回车`转换状态。进入`active`状态后，在字母后加`l`或`r`单独控制左/右手，不加则默认双手。
<p align="center">
  <img src="brainco_ws/figs/ros2_statemachine.png" alt="statemachine" width="600"/>
</p>

## FAQ
[FAQ.md](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/FAQ.md).