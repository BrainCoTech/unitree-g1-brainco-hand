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
2. 创建conda环境，环境名为envname，使用python3.8
```sh
conda create -n envname python=3.8
conda activate envname
```
3. 按照[Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py)在新建的conda环境下安装依赖
```sh
# 用于手臂运动控制
conda install pinocchio -c conda-forge
pip install meshcat
# 用于conda环境下编译ROS2
pip install rospkg
pip install -U colcon-common-extensions
```

### 安装宇树ROS
如宇树ROS未安装，可参考[宇树文档中心|ROS2通信例程](https://support.unitree.com/home/zh/G1_developer/ros2_communication_routine)安装。

### 安装强脑灵巧手SDK
下载本仓库到G1
- 方法1：
```sh
git clone https://github.com/BrainCoTech/unitree-g1-brainco-hand.git
```
- 方法2：
    下载到本地后上传
```sh
scp -r arm_ws unitree@192.168.XXX.XXX:/home/unitree/
scp -r stark-serialport-example unitree@192.168.XXX.XXX:/home/unitree/
```

### 配置灵巧手
1. 打开`stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v2_double.yaml`，根据灵巧手配置修改参数，通常使用默认参数。  
此处`port_l`和`port_r`对应左右手串口，分别对应USB-485板的`485_A`，`485_B`信号端口，`baudrate`为波特率，左右手`slave_id`默认为`0x7e`和`0xyf`。

2. 打开`stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/launch/stark_launch.py`，修改`parameters`路径为刚刚修改的`yaml`文件路径。

### 测试Demo

##### 运行灵巧手ROS节点
1. 新打开一个终端，灵巧手节点无需conda环境
```sh
cd stark-serialport-example/ros2_stark_ws/          # 进入灵巧手工作空间
colcon build                                        # 编译节点
source install/setup.bash                           # 加载工作空间环境配置  
ros2 launch ros2_stark_controller stark_launch.py   # 运行灵巧手节点
```
2. 检查输出信息
- 左右手 Port，Baudrate，slave_id 都正确
- 串口已打开 serial port opened
- 正在等待关节控制命令 Waiting for joint cmd ...

如果未出现上述信息，可能是config文件参数不正确，或launch文件里地址不正确


##### 运行主控制ROS节点
1. 新打开一个终端，手臂控制需激活新建的环境
```sh
conda activate envname              # 激活conda环境
cd arm_ws/                          # 进入主控制工作空间
python -m colcon build              # 在conda环境下编译节点
source install/setup.bash           # 加载工作空间环境配置
source ~/unitree_ros2/setup.sh      # 加载宇树ROS工作空间环境配置
ros2 run control_py hello_pub       # 运行主控制节点
```

2. IK初始化完毕后，按下回车机器人手臂和灵巧手开始运动，Ctrl+C 可随时停止  
Hello Demo [hello.py](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/arm_ws/src/control_py/control_py/hello.py)包含一个简单的示例
```py
self.show_hello(2, 10, "right", speed=1.5)  # 2~10秒右手持续挥手
self.show_like(2, 3, "left")    # 2~3秒左手点赞
```

3. 查看ROS信息
```sh
ros2 node list
```
/hello_pub 为主控制节点  
/stark_node 为灵巧手节点  
/ros_bridge 为宇树节点  

```sh
ros2 topic list
```
/arm_sdk 为传递宇树手臂控制信息的话题  
/joint_commands_left 和 /joint_commads_right 分别为传递左右手控制信息的话题


```sh
cd ~/unitree-g1-brainco-hand/brainco_ws
# 启动机器人节点和灵巧手节点
(g1brainco) unitree@ubuntu:~/unitree-g1-brainco-hand/brainco_ws$ ./launch/launch_robot.sh
# 启动状态转换 client 节点
(g1brainco) unitree@ubuntu:~/unitree-g1-brainco-hand/brainco_ws$ ./launch/launch_trans.sh
```


## FAQ
[FAQ.md](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/FAQ.md).