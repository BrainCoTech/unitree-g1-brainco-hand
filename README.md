# unitree-g1-brainco-hand

强脑灵巧手Revo2适配宇树G1(Edu标准版23自由度)教程及简单动作示例。

## 代码库说明

### arm_ws

G1手臂IK计算基于宇树官方示例[Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py).  
Hello Demo [hello.py](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/arm_ws/src/control_py/control_py/hello.py).

### stark-serialport-example

本例中使用的灵巧手SDK与[原版SDK](https://github.com/BrainCoTech/stark-serialport-example/tree/revo2)区别: 
强脑灵巧手与宇树G1通过**双485**串口通信，即单ROS节点中左右手分别通过`/dev/ttyUSB0`和`/dev/ttyUSB1`串口同时传输信息。


## 灵巧手适配教程

视频演示(待添加)

### 1. 机器人启动
1. 宇树G1开机，具体可参照[宇树文档中心|操作指南](https://support.unitree.com/home/zh/G1_developer/quick_start)。接电时，灵巧手手背指示灯亮起，手指自动复位。
2. 等待（约1分钟）宇树G1进入**零力矩模式**，具体表现为随意活动关节无阻力。
3. 使用遥控器，按照说明按下对应按钮，使机器人先进入**阻尼模式**，再进入**锁定站立模式**。(注意：手臂开发**不进入**运动模式)

### 2. 远程连接
1. 具体可参考[宇树文档中心|快速开发](https://support.unitree.com/home/zh/G1_developer/quick_development)，首次需用网线连接宇树G1，将计算机以太网IP设置为与宇树G1同网段，默认是192.168.123.XXX。后续可按照[宇树文档中心|常见问题](https://support.unitree.com/home/zh/G1_developer/FAQ)配置WIFI，即可无线连接。
2. 打开VSCode，安装拓展Remote SSH，点击New Remote，输入`ssh unitree@192.168.XXX.XXX`(宇树G1的IP)，输入密码，即可连接。打开文件夹`/home/unitree/`访问所需文件。
3. 打开新的终端，需输入`1`(即选择ROS环境为Foxy)，按下回车，如需重新选择，可以输入`source ~/.bashrc`

### 3. 安装环境依赖
1. 安装Miniconda。进入[Miniconda官网](https://www.anaconda.com/docs/getting-started/miniconda/main)，选择系统：`Linux`，选择系统架构`ARM64`，按照官方提供的命令安装。
2. 创建conda环境，环境名为envname，使用python3.8
    ```
    conda create -n envname python=3.8
    conda activate envname
    ```
3. 按照[Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py)在新建的conda环境下安装依赖
    ```
    # 用于手臂运动控制
    conda install pinocchio -c conda-forge
    pip install meshcat

    # 用于conda环境下编译ROS2
    pip install rospkg
    pip install -U colcon-common-extensions
    ```

### 4. 安装宇树ROS
如宇树ROS未安装，可参考[宇树文档中心|ROS2通信例程](https://support.unitree.com/home/zh/G1_developer/ros2_communication_routine)安装。

### 5. 安装强脑灵巧手SDK
下载本仓库到G1
- 方法1：
    ```
    git clone https://github.com/BrainCoTech/unitree-g1-brainco-hand.git
    ```
- 方法2：
    下载到本地后上传
    ```
    scp -r arm_ws unitree@192.168.XXX.XXX:/home/unitree/
    scp -r stark-serialport-example unitree@192.168.XXX.XXX:/home/unitree/
    ```

### 6. 配置灵巧手
1. 打开`stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v2_double.yaml`，根据灵巧手配置输入左右手的串口、波特率、ID，如未设置过则使用默认参数。
2. 打开`stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/launch/stark_launch.py`，修改`parameters`路径为刚刚修改的`yaml`文件路径。

### 7. 测试Demo
1. 打开灵巧手ROS节点
2. 打开主控制ROS节点
    ```py
    self.show_hello(2, 10, "right", speed=1.5)  # 2~10秒右手持续挥手
    self.show_like(2, 3, "left")    # 2~3秒左手点赞
    ```

## FAQ
[FAQ.md](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/FAQ.md).