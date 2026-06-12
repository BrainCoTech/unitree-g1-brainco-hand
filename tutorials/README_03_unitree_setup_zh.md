## Install Unitree-SDK

参考 [宇树文档中心|应用开发](https://support.unitree.com/home/zh/G1_developer/get_sdk) 安装并编译 [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) 

## Install Unitree-ROS2
1. 参考 [宇树文档中心|ROS2通信例程](https://support.unitree.com/home/zh/G1_developer/ros2_communication_routine)，安装并编译 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

2. 打开`~/unitree_ros2/setup.sh`，修改`"enp3s0"`为`"eth0"`
```xml
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

## Download Unitree G1 URDF

下载官方模型 [g1-description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)，用于 IK 计算

打开 `brainco_ws\src\control_py\control_py\action_pkg\robot_control.py`, 修改 `arm_urdf_path` 路径。


## Install BrainCo SDK and Control System
```sh
cd ~
git clone https://github.com/BrainCoTech/unitree-g1-brainco-hand.git
```

设置可执行权限
```sh
cd ~/unitree-g1-brainco-hand/brainco_ws 
chmod +x ./launch/launch_trans.sh
chmod +x ./launch/launch_robot.sh
```

## Setup Hand Control Parameters
修改 `ros2_stark_ws/src/ros2_stark_controller_new/config/` 目录下的 `params_revo2_left.yaml` 和
`params_revo2_right.yaml`
- `port`: 左右手串口，分别对应USB-485板的两个信号端口
- `baudrate`: Revo2 波特率默认 `460800`
- `slave_id`: 左手默认 `0x7e` 右手默认 `0x7f`

## Build the Workspaces

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
