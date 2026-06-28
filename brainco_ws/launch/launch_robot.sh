#!/bin/bash

echo "Turning off g1 arm action service..."
cd ~/unitree_sdk2/build/bin
./turn_off_arm_action_service eth0

if [ $? -ne 0 ]; then
    echo "Failed to turn off arm action service!"
    exit 1
fi

echo "Done."

sleep 1

echo "Starting launch_robot.sh..."
cd ~/unitree-g1-brainco-hand/brainco_ws

source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/unitree-g1-brainco-hand/brainco_ws/install/setup.bash
source ~/unitree-g1-brainco-hand/ros2_stark_ws/install/setup.bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1

# ENV_NAME="detect8"

# # 初始化 conda（非常关键）
# source ~/miniconda3/etc/profile.d/conda.sh

# # 判断当前环境
# if [ "$CONDA_DEFAULT_ENV" != "$ENV_NAME" ]; then
#     echo "Activating conda environment: $ENV_NAME"
#     conda activate $ENV_NAME
# else
#     echo "Already in conda environment: $ENV_NAME"
# fi

# 设置 libgomp
export LD_PRELOAD=$CONDA_PREFIX/lib/libgomp.so.1
echo "LD_PRELOAD set to: $LD_PRELOAD"

# 判断参数
if [ "$1" = "arm" ]; then
    echo "Launch only arm..."
    ros2 launch ~/unitree-g1-brainco-hand/brainco_ws/src/control_py/launch/smach_launch.py

elif [ "$1" == "body" ]; then
    echo "Launch whole body..."
    ros2 launch ~/unitree-g1-brainco-hand/brainco_ws/launch/whole_body_launch.py

else
    echo "Launch arm and hand..."
    ros2 launch ~/unitree-g1-brainco-hand/brainco_ws/launch/multi_launch.py
fi