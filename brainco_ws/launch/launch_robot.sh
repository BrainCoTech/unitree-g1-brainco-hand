#!/bin/bash
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/g1-brainco/brainco_ws/install/setup.bash
source ~/g1-brainco/ros2_stark_ws/install/setup.bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
ros2 launch ~/g1-brainco/brainco_ws/launch/multi_launch.py
