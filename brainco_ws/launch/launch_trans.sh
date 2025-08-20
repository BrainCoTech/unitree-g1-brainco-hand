#!/bin/bash
source ~/g1-brainco/brainco_ws/install/setup.bash
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
ros2 run control_py trans_node
