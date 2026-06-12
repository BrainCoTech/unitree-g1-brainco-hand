#!/bin/bash

# 先加载 ROS2 环境
source ~/g1-brainco/brainco_ws/install/setup.bash

# 判断传入的参数
if [ "$1" == "agent" ]; then
    # agent 模式
    python src/control_py/control_py/smach_agent.py

else
    # 没有参数或其他参数
    ros2 run control_py smach_trans_node
fi
