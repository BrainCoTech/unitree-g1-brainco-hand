# unitree-g1-brainco-hand
Unitree G1 with BrainCo hand Revo2. Install, setup and a simple demo.

## 1. arm_ws
Arm IK is based on  [Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py).
This workspace contains a hello demo [hello.py](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/arm_ws/src/control_py/control_py/hello.py).
```py
self.show_hello(2, 10, "right", speed=1.5)  # 2~10秒右手持续挥手
self.show_good(2, 3, "left")    # 2~3秒左手点赞
```

## 2. stark-serialport-example
The Stark SDK can be found [here(original version)](https://github.com/BrainCoTech/stark-serialport-example/tree/revo2).  

Difference between this SDK version and the original one:  
Unitree G1 and the BrainCo hands communicate via **double 485** interfaces. In this SDK version, the left and right hand messages are transmitted via /dev/ttyUSB0 and /dev/ttyUSB1 simultaneously **in one ROS node**.

## 3. FAQ
See [FAQ.md](https://github.com/BrainCoTech/unitree-g1-brainco-hand/blob/main/FAQ.md).