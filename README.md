# unitree-g1-brainco-hand
Unitree G1 with BrainCo hand Revo2. Install, setup and a simple demo.

### arm_ws
Arm IK is based on  [Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py).

### stark-serialport-example
The Stark SDK can be found [here(original version)](https://github.com/BrainCoTech/stark-serialport-example/tree/revo2). The only difference between this SDK version and the original one is that Unitree G1 and the BrainCo hands communicate via **double 485** interfaces. It means the left and right hand messages are transmitted via ttyUSB0 and ttyUSB1 simultaneously. In the SDK version, the two interfaces are accessed separately to transmit data from both hands **in one ROS node**.

