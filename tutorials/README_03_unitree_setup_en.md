## Install Unitree-SDK

Refer to the [Unitree Documentation Center | Application Development](https://support.unitree.com/home/en/G1_developer/get_sdk).
1. Download [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
2. Copy `scripts/turn_off_arm_action_service.cpp` to `unitree_sdk2/example/g1/high_level/`.
3. Modify `unitree_sdk2/example/g1/CMakeLists.txt` accordingly.
4. Build and install `unitree_sdk2`.

## Install Unitree-ROS2
1. Refer to the [Unitree Documentation Center | ROS2 Communication Routine](https://support.unitree.com/home/en/G1_developer/ros2_communication_routine) to install and build [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2).

2. Open `~/unitree_ros2/setup.sh` and change `"enp3s0"` to `"eth0"`:

```xml
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

## Download the Unitree G1 URDF

Download the official model [g1-description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description) for IK computation.

Open `brainco_ws/src/control_py/control_py/action_pkg/robot_control.py` and modify the `arm_urdf_path` path.

## Install BrainCo SDK and Control System
```sh
cd ~
git clone https://github.com/BrainCoTech/unitree-g1-brainco-hand.git
```

Set executable permissions:
```sh
cd ~/unitree-g1-brainco-hand/brainco_ws
chmod +x ./launch/launch_trans.sh
chmod +x ./launch/launch_robot.sh
```

Install the BrainCo SDK library files:
```sh
cd ~/unitree-g1-brainco-hand/ros2_stark_ws
chmod +x download-lib.sh
./download-lib.sh
mkdir src/ros2_stark_controller_new/lib
mv dist/shared/linux/libbc_stark_sdk.so src/ros2_stark_controller_new/lib/libbc_stark_sdk.so
```

## Set Up Hand Control Parameters
Modify `params_revo2_left.yaml` and `params_revo2_right.yaml` under `ros2_stark_ws/src/ros2_stark_controller_new/config/`:

- `port`: The serial ports for the left and right hands, corresponding to the two signal ports on the USB-to-485 board.
- `baudrate`: The default baud rate for Revo2 is `460800`.
- `slave_id`: The default is `0x7e` for the left hand and `0x7f` for the right hand.

## Build the Workspaces

```sh
conda activate g1brainco
# Build brainco_ws
cd ~/unitree-g1-brainco-hand/brainco_ws
python -m colcon build
# Build ros2_stark_ws
cd ~/unitree-g1-brainco-hand/ros2_stark_ws
python -m colcon build
```
