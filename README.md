# unitree-g1-brainco-hand

English Version | [中文版](./README_zh.md)

Tutorials and simple motion demos for adapting the BrainCo Revo2 dexterous hand to the Unitree G1 (Edu advanced version, 29 DOF).

## Version Notes
Update 2026.7.16
- Added the `eeg` brain-controlled robot task domain (G1-29 DOF only), which can connect to the BrainCo brain-controlled robot training platform.

Fix 2026.6.29
- **Important: the built-in Unitree arm control service is automatically disabled before startup** to avoid conflicts between upper-limb motions developed through `/arm_sdk` and the built-in Unitree arm control service, which could otherwise cause abnormal arm motion.

Update 2026.6.8:
- Improved the state management system with multiple task domains for easier task creation and management
- `calibrate`: camera calibration (G1-29 DOF only)
- `simple`: simple arm motions (supports G1-23 DOF and G1-29 DOF)
- Updated the remote UI control interface `ui_client` and its related APIs
- Updated the dexterous hand SDK in `ros2_stark_ws`

### Previous Versions

| Version | Update | Description | Access |
| ------- | ------- | ------- | ------- |
| v2.0.0 | 2026.6 | Current version | `main` branch |
| v1.1.0 | 2026.4 | Supports both G1-23 DOF and G1-29 DOF | `v1.1.0_basic` branch |
| v1.0.0 | 2025.11 | Supports only G1-23 DOF | [Releases v1.0.0](../../releases) |

## Repository Overview

| Dir | Function | Description |
| ------- | ------- | ------- |
| brainco_ws | main control | G1 arm IK computation is based on the official Unitree example [Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py). Dual-arm dual-hand control is based on ROS 2. |
| ros2_stark_ws | Revo2 Hand SDK | The BrainCo dexterous hands communicate with the Unitree G1 through **dual RS-485** serial ports. The left and right hands transmit data simultaneously through two `/dev/ttyUSB*` ports.<br>Dual-hand control is based on the official BrainCo ROS 2 example [stark-serialport-example/ros2_example](https://github.com/BrainCoTech/stark-serialport-example/tree/ros2_example). |
| cam_calibr | hand-eye calibration | |
| ui_client | control GUI | |

## Dexterous Hand Integration

[01 Dexterous Hand Installation, Robot Startup, and Remote Connection](./tutorials/README_01_pre_setup_en.md)

## Environment Setup

### The following should be deployed on the G1 robot

[02 Install Conda Environment and Torch](./tutorials/README_02_dependencies.md)

[03 Install Unitree-SDK, ROS2, URDF, BrainCo-SDK, Control System, and Build Configuration](./tutorials/README_03_unitree_setup_en.md)

[04 (Optional) Vision Setup](./tutorials/README_04_other_config.md)

### The following should be deployed on the host PC

[05 Host PC Setup](./tutorials/README_05_control_setup.md)

## Test Run

[06 Example 1: Simple Motions](./tutorials/README_06_test_run_en.md)

[07 Example 2: Hand-Eye Calibration](./tutorials/README_07_test_run_calibrate_en.md)

[08 Example 3: Brain-Controlled Robot](./tutorials/README_08_test_run_eeg_en.md)

## FAQ
[FAQ English Version](./tutorials/FAQ_en.md) or [FAQ 中文](./tutorials/FAQ_zh.md)

## Development Safety Recommendations
1. Before joint debugging, make sure all configuration files are set correctly. Recommended test order:
	- Test arm motion first, then install the dexterous hands for combined testing.
	- Test simple non-vision motions first.
	- Test replay motions next.
	- Test vision-based grasping motions next.
	- Test lower-limb movement or turning next.
	- Connect BrainCo brain-controlled robot training platform event mappings last.
2. For segmented motions, pay attention to whether there is an obvious jump between segments.
3. Upper-body motion control includes waist motion. If you only need to move the arms, fix the waist; otherwise waist movement can shift the target position.
4. For motions near a table, use `SafeArmDown` first. The normal `ArmDown` is more suitable when the robot is away from the table and there is enough space below.
5. If the robot moves abnormally or becomes uncontrollable during joint debugging:
   - Normal case: immediately click `Ready` to return the robot state to zero directly, or click `Ready (Table Safe)` to return to zero while avoiding the table.
   - Emergency case: click `Shutdown` directly to stop the program.
   - If the robot still cannot be controlled, **press and hold the remote controller `Damping Mode (L2 + B)` for more than 5 seconds** to force the robot into damping mode, then power it off.
6. Before formal joint debugging, actively clear the table and the space around the robot. Leave extra clearance in front of, beside, and below the arms whenever possible.
