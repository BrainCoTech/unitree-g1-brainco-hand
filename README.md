# unitree-g1-brainco-hand

English Version | [中文版](./README_zh.md)

Tutorials and simple motion demos for adapting the BrainCo Revo2 dexterous hand to the Unitree G1 (Edu advanced version, 29 DOF).

## Version Notes
**Note:** This version supports only the G1 29-DOF model. To adapt it for the G1 23-DOF model, you need to modify the arm motion control parameters.

Update 2026.6.8:
- Improved the state management system with multiple task domains for easier task creation and management
- `calibrate`: camera calibration
- `simple`: simple arm motions
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

[07 Example 2: Hand-Eye Calibration](./tutorials/README_07_test_run_calibrate_zh.md)

## FAQ
[FAQ English Version](./tutorials/FAQ_en.md) or [FAQ 中文](./tutorials/FAQ_zh.md)
