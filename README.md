# unitree-g1-brainco-hand

[English Version](./README_en.md) | 中文版

强脑灵巧手 Revo2 适配宇树 G1(Edu进阶版29自由度) 教程及简单动作示例。

## 版本说明
**注意:** 该版本仅支持 G1-29 DOF，适配 G1-23 DOF 需修改手臂动作控制参数。

更新 2026.6.8: 
- 完善状态管理系统，包含多个任务域，方便新建与管理任务 
	- `calibrate` 相机校准
	- `simple` 简单手臂动作
- 更新远程 UI 控制界面 `ui_client` 与对应接口，可连接 BrainCo 脑控机器人软件
- 更新灵巧手SDK `ros2_stark_ws` 版本

### 历史版本

| Version | Update  | Description | Access |
| ------- | ------- | ------- | ------- |
| v2.0.0 | 2026.6  | 当前版本 | main 分支 |
| v1.1.0 | 2026.4  | 支持 G1-23DOF 和 G1-29 DOF | v1.1.0_basic 分支 |
| v1.0.0 | 2025.11 | 仅支持 G1-23DOF | [Releases v1.0.0](../../releases) |


## 代码库说明


| Dir | Function | Description |
| ------- | ------- | ------- |
| brainco_ws | main control | G1 手臂 IK 计算基于宇树官方示例 [Unitree/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py)。双臂双手控制基于ROS2。 |
| ros2_stark_ws | Revo2 Hand SDK  | 强脑灵巧手与宇树 G1 通过**双485**串口通信，左右手分别通过两个`/dev/ttyUSB*`串口同时传输信息。<br>双手控制基于 Brainco 官方 ROS2 示例 [stark-serialport-example/ros2_example](https://github.com/BrainCoTech/stark-serialport-example/tree/ros2_example)。 |
| cam_calibr | 手眼标定  | |
| ui_client | 控制 GUI | |


## 灵巧手适配

[01 灵巧手安装, 机器人启动, 远程连接](./tutorials/README_01_pre_setup_zh.md)


## 环境配置

### 以下需部署到 G1 Robot

[02 安装 Conda 环境, Torch](./tutorials/README_02_dependencies.md)

[03 安装 Unitree-SDK, ROS2, URDF, BrainCo-SDK, Control System, 配置和编译](./tutorials/README_03_unitree_setup_zh.md)


### 以下需部署到外部控制 PC

[04 Host PC Setup](./tutorials/README_04_control_setup_zh.md)  

## 测试运行

[05 Test Run](./tutorials/README_05_test_run_zh.md)

## FAQ
[FAQ 中文](./tutorials/FAQ_zh.md) or [FAQ English version](./tutorials/FAQ_en.md)