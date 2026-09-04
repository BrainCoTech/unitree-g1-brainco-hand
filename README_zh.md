# unitree-g1-brainco-hand

[English Version](./README.md) | 中文版

强脑灵巧手 Revo2 适配宇树 G1 机器人教程及简单动作示例。

## 版本说明
更新 2026.7.16
- 新增 `eeg` 脑控机器人任务域 (仅支持 G1-29 DOF)，可连接 BrainCo 脑控机器人训练平台。
- 注意: 所有上肢动作没有锁定腰部 WaistRoll 和 WaistPitch 关节，建议使用单腰模式或者自行补充代码。

修复 2026.6.29
- **重要: 启动前自动关闭宇树内置的手臂控制服务**，避免通过 `/arm_sdk` 开发的上肢动作与宇树内置手臂控制服务冲突，导致手臂动作异常。

更新 2026.6.8: 
- 完善状态管理系统，包含多个任务域，方便新建与管理任务 
	- `calibrate` 相机校准 (仅支持 G1-29 DOF)
	- `simple` 简单手臂动作（支持 G1-23 DOF 和 G1-29 DOF）
- 更新远程 UI 控制界面 `ui_client` 与对应接口
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

[04 (可选) Vision 配置](./tutorials/README_04_other_config.md)

### 以下需部署到控制端 PC

[05 Host PC Setup](./tutorials/README_05_control_setup.md)  

## 测试运行

[06 Test Run 示例1 简单动作](./tutorials/README_06_test_run_zh.md) 

[07 Test Run 示例2 手眼标定](./tutorials/README_07_test_run_calibrate_zh.md)

[08 Test Run 示例3 脑控机器人](./tutorials/README_08_test_run_eeg_zh.md)

## FAQ
[FAQ 中文](./tutorials/FAQ_zh.md) or [FAQ English version](./tutorials/FAQ_en.md)

## 开发安全建议
1. 联调前确保所有配置文件正确配置，推荐测试顺序：
	- 先测试手臂动作，再安装灵巧手联合测试
	- 先测试简单非视觉动作
	- 再测试 Replay 动作
	- 再测试视觉抓取动作
	- 再测试下肢移动或转向
	- 最后再接 BrainCo 脑控机器人训练平台事件映射
2. 对于分段动作，建议多留意两段之间是否有明显跳变
3. 注意上半身运控包含腰部动作，如果仅动手臂需固定腰部，否则腰部会被带动导致目标位置偏移
4. 桌面附近的动作，建议优先使用 SafeArmDown，普通 ArmDown 更适合远离桌面、下方空间足够的情况
5. 联调时如果机器人动作异常失控:
   - 普通情况：立即点击 `Ready` 直接控制机器人状态回零，或者 `Ready (Table Safe)` 绕过桌面回零
   - 紧急情况：直接点击 `Shutdown` 结束程序
   - 如仍无法控制机器人，**长按遥控器 `阻尼模式(L2 + B)` 5s 以上** 强制机器人进入阻尼模式，然后关闭电源
6. 建议在正式联调前先主动清理桌面和机器人周边空间，手臂前方、侧方、下方最好都预留余量