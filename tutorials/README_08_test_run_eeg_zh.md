## 示例3：脑控机器人任务

### 1. 机器人配置

#### 通用配置
修改通用配置 `config/smach_config.yaml`
```yaml
mode:
  current_mode: eeg # 选择当前模式
  trigger_test: False # 转换异常是否触发程序中断（机器人调试时可设置为 True，脑控时推荐 False）

robot:
	robot_dof: 29 # 机器人型号
	safe_height_threshold: 0.07 # 单位:米，相对于机器人腰部零位高度，用于判定是否启用 SafeArmDown
```

#### 脑控动作域配置

可以修改 `eeg` 动作配置 `state_manager/eeg/eeg_states_config.yaml`
```yaml
behavior_trees: # 动作序列
active_expected_duration: # 为每个动作设置期望持续时间，单位为秒
```

#### 视觉配置
部分动作 `Grasp-Apple`, `Pour-Water` 需要视觉信息，先擦完成 [07 Test Run 示例2 手眼标定](./README_07_test_run_calibrate_zh.md)

修改视觉偏移 `state_manager/eeg/vision_config.yaml`

#### Replay 动作配置
部分动作 `Biubiubiu` 为 `LeRobot 3.0` 上肢遥操数据集回放 

配置回放数据 `state_manager/eeg/datasets_config.yaml`

### 2. Unitree 终端1: 启动主控制节点和灵巧手节点

```sh
# 进入工作空间
cd ~/unitree-g1-brainco-hand/brainco_ws 
# 激活环境
conda activate g1brainco 
# 启动全身运控 (推荐先测试上肢动作，再启动全身运控)
./launch/launch_robot.sh body
```

检查输出信息:
- IK初始化结束 `"IK initialization done."`
- 当显示 `"Request 'configure' to start"` 则可以发送状态转换请求

### 3. Unitree 终端2: 启动 Agent 网络服务

```sh
# 进入工作空间
cd ~/unitree-g1-brainco-hand/brainco_ws 
# 激活 conda 环境
conda activate g1brainco 
# 启动 Agent 服务
./launch/launch_trans.sh agent
```

### 4. 客户端 PC: 打开控制 GUI
```sh
# 进入 client 目录
cd ui_client 
# 激活 conda 环境
conda activate braincogui
# 启动 GUI
python ui_client/ui_client.py
```

<p align="center">
  <img src="images/tutorial_example3.png" width="600">
</p>

通过软件 GUI 管理机器人状态

```
Init -> Ready -> Active 
```

点击GUI 右侧动作按钮，或者通过脑控机器人训练平台发出动作指令
```
(e.g.)a1_Hello -> 手臂放下3s后可执行下个动作
```

通过软件管理结束动作
```
Ready/Ready(Table Safe) 会打断正在执行的动作
```