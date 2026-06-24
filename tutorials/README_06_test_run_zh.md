## 示例1：简单动作

### 1. 机器人配置
修改通用配置 `brainco_ws/src/control_py/config/smach_config.yaml`
```yaml
mode:
	current_mode: simple # 选择预定义的模式
```

修改模式配置 `brainco_ws/src/control_py/control_py/state_manager/<mode>/<mode>_states_config.yaml`
```yaml
behavior_trees:
	active: # 动作序列
	
active_expected_duration: # 每个动作的执行时间，在此期间不会被其他动作打断
```
### 2. Unitree 终端1: 启动主控制节点和灵巧手节点

```sh
# 进入工作空间
cd ~/unitree-g1-brainco-hand/brainco_ws 
# 激活环境
conda activate g1brainco 

# (可选)启动上肢运控
./launch/launch_robot.sh
# (可选)单独启动手臂运控
./launch/launch_robot.sh arm
# (可选)启动全身运控
# 需使用遥控手柄让机器人进入"常规运控模式(R1 + X)"
./launch/launch_robot.sh body
```

检查输出信息:

| node | info | |
| --------- | --------------------- | ----------------- |
| stark_node | slave[126] get_device_info... DeviceInfo: ...<br>slave[127] get_device_info... DeviceInfo: ... | 双手通信正常 |
| high_cmd_node | G1 HighCmd service ready | 下肢运控 bridge 已准备 |
| smach_main_node | Importing ultralytics YOLO. Please wait... | 等待 YOLO 模型加载完成（如果启用）|
| smach_main_node | Mode: eeg 检查正确 | 选择的控制模式 |
| smach_main_node | IK initialization done. | 加载 IK |
| smach_main_node | Request 'configure' to start | 等待 agent 控制命令 |


### 3. Unitree 终端2: 启动 Agent 网络服务

```sh
# 进入工作空间
cd ~/unitree-g1-brainco-hand/brainco_ws 
# 激活conda环境
conda activate g1brainco 
# 启动 Agent 服务
./launch/launch_trans.sh agent
```

检查输出信息:

| node   | info                                          |              |
| ------ | --------------------------------------------- | ------------ |
| smach_trans_node | All ROS2 services ready | 检测 ROS2 服务已启动     |
| server | UDP discovery listening on ('0.0.0.0', 37020) | 等局域网广播发现     |
| agent  | TCP listening on ('0.0.0.0', 43210)           | 状态机命令端口      |
| agent  | TCP listening on ('0.0.0.0', 46000)           | 动作/切换命令端口    |
| smach_trans_node  | Agents started | 网络代理服务已经启动成功 |


### 4. 控制端 PC: 打开控制 GUI
```sh
# 进入 client 目录
cd ui_client 
# 激活conda环境
conda activate braincogui
# 启动 GUI
python ui_client/ui_client.py
```

<p align="center">
  <img src="images/tutorial_example1.png" width="600">
</p>

执行动作

```
Init -> Ready -> Active -> (e.g.)a1_Hello -> 手臂放下3s后可执行下个动作
```

结束动作
```
(e.g.)a1_Hello -> Ready/Ready(Table Safe) 会打断正在执行的动作
```


| UI config | description           | default           |
| --------- | --------------------- | ----------------- |
| 左上角IP     | 同网络下的机器人IP            | `192.168.123.164` |
| Port1     | 内部状态机命令，不可被发现         | `43210`           |
| Port2     | Action切换命令，可以通过广播自动发现 | `46000`           |

| State              |          |                  |
| ------------------ | -------- | ---------------- |
| Init               | 初始状态     |                  |
| Ready              | 机器人准备/回零 | 手臂从当前位置径直恢复原位    |
| Ready (Table Safe) | 机器人准备/回零 | 手臂绕过桌子恢复原位       |
| Shutdown           | 机器人急停    | 重启需在机器人端重启两个终端命令 |

**注意：**
- 激活命令显示黄色，可达命令显示白色
- 左侧状态机不影响下肢 HighCmd
- `Active` 期间，正在执行的动作在一定时间(可配置)内不会被其他 `Action` 打断，但可通过 `Ready` 回零，或通过`Shutdown` 急停

