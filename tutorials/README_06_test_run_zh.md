## 测试运行

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
- 左右手 `Port`，`Baudrate`，`slave_id` 都正确
- 串口已打开 `"serial port opened"`
- 正在等待关节控制命令 `"Waiting for joint cmd ..."`
如果未出现上述信息，可能是config文件参数不正确
- IK初始化结束 `"IK initialization done."`
- 当显示 `"Request 'configure' to start"` 则可以发送状态转换请求

### 3. Unitree 终端2: 启动 Agent 网络服务

```sh
# 进入工作空间
cd ~/unitree-g1-brainco-hand/brainco_ws 
# 激活conda环境
conda activate g1brainco 
# 启动 Agent 服务
./launch/launch_trans.sh agent
```

### 4. 客户端 PC: 打开控制 GUI
```sh
# 进入 client 目录
cd ui_client 
# 激活conda环境
conda activate braincogui
# 启动 GUI
python ui_client/ui_client.py
```
