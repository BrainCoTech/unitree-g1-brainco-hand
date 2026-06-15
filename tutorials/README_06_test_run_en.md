## Test Run

English Version | [中文版](./README_06_test_run_zh.md)

### 1. Robot Configuration
Modify the general configuration in `brainco_ws/src/control_py/config/smach_config.yaml`:

```yaml
mode:
	current_mode: simple # Select a predefined mode
```

Modify the mode configuration in `brainco_ws/src/control_py/control_py/state_manager/<mode>/<mode>_states_config.yaml`:

```yaml
behavior_trees:
	active: # Action sequence
	
active_expected_duration: # Execution time of each action; actions will not be interrupted during this period
```

### 2. Unitree Terminal 1: Start the Main Control Node and Dexterous Hand Node

```sh
# Enter the workspace
cd ~/unitree-g1-brainco-hand/brainco_ws
# Activate the environment
conda activate g1brainco

# (Optional) Start upper-limb control
./launch/launch_robot.sh
# (Optional) Start arm-only control
./launch/launch_robot.sh arm
# (Optional) Start whole-body control
# The robot must be switched to "Normal Motion Control Mode (R1 + X)" using the remote controller
./launch/launch_robot.sh body
```

Check the output:
- The left and right hand `Port`, `Baudrate`, and `slave_id` are all correct.
- The serial port is open: `"serial port opened"`
- Waiting for joint control commands: `"Waiting for joint cmd ..."`

If the messages above do not appear, the config file parameters may be incorrect.

- IK initialization is complete: `"IK initialization done."`
- When `"Request 'configure' to start"` is displayed, you can send state transition requests.

### 3. Unitree Terminal 2: Start the Agent Network Service

```sh
# Enter the workspace
cd ~/unitree-g1-brainco-hand/brainco_ws
# Activate the conda environment
conda activate g1brainco
# Start the Agent service
./launch/launch_trans.sh agent
```

### 4. Client PC: Open the Control GUI

```sh
# Enter the client directory
cd ui_client
# Activate the conda environment
conda activate braincogui
# Start the GUI
python ui_client/ui_client.py
```
