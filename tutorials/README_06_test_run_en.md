## Example 1: Simple Motions

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

| node | info |  |
| ---- | ---- | ---- |
| `stark_node` | `slave[126] get_device_info... DeviceInfo: ...`<br>`slave[127] get_device_info... DeviceInfo: ...` | Both hands are communicating normally |
| `high_cmd_node` | `G1 HighCmd service ready` | The lower-limb control bridge is ready |
| `smach_main_node` | `Importing ultralytics YOLO. Please wait...` | Wait for YOLO model loading to complete, if enabled |
| `smach_main_node` | `Mode: eeg` Confirm it is correct | The selected control mode |
| `smach_main_node` | `IK initialization done.` | IK loaded |
| `smach_main_node` | `Request 'configure' to start` | Waiting for Agent control commands |

### 3. Unitree Terminal 2: Start the Agent Network Service

```sh
# Enter the workspace
cd ~/unitree-g1-brainco-hand/brainco_ws
# Activate the conda environment
conda activate g1brainco
# Start the Agent service
./launch/launch_trans.sh agent
```

Check the output:

| node | info |  |
| ---- | ---- | ---- |
| `smach_trans_node` | `All ROS2 services ready` | Confirms the ROS2 services have started |
| `server` | `UDP discovery listening on ('0.0.0.0', 37020)` | Waiting for LAN broadcast discovery |
| `agent` | `TCP listening on ('0.0.0.0', 43210)` | State machine command port |
| `agent` | `TCP listening on ('0.0.0.0', 46000)` | Action/switching command port |
| `smach_trans_node` | `Agents started` | The network proxy service started successfully |

### 4. Host PC: Open the Control GUI

```sh
# Enter the client directory
cd ui_client
# Activate the conda environment
conda activate braincogui
# Start the GUI
python ui_client/ui_client.py
```

<p align="center">
  <img src="images/tutorial_example1.png" width="600">
</p>

Run an action:

```
Init -> Ready -> Active -> (e.g.)a1_Hello -> After the arm lowers for 3 seconds, the next action can be executed
```

Stop an action:
```
(e.g.)a1_Hello -> Ready/Ready(Table Safe) will interrupt the action currently being executed
```

| UI config | description | default |
| --------- | ----------- | ------- |
| IP (top left) | Robot IP on the same network | `192.168.123.164` |
| Port1 | Internal state machine commands, not discoverable | `43210` |
| Port2 | Action switching commands, can be discovered automatically by broadcast | `46000` |

| State |  |  |
| ----- | -------- | ---------------- |
| Init | Initial state |  |
| Ready | Robot ready / back to zero | The arm returns directly to the initial pose from the current position |
| Ready (Table Safe) | Robot ready / back to zero | The arm returns to the initial pose while avoiding the table |
| Shutdown | Emergency stop | Restarting requires rerunning the two terminal commands on the robot side |

**Note:**
- The active command is shown in yellow, and reachable commands are shown in white.
- The state machine on the left does not affect lower-limb `HighCmd`.
- During `Active`, the action currently being executed cannot be interrupted by other `Action` commands for a certain period of time (configurable), but it can still be interrupted by `Ready` to return to zero, or by `Shutdown` for an emergency stop.
