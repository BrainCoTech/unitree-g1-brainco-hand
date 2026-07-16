## Example 3: Brain-Controlled Robot Task

English Version | [中文版](./README_08_test_run_eeg_zh.md)

### 1. Robot Configuration

#### General Configuration
Modify the general configuration in `config/smach_config.yaml`:

```yaml
mode:
  current_mode: eeg # Select the current mode
  trigger_test: False # Whether abnormal transitions interrupt the program. This can be set to True during robot debugging; False is recommended for brain control.

robot:
	robot_dof: 29 # Robot model
	safe_height_threshold: 0.07 # Unit: meters. Relative to the robot waist zero height, used to determine whether SafeArmDown should be enabled.
```

#### Brain-Controlled Action Domain Configuration

You can modify the `eeg` action configuration in `state_manager/eeg/eeg_states_config.yaml`:

```yaml
behavior_trees: # Action sequences
active_expected_duration: # Expected duration of each action, in seconds
```

#### Vision Configuration
Some actions, such as `Grasp-Apple` and `Pour-Water`, require vision information. Complete [07 Example 2: Hand-Eye Calibration](./README_07_test_run_calibrate_en.md) first.

Modify the vision offset in `state_manager/eeg/vision_config.yaml`.

#### Replay Action Configuration
Some actions, such as `Biubiubiu`, are replayed from `LeRobot 3.0` upper-limb teleoperation datasets.

Configure replay data in `state_manager/eeg/datasets_config.yaml`.

### 2. Unitree Terminal 1: Start the Main Control Node and Dexterous Hand Node

```sh
# Enter the workspace
cd ~/unitree-g1-brainco-hand/brainco_ws
# Activate the environment
conda activate g1brainco
# Start whole-body control. Testing upper-limb motions first is recommended before starting whole-body control.
./launch/launch_robot.sh body
```

Check the output:
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

<p align="center">
  <img src="images/tutorial_example3.png" width="600">
</p>

Manage the robot state through the GUI:

```
Init -> Ready -> Active
```

Click an action button on the right side of the GUI, or send an action command from the BrainCo brain-controlled robot training platform:
```
(e.g.)a1_Hello -> After the arm lowers for 3 seconds, the next action can be executed
```

Stop an action through the GUI:
```
Ready/Ready(Table Safe) will interrupt the action currently being executed
```
