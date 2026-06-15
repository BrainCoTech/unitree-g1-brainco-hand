## Example 2: Hand-Eye Calibration

English Version | [中文版](./README_07_test_run_calibrate_zh.md)

### 1. Robot Configuration
Modify the general configuration in `brainco_ws/src/control_py/config/smach_config.yaml`:

```yaml
mode:
	current_mode: calibr # Select hand-eye calibration mode
```

Check the calibration configuration in `brainco_ws/src/control_py/control_py/state_manager/calibrate/calibrate_config.yaml`:

```yaml
paths:
  cam_calibr_dir: /home/unitree/unitree-g1-brainco-hand/cam_calibr

collection:
  move_range: [0.015, 0.015, 0.015, 0.15, 0.15, 0.15]
```

- `cam_calibr_dir`: Directory for saving calibration data
- `move_range`: Random end-effector motion range during calibration data collection

### 2. Unitree Terminal 1: Start the Main Control Node and Dexterous Hand Node

```sh
# Enter the workspace
cd ~/unitree-g1-brainco-hand/brainco_ws
# Activate the environment
conda activate g1brainco
# (Recommended) Start arm-only control
./launch/launch_robot.sh arm
```

Check the output:
- IK initialization is complete: `"IK initialization done."`
- When `"Request 'configure' to start"` is displayed, you can send state transition requests

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

### 5. Collect Hand-Eye Calibration Data
Prepare a chessboard calibration board first. We provide a 3D-printable [calibration board](./tutorials/cad/calibration_27x27.pdf) and a [calibration board bracket](./tutorials/cad/calibration_bracket.STEP) with the same wrist interface as the Revo2 hand. After printing, you can directly replace the Revo2 hands with them for calibration.

<p align="center">
  <img src="images/tutorial_calibrate.jpg" width="300">
</p>

Run calibration for the left and right arms separately:
```
Init -> Ready -> Active -> Calibr-Left-Start -> Calibr-Right-Start -> Ready
```

During collection, the program automatically moves the robot arm randomly and saves the data. By default, `20` samples are collected for each side.

Generated data files:
- Images: `cam_calibr/imgs/left/`, `cam_calibr/imgs/right/`
- Robot arm poses: `cam_calibr/arm_pose/left.json`, `cam_calibr/arm_pose/right.json`

### 6. Compute Calibration Results Offline

After collecting data for both left and right arms, check the images:

```sh
cd ~/unitree-g1-brainco-hand/cam_calibr
chmod +x ./imgs/check_img_grid.sh
./imgs/check_img_grid.sh left
./imgs/check_img_grid.sh right
```

Compute the calibration parameters offline:

```sh
cd ~/unitree-g1-brainco-hand/cam_calibr
python compute_calibration.py
```

After the script finishes, it will generate:
- `cam_calibr/settings.yaml`

This file contains the camera extrinsics for the left and right hands. The `calibrate` mode reads this file later for coordinate conversion.
