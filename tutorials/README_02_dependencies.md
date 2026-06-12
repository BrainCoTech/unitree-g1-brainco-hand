# Install Dependencies

Install Miniconda/Miniforge (`Linux` + `ARM64/aarch64`)
```sh
conda create -n g1brainco python=3.8
conda activate g1brainco
```
## Install Dependencies in Conda
```sh
# For arm motion control
conda install pinocchio -c conda-forge
pip install meshcat transitions

# For building ROS 2 in Conda
pip install rospkg
pip install -U colcon-common-extensions
# Install empy after the two packages above to avoid empy dependency issues
pip install empy==3.3.2 lark-parser

# For vision applications
pip install opencv-python ultralytics mediapipe pyrealsense2

# For trajectory data loading
pip install pyarrow datasets

# Other dependencies
pip install loguru matplotlib
```

## Install CUDA, Torch, Torchversion in Conda

### 1. Install CUDA and Torch
Check the System Jetson version
```sh
dpkg -l | grep nvidia-l4t-core
```

Refer to the [JetPack - Jetson Linux (L4T) Version Mapping](https://jetsonhacks.com/jetpack-and-jetson-linux-l4t-versions/)


| Jetson Version | JetPack Version | Jetson Model |
| ------- | ------- | ------- |
| [L4T 35.3.1](https://developer.nvidia.com/embedded/jetson-linux-r3531) | [5.1.1](https://developer.nvidia.com/embedded/jetpack-sdk-511) | Jetson AGX Orin Series, Jetson Orin NX Series, Jetson Orin Nano Series, Jetson Xavier NX series, Jetson AGX Xavier Series |


Go to [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) and download the PyTorch wheel that matches the JetPack version.

Upload the wheel package to the Unitree robot
```sh
scp torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl unitree@192.168.123.164:/home/unitree/Downloads
```

Install CUDA and PyTorch
```sh
conda activate g1brainco
cd ~/Downloads
pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
```

Verify the installation
```sh
python3 -c "import torch; print(torch.__version__); print(torch.version.cuda); print(torch.cuda.is_available()); print(torch.cuda.device_count())"
```

### 2. Install Torchversion 

| Torch | Torchversion | Repository | Python |
| ------- | ------- | ------- | ------- |
| 2.1.0 | 0.16.0 | [release/0.16](https://github.com/pytorch/vision/tree/release/0.16) | >=3.8, <=3.11 |

Download the matched Torchversion. Upload it to the robot.
```sh
scp -r vision-release-0.16 unitree@192.168.123.164:/home/unitree/Downloads
```

Install
```sh
cd ~/Downloads/vision-release-0.16  
# Activate the Conda environment
conda activate g1brainco 
export BUILD_VERSION=0.16.0
python3 setup.py install 
```
