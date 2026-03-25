
# Introduction

English Version | [中文版](./README.md)

Extensions of [unitreerobotics/brainco_hand_service]((https://github.com/unitreerobotics/brainco_hand_service))

## Setup
Clone or upload [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) and [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service) to the robot development unit.

```sh
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/brainco_hand_service.git
```

Create new example
```sh
cd ~/unitree_sdk2/example/g1
mkdir brainco_hand & cd brainco_hand
touch g1_arm7_brainco_example.cpp
```

Copy and paste the content of `brainco_hand_service_extension/g1_arm7_brainco_example.cpp`


Edit `CMakeLists.txt` 

1. `unitree_sdk2/example/g1/CMakeLists.txt`

    ```cmake
    # Add at the very beginning
    add_executable(g1_arm7_brainco_example brainco_hand/g1_arm7_brainco_example.cpp)
    target_link_libraries(g1_arm7_brainco_example unitree_sdk2)
    ```

2.  Replace `unitree_sdk2/CMakeLists.txt` with `brainco_hand_service_extension/CMakeLists.txt`.

## Build
Ensure the system time is correct.
```sh
# show current time
date
```

If the system time has not been updated.
```sh
# remove the default route
sudo ip route del default via 192.168.123.1 dev eth0
# restart the system time sync
sudo systemctl restart systemd-timesyncd
# show current time
date
```

build `brainco_hand_service`
```sh
cd ~/brainco_hand_service
mkdir build && cd build
cmake ..
make -j6
```

build `unitree_sdk2`
```sh
cd ~/unitree_sdk2
mkdir build & cd build
cmake ..
sudo make install
```

## Run

### Terminal 1 stay running
```sh
cd ~/brainco_hand_service/bin
# start server (eth0 or wlan0)
sudo ./brainco_hand_server --network eth0
```

### Terminal 2
#### Hand test
```sh
cd ~/brainco_hand_service/bin
# test left hand
sudo ./test_brainco_hand_server left
# test right hand
sudo ./test_brainco_hand_server right
```

#### Arm and hand test
```sh
cd ~/unitree_sdk2/build/bin
# test arm and hand (whether wired or wireless, eth0 must be used here.)
./g1_arm7_brainco_sdk_dds_example eth0
```
