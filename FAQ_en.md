# FAQ

English Version | [中文版](./FAQ_zh.md)

## 1. Unable to connect to the network after configuring a custom network

```sh
# Remove the default route
sudo ip route del default via 192.168.123.1 dev eth0
```


## 2. 'colcon build' shows system clock warning

```
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
```

```sh
# Check system time
date
```
If it shows 1970, it may be because the system cannot update the clock automatically due to no network connection.

```sh
# Remove default route
sudo ip route del default via 192.168.123.1 dev eth0
```

Close and reopen the terminal.
```sh
# Check time synchronization status
timedatectl status
```

If the time is updated successfully, delete previous build/install/log files and rebuild:
```sh
# Make sure you are in the correct workspace
rm -rf build install log
colcon build
```
The warning should no longer appear.


## 3. not found
```
ros2: command not found
Package 'XXX' not found
```
These issues usually occur when the correct setup files are not sourced.



## 4. General ROS troubleshooting

1. Close all terminals and open a new one.
2. Delete `build`, `install`, `log` folders and rebuild:

## 5. Network configuration issues
```sh
unitree@ubuntu:~$ sudo ifconfig wlan0 up 
# Error: SIOCSIFFLAGS: Operation not possible due to RF-kill
```

Solution:
```sh
# Check RF-kill status
unitree@ubuntu:~$ rfkill list
# 0: phy0: Wireless LAN
#         Soft blocked: yes
#         Hard blocked: no

unitree@ubuntu:~$ sudo rfkill unblock wifi

# 再查看
unitree@ubuntu:~$ rfkill list
# 0: phy0: Wireless LAN
#         Soft blocked: no
#         Hard blocked: no

# 重新运行不再报错
unitree@ubuntu:~$ sudo ifconfig wlan0 up
```

## 6. SSH works but VS Code Remote fails
Make sure the Download Server has network access and the local network is working.

If VS Code repeatedly asks for a password and shows:
```
Failed to connect to the remote extension host server (Error: ExtensionInstallFailed(ExtensionInstallFailed("Error while installing extensions: getaddrinfo EAI_AGAIN marketplace.visualstudio.com\ngetaddrinfo EAI_AGAIN marketplace.visualstudio.com\n")
```
This is a typical network issue where the Linux server cannot access `marketplace.visualstudio.com` due to DNS failure or timeout.

Solution:
1. Connect via SSH using Windows PowerShell or Linux terminal:
```sh
ssh unitree@192.168.xx.xx
```

2. On the Linux server, check network:
```sh
unitree@ubuntu:~$ ping marketplace.visualstudio.com
```
If it fails, the Linux server cannot reach the Internet.

3. Check DNS configuration:
```sh
unitree@ubuntu:~$ cat /etc/resolv.conf
```
If it shows something like: 
```
nameserver 127.0.0.53 
search Huawei.Local
```
 
Ubuntu is using `systemd-resolved` local DNS stub which cannot resolve external domains.

4. Edit `resolved.conf`:
```sh
unitree@ubuntu:~$ sudo vim /etc/systemd/resolved.conf
```

Uncomment or add:
```
[Resolve] DNS=8.8.8.8 1.1.1.1 
FallbackDNS=114.114.114.114 
DNSStubListener=yes
```
Save the file.

5. Restart the DNS service:
```sh
unitree@ubuntu:~$ sudo systemctl restart systemd-resolved
```

6. Verify DNS:
```sh
unitree@ubuntu:~$ systemd-resolve --status
```

Should show:
```
DNS Servers: 8.8.8.8 1.1.1.1
Fallback DNS Servers: 114.114.114.114 8.8.8.8
```
press `q` to exit.

7. Test network connectivity:

```sh
unitree@ubuntu:~$ ping 8.8.8.8
unitree@ubuntu:~$ ping marketplace.visualstudio.com
```
If successful, VS Code Remote SSH should now work properly.


## 7. Revo2Touch hand: `get_touch_sensor_status: deprecated for current firmware`

The `libbc_stark_sdk.so` currently shipped via `download-lib.sh` is version
**0.4.3**, which hard-codes every touch-sensor API as "deprecated for current
firmware" and returns empty regardless of the actual hardware:

```
WARN touch_sensor_setup: deprecated for current firmware
WARN touch_sensor_calibrate: deprecated for current firmware
WARN get_touch_sensor_status: deprecated for current firmware, return empty
WARN stark_get_touch_sensor_raw_data: deprecated for current firmware, return empty
```

This happens even on **Revo2Touch** hardware where
`modbus_get_device_info` reports `hardware_type = 4`
(`STARK_HARDWARE_TYPE_REVO2_TOUCH`) and the physical touch sensors are
installed and alive.

There are two fixes:

**Option A — upgrade the SDK (requires re-download).**
Version **0.8.1** of the SDK (shipped with
[`unitreerobotics/brainco_hand_service`](https://github.com/unitreerobotics/brainco_hand_service))
adds `STARK_HARDWARE_TYPE_REVO2_TOUCH = 4` and accepts the touch-API
calls.  Drop that `libbc_stark_sdk.so` into
`ros2_stark_ws/src/ros2_stark_controller/lib/`, update `stark-sdk.h` to
the matching version, and set `firmware_type: 4` in
`config/params_v2_double.yaml`.

**Option B — skip the SDK and talk raw Modbus.**
A standalone workaround script that uses only `pyserial` and talks the
same Modbus RTU commands the newer SDK sends internally:

```sh
pip install pyserial
python3 ros2_stark_ws/src/ros2_stark_controller/scripts/touch_sensor_pyserial.py --dual
```

It enables the touch sensors, calibrates the idle baseline, and
polls register `4200` at 10 Hz.  Pressing a fingertip shows the per-finger
normal_force going from 0 to ~2500.  Works on any installed SDK version
because it bypasses `libbc_stark_sdk.so` entirely.  See the top of that
file for the register-layout documentation.