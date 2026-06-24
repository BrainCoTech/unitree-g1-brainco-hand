# FAQ

English Version | [中文版](./FAQ_zh.md)

## System

### Q1. Unable to connect to the network after configuring a custom network

```sh
# Remove the default route
sudo ip route del default via 192.168.123.1 dev eth0
```


### Q2. `colcon build` shows a system clock warning

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


### Q3. not found
```
ros2: command not found
Package 'XXX' not found
```
These issues usually occur when the correct setup files are not sourced.



### Q4. For any ROS issue, you can try:

1. Close all terminals and open a new one.
2. Delete `build`, `install`, and `log`, then rebuild.

### Q5. Network configuration
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

# Check again
unitree@ubuntu:~$ rfkill list
# 0: phy0: Wireless LAN
#         Soft blocked: no
#         Hard blocked: no

# Run again; the error should be gone
unitree@ubuntu:~$ sudo ifconfig wlan0 up
```

### Q6. SSH works, but VS Code remote connection fails
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

## Control

### Q1. The client cannot discover the robot device

Check these first:
- Confirm `./launch_robot.sh` and `./launch_trans.sh agent` have both been started and their outputs look normal.
- Confirm the robot and the control PC are on the same network.
- Confirm ports `46000` and `37020` are not already in use.

### Q2. Newly added actions do not appear in the UI

Check these first:
- Confirm the project has been rebuilt.
- Confirm the current mode is correct in `src/control_py/config/smach_config.yaml`.
- Confirm the new action has been added to `active.children`.
- Confirm that action ID has not been added to `hidden_action_reg`.
- Confirm you restarted the Agent and rescanned the device.

### Q3. An action can be triggered, but it cannot switch to the next step

Check these first:
- Confirm `active_expected_duration` is not set too long.
- Confirm `allowed_sub_trans` is not restricting the next transition.
- For actions using `auto`, confirm `mark_active_action_done()` is called correctly.

### Q4. Why does grasping always use the right hand?

In the current code, some grasping entry points and intermediate steps are hard-coded to `right`.
If you want to support the left hand or both hands, you need to update the handling logic in the relevant `on_xx_handler` functions as well.

### Q5. Why does the action not start after an object is recognized?

Common causes:
- The object is not recognized stably enough. There is currently a stability-check mechanism, so hand motion can cause an object being held to fail the stability check.
- The detected object bounding box is too close to the edge of the camera view.
