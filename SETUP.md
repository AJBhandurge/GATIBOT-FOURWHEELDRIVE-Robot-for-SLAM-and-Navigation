# GatiBot / Smorphi — ROS2 Humble Setup Guide

Complete setup guide for the GatiBot four-wheel-drive robot running ROS2 Humble with SLAM, Nav2, and YDLIDAR on Ubuntu 22.04 (Raspberry Pi or NUC).

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Remote Desktop (XRDP)](#remote-desktop-xrdp)
3. [ROS2 Package Dependencies](#ros2-package-dependencies)
4. [Python Dependencies](#python-dependencies)
5. [YDLIDAR SDK](#ydlidar-sdk)
6. [Workspace Setup](#workspace-setup)
7. [Build & Source](#build--source)
8. [udev Rules (Stable Device Symlinks)](#udev-rules-stable-device-symlinks)
9. [Firewall Configuration](#firewall-configuration)
10. [Device Layout Reference](#device-layout-reference)
11. [Troubleshooting](#troubleshooting)

---

## Prerequisites

- Ubuntu 22.04 (Jammy) — bare metal or Raspberry Pi OS with Ubuntu
- ROS2 Humble already installed and sourced (`/opt/ros/humble/setup.bash`)
- Internet access for `apt` and `git`
- User in the `sudo` group

---

## Remote Desktop (XRDP)

Enables Windows Remote Desktop (RDP) access to the robot's desktop over LAN.

```bash
sudo apt update
sudo apt install xrdp -y

# Add xrdp to ssl-cert group (required for TLS)
sudo usermod -a -G ssl-cert xrdp

sudo systemctl restart xrdp
sudo systemctl status xrdp        # Confirm: active (running)
```

> **Connecting:** From Windows, open **Remote Desktop Connection** and enter the robot's IP (visible via `ifconfig`). Port: `3389`.

---

## ROS2 Package Dependencies

Install all required ROS2 Humble packages:

```bash
# URDF / Robot Description
sudo apt install ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui -y

# Geographic & Transform libraries
sudo apt-get install libgeographic-dev -y
sudo apt-get install ros-humble-geographic-msgs -y
sudo apt-get install ros-humble-tf-transformations -y
sudo apt-get install ros-humble-diagnostics -y

# Navigation & SLAM
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-* -y
sudo apt install ros-humble-slam-toolbox -y

# Camera driver
sudo apt install ros-humble-v4l2-camera -y
```

---

## Python Dependencies

```bash
sudo apt install python3-pip -y

# transforms3d — required for tf_transformations
sudo pip3 install transforms3d

# pyserial — for ESP32 serial communication
pip install pyserial
```

> **Note:** Avoid bare `sudo pip3 install` for project packages. Prefer a virtual environment or `--break-system-packages` on Ubuntu 22.04+ if pip complains.

---

## YDLIDAR SDK

The YDLIDAR ROS2 driver depends on the C++ SDK being installed system-wide.

```bash
sudo apt install cmake pkg-config libsystemd-dev git -y

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install

# Verify installation
ls /usr/local/lib/cmake/ydlidar_sdk/
# Expected: YDLidarSDKConfig.cmake  YDLidarSDKConfigVersion.cmake
```

---

## Workspace Setup

```bash
mkdir -p ~/s_ws/src
cd ~/s_ws/src

git clone https://github.com/AJBhandurge/GATIBOT-FOURWHEELDRIVE-Robot-for-SLAM-and-Navigation.git

# Flatten repo contents into src/
cp -r GATIBOT-FOURWHEELDRIVE-Robot-for-SLAM-and-Navigation/* ~/s_ws/src/
rm -rf GATIBOT-FOURWHEELDRIVE-Robot-for-SLAM-and-Navigation/
```

---

## Build & Source

```bash
cd ~/s_ws
colcon build

# Source for current session
source install/setup.bash

# Persist across sessions
echo "source ~/s_ws/install/setup.bash" >> ~/.bashrc
```

> **Tip:** If `colcon build` fails on a single package, use `--packages-select <pkg>` to isolate it. Check `log/latest_build/<pkg>/stdout_stderr.log` for details.

---

## udev Rules (Stable Device Symlinks)

Without udev rules, the LIDAR and ESP32 compete for `/dev/ttyUSB0` and `/dev/ttyUSB1` — their assignment can swap on every reboot or reconnect. These rules bind each device to its **physical USB port** (hub port path), not its enumeration order.

### Write the Rules File

```bash
sudo nano /etc/udev/rules.d/99-smorphi.rules
```

Paste the following (adjust `KERNELS` values if your hub topology differs):

```udev
# LIDAR — USB hub port 1-1.1
SUBSYSTEM=="tty", KERNELS=="1-1.1", SYMLINK+="smorphi_lidar", MODE="0666"

# ESP32 (Mainboard) — USB hub port 1-1.2
SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="smorphi_mb", MODE="0666"
```

### Reload and Trigger

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify symlinks exist
ls -l /dev/smorphi_*
# Expected:
# /dev/smorphi_lidar -> ttyUSB0   (or USB1, depends on boot order)
# /dev/smorphi_mb   -> ttyUSB1
```

---

## Firewall Configuration

Allow RDP access only from devices on the local network (`192.168.1.0/24`):

```bash
sudo ufw allow from 192.168.1.0/24 to any port 3389
sudo ufw reload
sudo ufw status         # Confirm rule is listed
```

> Adjust the subnet (`192.168.1.0/24`) to match your actual LAN if different.

---

## Device Layout Reference

| Symlink | Physical Port | Device | Purpose |
|---|---|---|---|
| `/dev/smorphi_lidar` | USB hub `1-1.1` | YDLIDAR X3 | SLAM / obstacle detection |
| `/dev/smorphi_mb` | USB hub `1-1.2` | ESP32 (CP2102) | Motor control, odometry |

---

## Troubleshooting

**Symlinks not appearing after udev trigger**
```bash
# Check if the rule file has correct syntax
sudo udevadm test $(udevadm info -q path -n /dev/ttyUSB0)
```

**YDLIDAR SDK not found during colcon build**
```bash
# Confirm install
ls /usr/local/lib/cmake/ydlidar_sdk/
# If missing, re-run: sudo make install inside YDLidar-SDK/build/
```

**ESP32 / LIDAR device swap after reboot**
- Ensure both devices are plugged into the **same physical USB ports** every time.
- If using a USB hub, power cycle the hub after plugging in devices before booting.

**xrdp black screen on connect**
```bash
# Check if a .xsession file exists and is correct
echo "startxfce4" > ~/.xsession   # or your DE of choice
sudo systemctl restart xrdp
```

**Nav2 not launching / BT node errors**
```bash
# Ensure workspace is sourced before ROS2 commands
source /opt/ros/humble/setup.bash
source ~/s_ws/install/setup.bash
```

---


*Maintained by Ayush Bhandurge — ayushjbhandurge@gmail.com*  
*Robot: GatiBot / Smorphi | Platform: ROS2 Humble | SVR Robotics Pvt Ltd, Pune*
