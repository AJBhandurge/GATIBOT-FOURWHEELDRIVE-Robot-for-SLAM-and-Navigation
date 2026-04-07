# Four-Wheel Gati Bot — ROS 2 Autonomous Mobile Robot

A **4-wheel Mecanum-drive autonomous mobile robot** built on the Smorphi platform, running **ROS 2 Humble** on a Raspberry Pi 4B. The robot supports manual teleoperation, encoder-based odometry, SLAM mapping, AMCL localisation, Nav2 autonomous navigation, and custom frontier-based autonomous exploration.

---

## System Overview

```
ESP32 (Motor Shield + Encoders)
        │  Serial 115200 baud
        │  TX: "c1,c2,c3,c4\n"   (encoder ticks at 50 Hz)
        │  RX: "d1,d2,d3,d4,p1,p2,p3,p4\n"  (direction + PWM)
        ▼
Raspberry Pi 4B  ──  ROS 2 Humble
  ├── smorphi_driver       → /odom, TF: odom→base_footprint
  ├── robot_state_publisher → TF: base_footprint→base_link→laser_link
  ├── ydlidar_ros2_driver  → /scan
  ├── slam_toolbox         → /map  (SLAM mode)
  ├── nav2_amcl            → localisation (navigation mode)
  ├── Nav2 stack           → /cmd_vel  (planner + controller)
  └── ExplorerNode         → NavigateToPose goals (autonomous exploration)
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Compute | Raspberry Pi 4B |
| Microcontroller | ESP32 with Adafruit Motor Shield |
| Drivetrain | 4× DC motors with Mecanum wheels |
| Encoders | Quadrature encoders, RISING-edge ISR (single channel) |
| LiDAR | YDLidar (connected to RPi via USB) |
| Chassis | Smorphi 4-wheel platform — 170×170×50 mm body |
| Wheel diameter | 60 mm (radius 30 mm) |
| Encoder ticks/rev | 585 (45:1 gear ratio × 13 encoder counts) |

---

## Workspace Structure

```
s_ws/src/
├── model_description/               # URDF / xacro robot model
│   └── urdf/smorphi.urdf.xacro      # 4-wheel Mecanum URDF with laser_link
│
├── smorphi_node/                    # Hardware driver (ROS 2 Python package)
│   └── smorphi_node/
│       ├── smorphi_driver.py        # PRIMARY DRIVER — Mecanum IK/FK + odometry
│       ├── motor2.py                # Alternate driver (Mecanum, earlier version)
│       ├── motor3.py                # Alternate driver (differential drive mode)
│       └── tf.py                   # Static TF publisher utility
│
├── smorphi_bringup/                 # Launch files and Nav2 configuration
│   ├── launch/
│   │   ├── smorphi.launch.py               # Robot bringup + YDLidar + RViz
│   │   ├── navigation_launch.py            # Nav2 with AMCL (pre-built map)
│   │   ├── navigation_exploration.launch.py # Nav2 + SLAM (live mapping)
│   │   ├── online_async_launch.py          # SLAM Toolbox only
│   │   └── localization_launch.py          # AMCL localisation only
│   ├── config/
│   │   ├── nav2_params.yaml                # Nav2 params (DWB, AMCL) — RPi4 tuned
│   │   ├── nav2_params2.yaml               # Nav2 params for exploration mode
│   │   ├── mapper_params_online_sync.yaml  # SLAM Toolbox online sync config
│   │   └── smorphi_mapper_online_async.yaml
│   └── maps/
│       └── svr1.yaml / svr1.pgm            # Pre-built map (server room)
│
├── smorphi_ros2_launchers/          # Original Smorphi SDK launchers (reference)
│   └── launch/
│       ├── smorphi_nav2.launch.py
│       └── smorphi_bringup.launch.py
│
├── Autonomous-Explorer-and-Mapper-ros2-nav2/  # Frontier-based autonomous explorer
│   └── custom_explorer/
│       └── explorer.py              # Frontier detection + Nav2 goal sender
│
└── ydlidar_ros2_driver/             # YDLidar ROS 2 driver
    └── launch/ydlidar_launch.py
```

---

## Robot Model (URDF)

**TF Tree:**
```
base_footprint  (ground-plane reference, z=0)
  └── base_link  (robot body centre, z=+0.06 m)
        ├── front_left_wheel   (+0.035, +0.065, -0.06)
        ├── front_right_wheel  (+0.035, -0.065, -0.06)
        ├── rear_left_wheel    (-0.035, +0.065, -0.06)
        ├── rear_right_wheel   (-0.035, -0.065, -0.06)
        └── laser_link         (0, 0, +0.05)
```

**Physical dimensions:**

| Parameter | Value |
|-----------|-------|
| Body | 170 × 170 × 50 mm |
| Wheel radius | 30 mm |
| Wheel offset X (Lx) | 35 mm |
| Wheel offset Y (Ly) | 65 mm |
| Wheel distance (wd = Lx+Ly) | 100 mm |
| Robot mass | 2.0 kg |
| Wheel mass | 0.15 kg each |

---

## Mecanum Drive Kinematics

**Wheel layout (top view):**
```
M1(FL) ╲  front  ╱ M2(FR)
M4(RL) ╱  rear   ╲ M3(RR)
```

**Forward Kinematics (encoders → odometry):**
```
Δx   = ( FL + FR + RR + RL) / 4
Δy   = (-FL + FR - RR + RL) / 4
Δyaw = (-FL + FR + RR - RL) / (4 × wheel_dist)
```

**Inverse Kinematics (cmd_vel → wheel speeds):**
```
ω_FL = ( vx - vy - wz×wd) / r
ω_FR = ( vx + vy + wz×wd) / r
ω_RR = ( vx - vy + wz×wd) / r
ω_RL = ( vx + vy - wz×wd) / r
```

Pose integration uses **mid-point RK2** for improved accuracy at higher speeds.

---

## Serial Protocol (ESP32 ↔ Raspberry Pi)

**ESP32 → RPi (50 Hz, every 20 ms):**
```
c1,c2,c3,c4\n
```
Signed cumulative encoder tick counts: `c1=FL, c2=FR, c3=RR, c4=RL`

**RPi → ESP32:**
```
d1,d2,d3,d4,p1,p2,p3,p4\n
```
`d` = direction per wheel: `0=STOP, 1=FORWARD, 2=BACKWARD`
`p` = PWM per wheel: `0–255`

---

## Operating Modes

### Mode 1 — SLAM Mapping (build a new map)

```bash
# Terminal 1 — Robot bringup
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup smorphi.launch.py

# Terminal 2 — Driver
source ~/s_ws/install/setup.bash
ros2 run smorphi_node smorphi_node.py

# Terminal 3 — SLAM + Nav2 (exploration mode)
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup navigation_exploration.launch.py

# Terminal 4 — Autonomous explorer (frontier-based)
source ~/s_ws/install/setup.bash
ros2 run custom_explorer explorer
```

Save the map when done:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/s_ws/src/smorphi_bringup/maps/my_map
```

---

### Mode 2 — Navigation with Pre-built Map (AMCL localisation)

```bash
# Terminal 1 — Robot bringup
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup smorphi.launch.py

# Terminal 2 — Driver
source ~/s_ws/install/setup.bash
ros2 run smorphi_node smorphi_node.py

# Terminal 3 — Nav2 with saved map
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup navigation_launch.py \
  map:=~/s_ws/src/smorphi_bringup/maps/svr1.yaml

# Use RViz "2D Pose Estimate" to initialise AMCL, then send Nav2 goals
```

Custom map:
```bash
ros2 launch smorphi_bringup navigation_launch.py \
  map:=/path/to/your_map.yaml
```

---

### Mode 3 — Manual Teleoperation

```bash
# Terminal 1 — Robot bringup + driver
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup smorphi.launch.py
ros2 run smorphi_node smorphi_node.py

# Terminal 2 — Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### Mode 4 — SLAM Only (no Nav2)

```bash
source ~/s_ws/install/setup.bash
ros2 launch smorphi_bringup online_async_launch.py
```

---

## ROS Parameters (smorphi_driver)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/smorphi_mb` | Serial device for ESP32 |
| `baud_rate` | `115200` | Serial baud rate |
| `wheel_radius` | `0.03` m | Wheel radius (matches URDF) |
| `ticks_per_rev` | `585.0` | Encoder ticks per revolution |
| `wheel_dist` | `0.10` m | Lx + Ly for mecanum kinematics |
| `max_wheel_vel` | `6.0` rad/s | Wheel speed mapped to PWM 255 |
| `min_motor_pwm` | `70` | Minimum PWM to overcome friction |
| `cmd_threshold` | `0.001` | Stop threshold for cmd_vel |
| `publish_tf` | `true` | Broadcast odom→base_footprint TF |
| `debug_cmd_vel` | `false` | Print wheel PWMs to terminal |

Override at runtime:
```bash
ros2 run smorphi_node smorphi_node.py --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p debug_cmd_vel:=true \
  -p min_motor_pwm:=80
```

---

## Calibration

**Distance calibration (ticks_per_rev):**
```
1. Drive the robot forward exactly 1 metre.
2. Check: ros2 topic echo /odom  (read pose.pose.position.x)
3. new_ticks_per_rev = 585 × (odom_reading / 1.0)
```

**Rotation calibration (wheel_dist):**
```
1. Rotate the robot exactly 360°.
2. Check: ros2 topic echo /odom  (read orientation yaw in degrees)
3. new_wheel_dist = 0.10 × (odom_degrees / 360)
```

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Publish | Wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | Publish | YDLidar scan |
| `/map` | `nav_msgs/OccupancyGrid` | Publish | SLAM map |
| `/tf` | TF | Publish | odom→base_footprint |

---

## Available Maps

| Map | File | Resolution | Description |
|-----|------|------------|-------------|
| Server room | `smorphi_bringup/maps/svr1.yaml` | 0.05 m/px | Indoor server room environment |
| Passage | `s_ws/passage.yaml` | 0.05 m/px | Corridor / passage environment |

---

## Autonomous Explorer

The `ExplorerNode` (`explorer.py`) implements greedy frontier-based exploration:

1. Subscribes to `/map` (OccupancyGrid)
2. Every 5 seconds, scans the map for **frontier cells** — free cells adjacent to unknown cells
3. Selects the closest unvisited frontier
4. Converts grid coordinates to world coordinates and sends a `NavigateToPose` action goal to Nav2
5. Tracks visited frontiers to avoid revisiting
6. Stops when no frontiers remain (map fully explored)

---

## Build & Install

```bash
cd ~/s_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Serial Port Setup

Create a persistent udev rule for the ESP32:
```bash
# Find the device ID
udevadm info -a /dev/ttyUSB0 | grep idVendor

# Create rule
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="smorphi_mb"' \
  | sudo tee /etc/udev/rules.d/99-smorphi.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## ESP32 Firmware

Flash `smorphi_encoder_ros4/smorphi_encoder_ros4.ino` using Arduino IDE with:
- Board: **ESP32 Dev Module**
- Libraries: **Adafruit Motor Shield V2** (`Adafruit_MotorShield.h`)
- Baud rate: **115200**

**Encoder pin mapping:**

| Motor | Channel A | Channel B |
|-------|-----------|-----------|
| M1 FL | GPIO 4 | GPIO 5 |
| M2 FR | GPIO 19 | GPIO 18 |
| M3 RR | GPIO 25 | GPIO 23 |
| M4 RL | GPIO 26 | GPIO 27 |

---

## Dependencies

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-keyboard

pip3 install pyserial numpy
```

---

## Maintainer
email: ayushjbhandurge@gmail.com
