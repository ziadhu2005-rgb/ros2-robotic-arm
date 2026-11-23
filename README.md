# Robotic Arm — URDF, Simulation and Hardware Control (ROS 2)

A 4-DOF robotic arm modelled in URDF, simulated in Gazebo, visualised in RViz2,
and driven by `trajectory_msgs/msg/JointTrajectory` messages on
`/arm_controller/joint_trajectory`. The same messages are forwarded over USB
serial to a microcontroller that drives real servo motors.

```
slider_gui / trajectory_publisher
        │  /arm_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
        ├──► trajectory_follower ──/arm/jointN/cmd_pos──► ros_gz_bridge ──► Gazebo
        │                                                                     │
        │      RViz2 ◄── robot_state_publisher ◄── /joint_states ◄── ros_gz_bridge
        │
        └──► serial_bridge ──USB serial──► Arduino ──PWM──► servos
```

## Tested environment

| Component | Version |
|---|---|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble |
| Gazebo | Fortress (Gazebo Sim 6.18), via `ros-humble-ros-gz` |
| Microcontroller | Arduino Uno / Nano |

## Repository layout

```
arm_ws/
├── README.md
├── firmware/
│   └── arm_servos/arm_servos.ino      Arduino firmware
└── src/
    ├── arm_description/               URDF, RViz2 config, display launch
    │   ├── urdf/arm.urdf
    │   ├── rviz/arm.rviz
    │   └── launch/display.launch.py
    ├── arm_gazebo/                    Gazebo world, bridge config, main launch
    │   ├── worlds/arm_world.sdf
    │   ├── config/bridge.yaml
    │   └── launch/gazebo.launch.py
    ├── arm_control/                   Python nodes
    │   ├── trajectory_publisher.py    publisher node (demo poses)
    │   ├── trajectory_follower.py     trajectory -> Gazebo joint commands
    │   └── slider_gui.py              slider GUI that publishes trajectories
    └── arm_hardware/
        └── serial_bridge.py           trajectory -> USB serial -> servos
```

## The arm

Kinematic chain: `base_link → joint1 → L1 → joint2 → L2 → joint3 → L3 → joint4 → end_effector`

| Link | Shape | Size |
|---|---|---|
| base_link | box | 10 × 10 × 8 cm (8 cm above the working plane) |
| L1 | cylinder | 12 cm |
| L2 | cylinder | 15 cm |
| L3 | cylinder | 9 cm |
| end_effector | box | 6 cm |

| Joint | Type | Axis | Limits |
|---|---|---|---|
| joint1 | revolute | z (base rotation) | ±180° |
| joint2 | revolute | y (shoulder) | ±90° |
| joint3 | revolute | y (elbow) | ±90° |
| joint4 | revolute | y (wrist) | ±90° |

Every link has `<visual>`, `<collision>` and `<inertial>`. A fixed `world → base_link`
joint anchors the arm to the ground in Gazebo.

## Dependencies

```bash
sudo apt install ros-humble-desktop ros-humble-ros-gz \
                 ros-humble-joint-state-publisher-gui python3-serial python3-tk
```

For the hardware, the serial port needs permission:

```bash
sudo usermod -aG dialout $USER   # then log out and back in
```

## Build

```bash
cd ~/arm_ws
colcon build
source install/setup.bash
```

## Run

**URDF in RViz2 only** (with joint sliders):

```bash
ros2 launch arm_description display.launch.py
```

**Simulation** — Gazebo + RViz2 + slider GUI:

```bash
ros2 launch arm_gazebo gazebo.launch.py
```

Moving a slider publishes to `/arm_controller/joint_trajectory`; the arm moves in
Gazebo and RViz2 at the same time (RViz2 follows `/joint_states` coming back from Gazebo).

**Publisher node** (cycles through demo poses, in a second terminal):

```bash
ros2 run arm_control trajectory_publisher
```

**Single command from the terminal:**

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4],
    points: [{positions: [1.0, 0.5, 0.8, 0.3], time_from_start: {sec: 2}}]}"
```

**Full pipeline** — sliders → Gazebo + RViz2 + real servos:

```bash
ros2 launch arm_gazebo gazebo.launch.py hardware:=true port:=/dev/ttyACM0
```

**Hardware bridge without a board** (prints what it would send):

```bash
ros2 run arm_hardware serial_bridge --ros-args -p dry_run:=true
```

Launch options for `gazebo.launch.py`: `rviz:=false`, `gui:=false`,
`hardware:=true`, `port:=/dev/ttyUSB0`.

## Hardware communication

### Transport

USB serial at **115200 baud** (`pyserial` on the PC, `Serial` on the Arduino).

### Message format

Plain text, one command per line, framed by `<` and `>`:

```
<a1,a2,a3,a4,ms>
```

| Field | Meaning |
|---|---|
| a1..a4 | servo angles in whole degrees (0–180) for joint1..joint4 |
| ms | time in milliseconds the servos take to reach the angles |

Example: `<147,119,136,107,2000>`

The board replies `OK a1,a2,a3,a4` for every valid command, or `ERR ...`.
The `serial_bridge` node prints these replies.

### Angle conversion

Joint angle 0 rad (arm straight up) is servo 90°:

```
servo_deg = clamp(90 + degrees(joint_rad), 0, 180)
```

The bridge sends the last point of each trajectory; the firmware interpolates
from the current angle to the target over `ms`, so the servos move in step with
the simulation.

### Wiring (Arduino Uno / Nano)

| Board pin | Connects to |
|---|---|
| D3 | Servo 1 signal — joint1 (base rotation) |
| D5 | Servo 2 signal — joint2 (shoulder) |
| D6 | Servo 3 signal — joint3 (elbow) |
| D9 | Servo 4 signal — joint4 (wrist) |
| GND | Servo GND and external supply GND (common ground) |

Power the servos from an external 5–6 V supply, not from the Arduino 5 V pin.
At least 2 servos are needed (joint1 and joint2); the others are optional.

### Uploading the firmware

Open `firmware/arm_servos/arm_servos.ino` in the Arduino IDE, select the board and
port, and upload. The board prints `READY` on start-up.
