# UI

Two packages provide the human interface to SELQIE Lite 2:

- **`selqie_python`** — `selqie.py`, a ROS2 node that wraps all robot subsystems in a single Python class.
- **`selqie_ui`** — `selqie_terminal.py`, an interactive `cmd`-based CLI that drives the `selqie.py` node.

---

## Package Layout

```
ui/
├── selqie_python/
│   └── selqie_python/
│       └── selqie.py           # Core SELQIE ROS2 node / API class
├── selqie_ui/
│   └── selqie_ui/
│       ├── selqie_terminal.py  # Interactive terminal
│       └── selqie_joint_publisher.py
├── selqie_tools/
│   └── selqie_tools/
│       └── interactive_rgb_viewer.py   # LED color picker utility
└── ui_bringup/
    └── launch/
        ├── urdf.launch.py      # Robot description publisher
        └── rviz.launch.py      # RViz2 visualization
```

---

## Running the Terminal

```bash
ros2 run selqie_ui selqie_terminal
```

The tmux session (`tmux/selqie.sh`) starts this automatically in the top-right pane.

---

## selqie_terminal Commands

Type `help` at the prompt to list all commands. Type `help <command>` for details. This table is generated directly from `selqie_ui/selqie_ui/selqie_terminal.py` — a fuller version, including which launch file each command needs, is in `docs/SELQIE_LITE_2_SOP.md` Sections 9–10.

### Motor & Leg Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `idle` | — | Send `"exit"` to all 8 motors (disable) |
| `ready` | — | Send `"start"` to all 8 motors (enable, closed-loop MIT mode) |
| `zero` | — | Call `set_motor_position_zero` on all motors (sets current position as encoder zero — does not move the motor) |
| `clear_errors` | — | Send `"clear"` to all motors |
| `set_motor_position` | `<motor_id 0-7> <pos_rad>` | Command one motor to a position |
| `set_gains` | `<kp> <kd> [velocity_kd]` | Set MIT gains on **all** motors, live — takes effect on the next command, no node restart. `kp` 0–500 (stiffness), `kd` 0–5 (damping). |
| `set_motor_gains` | `<motor> <kp> <kd> [velocity_kd]` | Same, for a single motor. |
| `gains` | — | Print the gains every motor is currently using. |
| `default` | — | Move all 4 legs to the default stance position (`0, 0, -0.18914`) |
| `set_leg_position` | `<leg\|*> <x> <y> <z>` | Move one leg (`FL`/`RL`/`RR`/`FR`) or all (`*`) to a Cartesian foot position (m) |
| `set_leg_force` | `<leg\|*> <x> <y> <z>` | Command one leg or all to a Cartesian foot force (N) |
| `print_motor_info` | — | Print live position/velocity/torque/current/temperature/error for every motor |
| `print_leg_info` | — | Print live position/velocity/force estimate for every leg |
| `print_errors` | — | List any active motor fault by name |

### Feed-Forward Trajectory Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `run_trajectory` | `<file> <loops> <hz> [<file2> <loops2> <hz2> ...]` | Play one or more trajectory files open-loop. Tab-completes filenames from `leg_trajectory_publisher/trajectories/`. Each file is resampled to a constant setpoint rate (`TRAJECTORY_RESAMPLE_HZ`, default 500 Hz) regardless of `<hz>`, so the delivered rate stays within the CAN/motor-node budget instead of scaling with frequency — see [Actuation](../actuation/README.md#run_trajectory-and-setpoint-resampling). |
| `run_trajectory_record` | same as above | Same as `run_trajectory`, but automatically starts a rosbag before the run and stops it after. Refuses to start if already recording. |

### Closed-Loop Gait Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `set_gait` | `<walk\|swim\|jump\|sink\|stand\|none>` | Select the active gait mode |
| `cmd_vel` | `<lin_x> <lin_z> <ang_z>` | Send a velocity command to the active gait |
| `walk` | `<lin_x> <ang_z>` | Shortcut: set gait to walk, then drive |
| `swim` | `<lin_x> <lin_z>` | Shortcut: set gait to swim, then drive |
| `jump` | `<lin_x> <lin_z>` | Shortcut: set gait to jump, then drive |
| `stand` | — | Shortcut: stand in place |
| `sink` | — | Shortcut: controlled sink, zero velocity |
| `set_goal` | `<x> <y> <theta>` | Publish an autonomous goal pose (only meaningful if the planning stack is separately launched) |

### Sensor / Status Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `battery` | — | Print current battery voltage (requires `sensing.launch.py`) |
| `calibrate_imu` | — | Trigger IMU calibration (requires IMU hardware + `imu.launch.py` enabled) |
| `reset_localization` | — | Reset the pose estimate to the origin (requires `localization.launch.py`) |
| `reset_map` | — | Clear the terrain map (requires `mapping.launch.py`) |

### Recording Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `start_recording` / `stop_recording` | — | Start/stop a rosbag of key topics → `/home/selqie/rosbags/<timestamp>` |

### LED / Servo / Light Commands

| Command | Arguments | Description |
|---------|-----------|-------------|
| `set_led_color` | `<r> <g> <b>` | Set WS2812B color (0–255 each channel; requires `sensing.launch.py`) |
| `led_off` | — | Turn LED off (equivalent to `set_led_color 0 0 0`) |
| `latch_open` / `latch_close` | — | Open/close the hull-latch servo (requires `sensing.launch.py`) |
| `set_light_brightness` | `<0-100>` | Set underwater light brightness, percent (requires `vision.launch.py`) |

### System

| Command | Arguments | Description |
|---------|-----------|-------------|
| `exit` | — | Quit the terminal |

---

## selqie.py API Reference

`selqie.py` is a `rclpy.node.Node` subclass. Import and spin it in your own Python scripts or Jupyter notebooks. **The method names below are copied directly from `ui/selqie_python/selqie_python/selqie.py`** — note that several differ from what earlier documentation described (there is no `start_motors()`/`stop_motors()`/`zero_motors()`/`get_motor_state()`, and `set_leg_position()` takes an integer leg index, not a name).

```python
import rclpy
from selqie_python.selqie import SELQIE

rclpy.init()
robot = SELQIE()
robot.init()         # Initialise all subsystems
robot.spin_background()   # Start a background spin thread (needed before commands take effect)
```

### Initialisation Methods

| Method | Description |
|--------|-------------|
| `init()` | Calls all `init_*` methods below |
| `init_motors()` | Publishers/subscribers for all 8 motors |
| `init_legs()` | Leg command/estimate/trajectory topics for all 4 legs |
| `init_sensors()` | Subscribe to IMU, Bar100, and TinyBMS topics |
| `init_localization()` | Odometry subscription, `set_pose` service client, IMU calibration publisher |
| `init_mapping()` | Map-reset publisher |
| `init_control()` | `cmd_vel`, `goal_pose`, and `gait` publishers/subscription |
| `init_vision()` | Camera light PWM publisher, left/right camera subscriptions |
| `init_led()` | LED color publisher |
| `init_servo()` | Hull-latch servo publisher |
| `init_recording()` | Bag recording utilities |

### Motor Methods

```python
robot.set_motor_ready(motor_idx)          # Send "start" — arm one motor
robot.set_motor_idle(motor_idx)           # Send "exit" — disable one motor
robot.set_motor_position_zero(motor_idx)  # Zero one motor's encoder
robot.set_motor_clear_errors(motor_idx)   # Clear one motor's fault state
robot.set_motor_position(motor_idx, pos_rad)
robot.set_motor_gains(motor_idx, kp, kd, velocity_kd=None)  # Live MIT gain change, no restart
robot.set_all_motor_gains(kp, kd, velocity_kd=None)         # Same, every motor
robot.get_motor_gains(motor_idx)                            # -> [kp, kd, velocity_kd]
robot.get_motor_estimate(motor_idx)       # Returns MotorState for that motor
robot.get_motor_error_name(motor_idx)     # Returns the latest error string
```
`NUM_MOTORS` (8) gives the valid index range. There is no single call that starts/stops "all" motors — the terminal's `ready`/`idle`/`zero`/`clear_errors` commands loop over `range(NUM_MOTORS)` themselves.

### Leg Methods

```python
robot.set_leg_position(leg_idx, x, y, z)   # leg_idx: integer 0-3; use LEG_NAMES.index(name) to convert from "FL"/"RL"/"RR"/"FR"
robot.set_leg_force(leg_idx, fx, fy, fz)
robot.set_leg_position_default(leg_idx)    # Move to DEFAULT_LEG_POSITION
robot.get_leg_estimate(leg_idx)            # Returns LegEstimate
```
`LEG_NAMES = ['FL', 'RL', 'RR', 'FR']` and `NUM_LEGS` (4) are class attributes.

### Trajectory Methods

```python
trajectories = robot.get_leg_trajectories_from_file('walk.txt', frequency_hz)
robot.run_leg_trajectories(trajectories)
```

### LED / Servo / Vision Methods

```python
robot.set_led_color(r, g, b)   # r,g,b: int 0-255
robot.set_led_off()
robot.latch_open()
robot.latch_close()
robot.set_vision_lights_brightness(brightness)   # 0-100
```

### Sensor Methods

```python
voltage, stamp = robot.snapshot_battery_voltage()   # (float | None, float | None) — volts, unix timestamp
robot.get_imu()               # Returns sensor_msgs/Imu
robot.get_pressure()          # Returns Float32 (Bar100 pressure)
robot.get_water_temperature() # Returns Float32
```

### Control / Gait Methods

```python
robot.set_control_gait(gait)                       # gait: "walk"/"swim"/"jump"/"stand"/"sink"/""
robot.set_control_command_velocity(lin_x, lin_z, ang_z)
robot.set_control_goal_pose(x, y, theta)
robot.get_control_gait()
```

### Localization Methods

```python
robot.set_localization_pose(x, y, z, theta)
robot.set_localization_pose_zero()
robot.send_localization_calibrate_imu()
robot.get_localization()   # Returns nav_msgs/Odometry
```

### Recording Methods

```python
robot.is_recording()
robot.start_recording(tag=None)   # Saves to ROSBAG_SAVE_FOLDER = '/home/selqie/rosbags'
robot.stop_recording()
```

### QoS Helpers

```python
from selqie_python.selqie import QOS_FAST, QOS_RELIABLE
```

`QOS_FAST()` — best-effort, depth 10, for sensor data.
`QOS_RELIABLE()` — reliable delivery, depth 10, for commands.

### Geometry Helpers

```python
from selqie_python.selqie import QUAT2EUL, EUL2QUAT
roll, pitch, yaw = QUAT2EUL(quaternion)
quaternion = EUL2QUAT([roll, pitch, yaw])
```

---

## Using selqie.py in a Script

```python
import rclpy
from selqie_python.selqie import SELQIE

rclpy.init()
robot = SELQIE()
robot.init()
robot.spin_background()   # Spins ROS2 in a background thread

# Command the robot
for i in range(robot.NUM_MOTORS):
    robot.set_motor_ready(i)

fl = robot.LEG_NAMES.index('FL')
robot.set_leg_position(fl, 0.0, 0.0, -0.18914)
robot.set_led_color(0, 255, 0)   # Green = running

voltage, stamp = robot.snapshot_battery_voltage()
if voltage is not None:
    print(f"Battery: {voltage:.2f} V")

for i in range(robot.NUM_MOTORS):
    robot.set_motor_idle(i)

robot.stop()
rclpy.shutdown()
```

---

## Interactive RGB Viewer

`selqie_tools/interactive_rgb_viewer.py` opens a simple GUI color picker that publishes to `led_colors` in real time — useful for visually calibrating LED colors.

```bash
ros2 run selqie_tools interactive_rgb_viewer
```
