# Leg Control

Translates high-level Cartesian leg commands into individual motor commands and back, using a 5-bar (fivebar) planar linkage model. Also houses gait (stride) generation nodes for walking, swimming, jumping, sinking, and standing.

---

## Package Layout

```
leg_control/
├── leg_control_bringup/        # Launch and config
│   ├── launch/
│   │   ├── fivebar.launch.py               # Kinematics node per leg
│   │   ├── leg_trajectory_publisher.launch.py
│   │   └── stride_generation.launch.py     # All gait nodes
│   └── config/
│       ├── fivebar.yaml
│       └── stride_generation.yaml
├── leg_kinematics/             # fivebar2d_node (C++)
├── leg_trajectory_publisher/   # Trajectory tracking node
├── stride_generation/          # Gait generator nodes
└── leg_control_msgs/           # Custom message definitions
```

---

## Fivebar Kinematics

Each leg is a symmetric 5-bar planar linkage. The `fivebar2d_node` (C++) runs one instance per leg and performs:

- **Forward kinematics** — joint angles → Cartesian foot position
- **Inverse kinematics** — Cartesian setpoint → joint angle commands
- **Jacobian** — Cartesian velocities/forces ↔ joint velocities/torques

### Leg Instances

| Leg | Motor 0 | Motor 1 | flip_y |
|-----|---------|---------|--------|
| FL | motor0 | motor1 | true |
| RL | motor2 | motor3 | true |
| RR | motor4 | motor5 | false |
| FR | motor6 | motor7 | false |

`flip_y = true` mirrors the Y-axis so left-side legs share the same kinematic model as right-side legs.

### Topics per Leg

Note: leg topics have **no underscore** between `leg` and the leg name (e.g. `legFL/command`), unlike the motor topics.

| Topic | Type | Direction |
|-------|------|-----------|
| `leg{name}/command` | `LegCommand` | In — Cartesian setpoint from stride gen (e.g. `legFL/command`) |
| `leg{name}/estimate` | `LegEstimate` | Out — Cartesian feedback |
| `leg{name}/trajectory` | `LegTrajectory` | In — feed-forward trajectory waypoints |
| `/motor{N}/command` | `MotorCommand` | Out — joint motor commands |
| `/motor{N}/estimate` | `MotorEstimate` | In — joint motor feedback |

---

## Stride Generation

Five gait nodes run continuously and publish `LegCommand` messages based on the current gait state:

| Node | Gait | Description |
|------|------|-------------|
| `walk_stride_node` | Walk | Alternating leg trot pattern for ground locomotion |
| `swim_stride_node` | Swim | Rowing stroke pattern for underwater propulsion |
| `jump_stride_node` | Jump | Simultaneous leg extension for hops |
| `sink_stride_node` | Sink | Controlled descent stroke |
| `stand_stride_node` | Stand | Static stance with configurable foot positions |

Gait parameters (step height, stride length, frequency, foot positions) are loaded from `leg_control_bringup/config/stride_generation.yaml`.

---

## Leg Trajectory Publisher

The `leg_trajectory_publisher` node sits between the stride generators and the kinematics node. It accepts `LegTrajectory` messages and replays them as a smooth stream of `LegCommand` messages, interpolating between waypoints.

---

## Message Definitions

**LegCommand**
```
uint32 control_mode       # 1=FORCE  2=VELOCITY  3=POSITION
geometry_msgs/Vector3 pos_setpoint   # m
geometry_msgs/Vector3 vel_setpoint   # m/s
geometry_msgs/Vector3 force_setpoint # N
```

**LegEstimate**
```
geometry_msgs/Vector3 pos_estimate   # m
geometry_msgs/Vector3 vel_estimate   # m/s
geometry_msgs/Vector3 force_estimate # N
```

**LegTrajectory**
```
float64[]    timing       # s from start, one per command
LegCommand[] commands

# Optional playback control (all default to 0 = play once, anchored on arrival)
uint32  loops             # repetitions to play back-to-back; 0/1 = once
float64 period            # cycle duration (s) used to schedule repeats
float64 start_time        # ROS time (s) to start at; shared across legs
```

`loops`/`period`/`start_time` let `leg_trajectory_publisher_node` repeat a stride natively, so a
multi-loop run publishes once instead of republishing every cycle — see
[Actuation](../actuation/README.md#multi-loop-runs-repeat-natively-no-republishing) for why that
matters for motion quality.

---

## Coordinate Frame

The foot position is expressed in the **leg frame** with origin at the hip pivot:

```
+X  → forward (direction of travel)
+Y  → outward (away from robot center)
+Z  → upward
```

Default stance foot position (from `selqie.py`'s `DEFAULT_LEG_POSITION`, used by the terminal's `default` command, and matched by `stand_stride_node`'s `standing_height`): `x=0.0, y=0.0, z=-0.18914` (≈18.9 cm below hip, directly below).

---

## Tuning Tips

- **Stance height** — adjust `z` in the stand gait config; lower values increase ground clearance.
- **Step frequency** — gait period controls speed; too fast a period with high position gains causes torque saturation.
- **Velocity mode** — for swimming, VELOCITY control mode reduces position-induced stiffness and allows compliant paddling.
- **Force mode** — useful for compliant terrain following; requires torque feedback from the motor estimate.

---

## Launch Arguments

`fivebar.launch.py` arguments:

| Argument | Description |
|----------|-------------|
| `leg_name` | FL / RL / RR / FR |
| `motor0` | ID of the inner-shaft motor |
| `motor1` | ID of the outer-shaft motor |
| `flip_y` | Mirror Y-axis for left-side legs |

`stride_generation.launch.py` arguments:

| Argument | Description |
|----------|-------------|
| `use_sim_time` | Use `/clock` topic for simulation |
