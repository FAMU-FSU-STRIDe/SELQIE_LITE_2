# Actuation

Controls all eight CubeMars AK40-10 brushless motors via CAN bus using the CubeMars **Servo Mode** protocol.

---

## Package Layout

```
actuation/
├── actuation_bringup/          # Launch files
│   └── launch/
│       ├── can.launch.py       # Bring up a single CAN interface
│       └── cubemars.launch.py  # Launch one motor node
├── cubemars_v2_ros/            # Motor driver
│   └── cubemars_v2_ros/
│       ├── servo_protocol.py   # Pure servo-mode CAN packing / unit conversion
│       └── motor_node.py       # ROS 2 node
├── can_bus/                    # Low-level SocketCAN helpers
├── actuation_msgs/             # Custom ROS2 message definitions
└── motor_interfaces/           # Shared motor state message
```

---

## Servo Mode Protocol

The motors run in **Servo Mode** (AK Series Module Driver Manual V1.0.18, §5), **not** the MIT
protocol. Two consequences drive the whole design:

1. **No Kp / Kd.** The position and velocity control loops run *inside the driver* and are
   configured over R-LINK, not over CAN. No gain is ever transmitted. Each CAN frame commands
   exactly one quantity.
2. **Different units.** Servo mode speaks degrees / ERPM / amperes, while the SELQIE leg-control
   stack speaks radians / rad·s⁻¹ / N·m at the output joint. The node converts between the two so
   the ROS interface is unchanged — trajectories keep publishing radians.

### Servo frames

All frames are **CAN 2.0 extended (29-bit)**: `CAN ID = (packet_id << 8) | node_id`.

| MotorCommand mode | Servo packet | Command unit | Conversion from ROS units |
|-------------------|--------------|--------------|---------------------------|
| `POSITION` (3) | `SET_POS` (4) / `SET_POS_SPD` (6) † | output-shaft degrees × 10000 | `deg = rad × 180/π` |
| `VELOCITY` (2) | `SET_RPM` (3) | rotor ERPM | `ERPM = rad/s × (60/2π) × gear × pole_pairs` |
| `TORQUE` (1) | `SET_CURRENT` (1) | phase current × 1000 (mA) | `I = τ / (Kt × gear)` |

The all-stride trajectory path uses **POSITION** mode, whose conversion is a pure rad↔deg scaling
(no gear factor — servo position is referenced to the output shaft). Velocity and torque modes
additionally need the gear ratio, pole pairs, and torque constant listed below.

† **Position streaming (`pos` vs `pos_spd`).** POSITION mode has two implementations, selected at
runtime with the `pos`/`pos_spd` special commands (startup default from the `position_mode`
parameter). The SELQIE UI picks between them by context: **`pos` for all gaits, `pos_spd` only for
the stand/ready hold.**

* **`pos` — plain `SET_POS`.** The driver drives to each streamed setpoint using the motor's **full
  physical acceleration**, so it tracks position accurately at *every* gait frequency — used for all
  gaits. It is not acceleration-shaped, so a *coarse* setpoint stream would move as a slam-and-wait
  staircase and can ring; the node streams finely at a high `control_hz` (default **500 Hz**) so the
  position steps are small and the motion stays smooth.

* **`pos_spd` — `SET_POS_SPD` (§5.1.7)** with a velocity feed-forward derived from the change in
  commanded position over one control period, plus a bounded acceleration. Smooth — used for the
  gentle stand/ready hold. **Not** used for gaits: its acceleration field is protocol-capped at
  `pos_spd_accel` ≈ 327670 ERPM/s (~245 rad/s² at the AK40-10 output), and gait acceleration demand
  grows with frequency *squared*, so above ~1–1.5× base frequency the motor cannot keep up and
  **positional accuracy is lost**.

  In `pos_spd`, the speed feed-forward is clamped to the motor's `V_MAX`, and a held/static setpoint
  (e.g. the `stand` pose) produces zero feed-forward — so a minimum approach speed
  (`pos_spd_min_speed`, rad/s) floors the commanded speed, letting held poses and the first move
  still reach their target. It only binds when the trajectory is (near-)stationary.

**Why `control_hz` matters.** The leg stack streams setpoints at 500–1000 Hz (the trajectory point
rate × frequency). The motor node samples the latest setpoint at `control_hz`; too low a rate both
coarsens the motion (staircase → ring) and discards trajectory detail. 500 Hz captures the trajectory
detail and stays within the 1 Mbps CAN budget (4 motors per bus, ~50% bus load with status feedback).

> **Notation trap:** servo **position** is output-shaft referenced, but servo **speed** is
> *rotor-electrical* (ERPM). That asymmetry is why velocity conversion carries a `gear × pole_pairs`
> factor and position does not.

### Feedback (status frame `0x29`)

| Field | Raw type | Scale | ROS value |
|-------|----------|-------|-----------|
| position | int16 | ×0.1 → deg | `pos = deg × π/180` (rad) |
| speed | int16 | ×10 → ERPM | `vel = ERPM ÷ (gear × pole_pairs) × 2π/60` (rad/s) |
| current | int16 | ×0.01 → A | `torque = I × Kt × gear` (N·m) |
| temperature | int8 | °C | °C |
| error | uint8 | — | 0–7 fault code |

---

## Supported Motor Types

| Model | V_MAX (rad/s) | T_MAX (Nm) | Gear | Pole pairs | Kt (Nm/A) |
|-------|--------------|------------|------|-----------|-----------|
| AK10-9 | ±50 | ±65 | 9 | 21 | 0.198 |
| AK40-10 | ±45.5 | ±4.1 | 10 | 14 † | 0.056 |
| AK60-6 | ±45 | ±15 | 6 | 14 | — |
| AK70-10 | ±50 | ±25 | 10 | 21 | 0.123 |
| AK80-6 | ±76 | ±12 | 6 | 21 | — |
| AK80-8 | ±37.5 | ±32 | 8 | 21 | — |
| AK80-9 | ±50 | ±18 | 9 | 21 | — |
| AK80-64 | ±9.2 | ±144 | 64 | 21 | 0.136 |

SELQIE Lite 2 uses **AK40-10** motors exclusively. † The AK40-10 row is datasheet-verified
(24 slots / 14 pole pairs, KT 0.056 Nm/A, 10:1, 4.1 Nm peak torque = 7.3 A peak current). Pole-pair
values for the other models are best-guess (21 is common for the AK series) and only affect
VELOCITY-mode scaling — verify them against your motors and override with the `pole_pairs` parameter.
If a torque constant or gear ratio for a given model is missing above, torque/velocity conversion
falls back to a safe default (current 0 / gear 1).

---

## ROS2 Interface

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/motor_state` | `MotorState` | Full feedback at the driver's upload rate |
| `/{joint}/estimate` | `MotorEstimate` | Position/velocity/torque for kinematics |
| `/{joint}/error_code` | `String` | Fault code with human-readable message |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/command` | `MotorCommand` | High-level command (position/velocity/torque mode) |
| `/{joint}/servo_cmd` | `Float64MultiArray` | Raw bench command `[mode, value]` (legacy 5-tuple accepted; gains ignored) |
| `/{joint}/special_cmd` | `String` | `"start"`, `"exit"`, `"zero"`, `"clear"` |

### Message Definitions

**MotorCommand**
```
uint32 control_mode    # 1=TORQUE  2=VELOCITY  3=POSITION
uint32 input_mode      # (ignored by CubeMars — always direct passthrough)
float32 pos_setpoint   # rad
float32 vel_setpoint   # rad/s
float32 torq_setpoint  # Nm
```

**MotorState**
```
string  name
float32 position       # rad (raw, wrapped within the feedback range)
float32 abs_position   # rad (unwrapped, tracks multiple revolutions)
float32 velocity       # rad/s
float32 torque         # Nm (output shaft)
float32 current        # A (motor phase current)
int32   temperature    # °C
```

**MotorEstimate**
```
float32 pos_estimate   # rad (unwrapped)
float32 vel_estimate   # rad/s
float32 torq_estimate  # Nm
```

---

## Motor Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `motor_id` / `can_id` | `0` | CAN node ID (0–7) |
| `motor_type` | `AK40-10` | Motor model string |
| `interface` / `can_interface` | `can0` | SocketCAN interface name |
| `control_hz` | `500.0` | Setpoint stream / command rate. Higher = finer, smoother position streaming |
| `pole_pairs` | `0` | Rotor pole pairs for ERPM scaling (`0` = per-motor table default) |
| `gear_ratio` | `0.0` | Gear reduction for ERPM/torque scaling (`0` = per-motor table default) |
| `position_mode` | `pos_spd` | Startup POSITION submode: `pos_spd` (feed-forward, smooth) or `pos` (plain SET_POS, accurate at all freq). Switchable at runtime via the `pos`/`pos_spd` special commands |
| `pos_spd_accel` | `327670.0` | Acceleration limit (ERPM/s) for `pos_spd` streaming (protocol max) |
| `pos_spd_min_speed` | `2.0` | Minimum approach speed (rad/s) for `pos_spd`; lets held poses (stand) reach their target |
| `reverse_polarity` | `false` | Negate position/velocity/torque |
| `cmd_timeout` | `0.5` | Seconds before a stale command releases the motor (0 = disabled) |
| `auto_start` | `false` | Enable motor on node startup |

There are **no gain parameters** — servo mode has no Kp/Kd. Tune the position/velocity loops in the
R-LINK upper computer instead.

---

## Tuning

Servo-mode loop gains are configured in the **R-LINK upper computer**, not in ROS. The launch files
carry no gain constants. `selqie_bringup/launch/actuation.launch.py` only maps motor IDs to CAN
interfaces and (optionally) sets `reverse_polarity` per shaft group.

> **Note:** `InnerShaft()`/`OuterShaft()` both currently launch with `reverse_polarity='false'`.
> As written, no motor launches with reversed polarity — polarity is handled inside the five-bar
> kinematics. Confirm this before relying on the "inner/outer" language elsewhere.

---

## Command Timeout Safety

`cmd_timeout` (default 0.5 s) protects against a stalled gait node. If no new command arrives within
that window the motor node:

1. Sets `_neutral_hold = true`
2. Sends a zero-current frame so the motor produces no torque (releases)
3. Logs a warning

Set `cmd_timeout: 0.0` to disable (not recommended for hardware runs). Note that on a legged robot,
releasing torque means the leg goes limp — the same behaviour the MIT node had when it sent all
zeros.

---

## CAN Bus

Two SocketCAN interfaces are used:

| Interface | Motors | Legs |
|-----------|--------|------|
| `can0` | 0, 1, 6, 7 | FL, FR |
| `can1` | 2, 3, 4, 5 | RL, RR |

The CAN interfaces are brought up by `actuation_bringup/launch/can.launch.py` using the
`loadcan_jetson.sh` script, which writes pin-mux registers via `devmem` and calls
`ip link set canN up type can bitrate 1000000`. The AK driver CAN bit rate is fixed at **1 Mbps**.

To verify CAN traffic:
```bash
candump can0
candump can1
```

---

## Special Commands

Send a string to `/{joint}/special_cmd`:

| Command | Effect |
|---------|--------|
| `start` | Enable command output (must be sent before motion commands) |
| `exit` | Release the motor (zero current) and stop driving it |
| `zero` | Set the current position as the (temporary) origin |
| `clear` | Neutral hold — release torque until a new command arrives |
| `pos` | Switch POSITION streaming to plain `SET_POS` (accurate; used for all gaits) |
| `pos_spd` | Switch POSITION streaming to `SET_POS_SPD` (smooth; used for the stand/ready hold) |

The SELQIE UI drives `pos`/`pos_spd` automatically: `run_trajectory` selects `pos` for the gait and
returns to `pos_spd` when the run completes; `ready` and `stand` select `pos_spd` for the held pose.

Example:
```bash
ros2 topic pub --once /motor0/special_cmd std_msgs/msg/String "data: 'start'"
```

---

## Tests

Pure-protocol and node-level command conversion are covered by unit tests that need neither ROS nor
a CAN bus:

```bash
python3 -m pytest actuation/cubemars_v2_ros/test/test_servo_protocol.py \
                  actuation/cubemars_v2_ros/test/test_motor_node_servo.py
```
