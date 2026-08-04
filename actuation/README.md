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
parameter). **`pos_spd` is the default for gaits and holds alike**; `pos` is the escape hatch for
runs fast enough to outrun the acceleration cap (see the ceiling table below).

* **`pos_spd` — `SET_POS_SPD` (§5.1.7).** Position, a **travel speed limit**, and a bounded
  acceleration. The driver shapes the move instead of slamming to each setpoint, which is what keeps
  wide/fast gaits like swim from ringing.

  The speed field is the crux: it is a *limit on the move*, not a target to hold, so whatever is put
  there is the fastest the leg will travel no matter how far ahead the setpoints run. The node takes
  it from `MotorCommand.vel_setpoint` — the joint velocity `leg_kinematics` derives from the
  trajectory's Cartesian velocity through the inverse Jacobian — so it is the stride's own intended
  speed, exact and independent of transport timing. It is clamped to the motor's `V_MAX`.

  For commands that carry position alone (`set_motor_position`, a held pose) the node falls back to
  differencing consecutive commanded positions at `control_hz`, and floors the result at
  `pos_spd_min_speed` (rad/s) so a static setpoint still travels to its target.

* **`pos` — plain `SET_POS`.** The driver drives to each streamed setpoint using the motor's **full
  physical acceleration**, so it tracks position accurately at *every* gait frequency, giving up the
  acceleration shaping to do it. A *coarse* setpoint stream then moves as a slam-and-wait staircase
  and can ring, so pair it with a high `control_hz` (default **500 Hz**) to keep the steps small.

**Why the trajectory files' velocities matter.** Every velocity column in
`leg_trajectory_publisher/trajectories/*.txt` is zero, and the C++ stride generators likewise set
position only. Fed straight through, `pos_spd`'s speed field collapsed to the `pos_spd_min_speed`
floor and **the gait ran slow** — and because the fallback difference samples a zero-order-held
signal at the control rate, a tick that saw no new command read zero while a tick that saw two read
double; since the field is a limit, the low samples throttled the move and the high ones could not
make up for it. Both producers now fill in velocities by central difference over the cycle
(`estimate_leg_trajectory_velocities` in `selqie_python.selqie`, `_fill_setpoint_velocity` in
`stride_generation_node.hpp`), so the speed limit is the one the stride actually calls for.

**The acceleration ceiling.** `pos_spd_accel` is protocol-capped at 327670 ERPM/s² ≈ **245 rad/s²**
at the AK40-10 output. That bounds how fast the driver may change its travel speed, so a stride whose
joint velocity has to reverse by Δv needs `Δv / 245` seconds just to turn around. Once that exceeds
the cycle itself the leg cannot keep up and lags its setpoints silently — the motor node warns
(throttled) when a commanded speed step outruns the budget. Measured from the shipped files, as a
fraction of one cycle spent ramping:

| `run_trajectory` frequency | `walk.txt` | `swim.txt` | `walk_20cm_stride.txt` | `jump.txt` |
|---|---|---|---|---|
| 0.5 Hz | 1 % | 1 % | 3 % | 15 % ‡ |
| 1 Hz | 5 % | 5 % | 11 % | 61 % ‡ |
| 2 Hz | 18 % | 21 % | 46 % | 244 % ‡ |
| 3 Hz | 41 % | 48 % | **103 %** | 548 % ‡ |
| 5 Hz | **113 %** | **133 %** | **286 %** | 1523 % ‡ |

Bold entries cannot be tracked in `pos_spd` — use `pos` there. ‡ `jump.txt` also exceeds the motor's
45.5 rad/s speed limit at every frequency (3.2× at 1 Hz), so it is speed-bound before it is
acceleration-bound and both submodes run it at `V_MAX`.

**Why `control_hz` matters.** `run_trajectory` resamples gait files to a constant
`TRAJECTORY_RESAMPLE_HZ` (default 500 Hz, see `selqie_python.selqie`) regardless of the run
frequency, so the setpoint stream rate is bounded at any frequency rather than scaling with it — see
"`run_trajectory` and setpoint resampling" below. The motor node samples the latest setpoint at
`control_hz`; it should match (or exceed) `TRAJECTORY_RESAMPLE_HZ`, since a lower rate would just
undersample the resampled stream and reintroduce the staircase/ring problem. Default is **500 Hz**
on both sides. CAN load at 500 Hz TX + up to 500 Hz status feedback (the driver's upload-rate cap,
§5.2.1) is ~54% of a 1 Mbps bus with 4 motors, leaving comfortable headroom.

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

## `run_trajectory` and setpoint resampling

Gait trajectory files (`leg_trajectory_publisher/trajectories/*.txt`) store one cycle as fixed
timestamped setpoints — e.g. 500 points/leg over a 1 s base. Running a file at frequency `f`
compresses that cycle to `duration/f` seconds. Naively replaying the file's original points in that
compressed window makes the required setpoint rate scale directly with `f`
(`rate = file_points × f`): at 5× on a 500-point/1 s file that's 2500 setpoints/s/motor, well past
what a 1 Mbps CAN bus with 4 motors can sustain (~1500/s/motor is a realistic ceiling) — the excess
either gets dropped unevenly or truncates the stride before it finishes.

`SELQIE.get_leg_trajectories_from_file` (in `selqie_python.selqie`) avoids this by **resampling**
each leg's cycle to a constant setpoint rate, `TRAJECTORY_RESAMPLE_HZ` (default **500 Hz**),
independent of `f`. The cycle is linearly interpolated and re-sampled evenly, so the delivered rate
is bounded at *every* run frequency:

A second bound, `TRAJECTORY_MAX_POINTS` (default **1000**), caps how many points a single cycle may
contain. Rate alone does not bound *message size*: at a low `f` the cycle is long, so a fixed rate
keeps adding points. Together they give:

| `f` | points/cycle (rate only) | points/cycle (with cap) | bytes sent per run (4 legs) |
|---|---|---|---|
| 0.25 | ~2000 | **1000** | 656 KB → **328 KB** |
| 0.5 | ~1000 | **1000** | 328 KB (cap just binds) |
| 1 | ~500 | ~500 | 165 KB (matches the source file) |
| 2 | ~250 | ~250 | 82 KB |
| 5 | ~100 | ~100 | 33 KB |

Nothing is unevenly dropped or the stride cut short — every frequency gets a complete, evenly-spaced
cycle, just at a resolution that trades off against frequency instead of exceeding the transport's
bandwidth. When the cap binds, the step is widened so the capped points still span the whole cycle.

Capping costs little in smoothness: what matters is the position delta *per setpoint*, not the
absolute rate, and foot speed scales down with `f`, so a fixed points-per-cycle budget holds that
delta roughly constant. The cap only binds at low `f`, where 1000 points is still denser than the
source files (330–500 points/cycle).

`TRAJECTORY_RESAMPLE_HZ` should be kept **≤ the motor nodes' `control_hz`** — resampling faster than
the motor node consumes buys nothing, since the node just samples its latest cached command each
control tick. Both default to 500 Hz.

### Multi-loop runs repeat natively (no republishing)

A `LegTrajectory` carries optional playback fields — `loops`, `period`, and `start_time` — that
`leg_trajectory_publisher_node` honours. `run_trajectory` therefore publishes each leg's trajectory
**once for the whole run** and lets the C++ node repeat it, instead of republishing once per cycle.

This matters because *every* trajectory message resets the publisher to the start of the stride.
Republishing per cycle meant that reset had to land exactly at the stride boundary; any timing
slop landed it mid-stride and snapped the foot back — a twitch whose size was exactly the timing
error. And since the 4 legs are published sequentially, their resets landed staggered, so a single
leg could twitch on its own. Repeating natively wraps only once a repetition has genuinely
finished, and advances the anchor by exactly one `period`, so repeats stay phase-exact and
drift-free with nothing to mistime.

`start_time` stamps one shared start instant on all 4 legs, so they begin phase-aligned rather than
each at its own message-arrival time. All fields default to zero, which reproduces the original
behaviour (play once, anchored on arrival) for other producers such as `swing_leg_node`.

Because motion timing now lives entirely in the C++ publisher, it no longer depends on Python or on
the ROS executor — which is significant, since `SELQIE` spins single-threaded while absorbing
`motor_state` and `error_code` from 8 motors (up to ~8000 msg/s at a 500 Hz status upload rate).

The `leg_trajectory_publisher` C++ node polls at 1000 Hz and advances to whatever command is due
*now* rather than one command per tick, so it keeps pace with the resampled stream.

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
| `control_hz` | `500.0` | Setpoint stream / command rate. Should match `TRAJECTORY_RESAMPLE_HZ` (selqie_python) |
| `pole_pairs` | `0` | Rotor pole pairs for ERPM scaling (`0` = per-motor table default) |
| `gear_ratio` | `0.0` | Gear reduction for ERPM/torque scaling (`0` = per-motor table default) |
| `position_mode` | `pos_spd` | Startup POSITION submode: `pos_spd` (acceleration-shaped, travels at the commanded velocity) or `pos` (plain SET_POS, accurate at all freq). Switchable at runtime via the `pos`/`pos_spd` special commands |
| `pos_spd_accel` | `327670.0` | Acceleration limit (ERPM/s) for `pos_spd` streaming (protocol max) |
| `pos_spd_min_speed` | `2.0` | Minimum approach speed (rad/s) for `pos_spd`, applied only when the command carries no velocity; lets held poses (stand) reach their target |
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
