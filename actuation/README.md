# Actuation

Controls all eight CubeMars AK40-10 V2.0 brushless motors over CAN using the CubeMars **MIT protocol**
(*AK Series Module Driver Manual* V1.0.18 §5.3).

> **Tuning gains?** Everything you need is in
> [`actuation_bringup/config/mit_gains.yaml`](actuation_bringup/config/mit_gains.yaml) — one file,
> all motors, with a symptom→fix table. Gains can also be changed **live** without relaunching:
> `ros2 topic pub --once /motor0/set_gains std_msgs/msg/Float64MultiArray "{data: [5.0, 0.4]}"`

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
│       ├── mit_protocol.py     # MIT-mode CAN packing / unpacking (§5.3)
│       ├── servo_protocol.py   # Servo-mode protocol (§5.1-5.2), retained
│       └── motor_node.py       # ROS 2 node (MIT)
├── can_bus/                    # Low-level SocketCAN helpers
├── actuation_msgs/             # Custom ROS2 message definitions
└── motor_interfaces/           # Shared motor state message
```

---

## MIT Protocol

The motors are driven with the **MIT ("Mini-Cheetah") protocol** (manual §5.3). The driver also
speaks a Servo protocol (§5.1–5.2); the decisive difference is where the gains live:

| | MIT mode (used here) | Servo mode |
|---|---|---|
| Kp / Kd | **sent in every CAN frame** — tunable live from ROS | inside the driver, R-LINK only |
| CAN ID | standard 11-bit | extended 29-bit |
| Units | rad, rad/s, N·m (output shaft) | degrees, ERPM, amperes |
| Command | one frame carries position + velocity + gains + torque | one quantity per frame |

Because MIT transmits the gains, retuning is a topic publish rather than a USB session — which is
why this driver uses it.

### Control law

The driver closes this loop onboard every cycle (§5.3 block diagram):

```
torque = Kp × (pos_setpoint − pos_measured)
       + Kd × (vel_setpoint − vel_measured)
       + torq_setpoint
```

then clamps to the torque limit and hands it to the FOC current loop. So `Kp` is **stiffness**,
`Kd` is **damping**, and `torq_setpoint` is a feed-forward term. The three ROS control modes are
just different corners of this one law:

| MotorCommand mode | Kp | Kd | Effect |
|---|---|---|---|
| `POSITION` (3) | `position_kp` | `position_kd` | Servo to a position with damping |
| `VELOCITY` (2) | 0 | `velocity_kd` | No position target; track a velocity |
| `TORQUE` (1) | 0 | 0 | Pure feed-forward torque |

### Frame layout (§5.3)

Standard 11-bit IDs, DLC 8, **1 Mbit/s**. Command frames go to the motor ID; replies return on
`0x00 + drive ID`.

**Command (host → driver)** — position 16-bit, velocity/Kp/Kd/torque 12-bit each:

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Field | pos[15:8] | pos[7:0] | vel[11:4] | vel[3:0]\|Kp[11:8] | Kp[7:0] | Kd[11:4] | Kd[3:0]\|τ[11:8] | τ[7:0] |

**Reply (driver → host):**

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Field | drive ID | pos[15:8] | pos[7:0] | vel[11:4] | vel[3:0]\|cur[11:8] | cur[7:0] | temp | error |

> The manual's reply table mislabels bytes 2–4 (it repeats "motor position" three times where speed
> and current belong). The layout above is what the firmware implements and what the manual's own
> field widths imply.

Each field is quantized linearly across the motor's configured range, so **the ranges in
`motor_node.LIMITS` must match the driver's own configuration** — otherwise the motor decodes
different values than were sent.

### Special commands (§5.3)

Seven `0xFF` bytes followed by a code. **Entering MIT mode is mandatory before any motion command.**

| Code | Meaning | Sent by |
|---|---|---|
| `0xFC` | Enter motor control mode | `start` (or `auto_start`) |
| `0xFD` | Exit motor control mode | `exit`, and on node shutdown |
| `0xFE` | Set current position to zero | `zero` |

---

## Tuning the gains

**Edit [`actuation_bringup/config/mit_gains.yaml`](actuation_bringup/config/mit_gains.yaml)** — it
is loaded by every motor node, so one edit covers all 8 motors, and no rebuild is needed for a YAML
change. It carries the full symptom→fix table inline.

Three ways to set gains, in increasing convenience:

1. **The YAML file** — the durable, shared default.
2. **Launch overrides** — `ros2 launch ... position_kp:=8.0 position_kd:=0.5` (blank = use the YAML).
3. **Live, mid-run** — no relaunch. From `selqie_terminal`:
   ```
   SELQIE> set_gains 8.0 0.6              # all motors
   SELQIE> set_motor_gains 3 8.0 0.6      # just motor 3
   SELQIE> gains                          # read back what every motor is using
   ```
   Or straight from the command line:
   ```bash
   ros2 topic pub --once /motor0/set_gains std_msgs/msg/Float64MultiArray "{data: [8.0, 0.6]}"
   ros2 topic echo /motor0/gains
   ```
   Both accept an optional third value to also set `velocity_kd`. Each node
   republishes its active gains at 1 Hz, so `gains` always has fresh data even if
   the terminal started after the motors.

   Live values are **not** persisted — copy anything you like back into the YAML.

Quick reference:

| Symptom | Change |
|---|---|
| Leg sags / lags behind target | raise `position_kp` |
| Buzzing, ringing, oscillation | raise `position_kd`, or lower `position_kp` |
| Sluggish, over-damped | lower `position_kd` |
| Harsh, slams into position | lower `position_kp` **and** raise `position_kd` |
| Motor hot / torque saturating | lower `position_kp` |

**Start low.** The AK40-10 peaks at 4.1 N·m, so at `Kp = 20` a mere 0.2 rad error already saturates
the motor. Sensible starting ranges: `position_kp` 1–10, `position_kd` 0.1–1.0. Protocol ceilings
are `Kp ≤ 500`, `Kd ≤ 5`; anything larger is clipped before packing.

`torque_limit_scale` (0–1) shrinks the commanded torque range — useful while first bringing gains
up, so a bad gain cannot command full torque into a hard stop. It also improves the 12-bit torque
resolution over the reduced range.

### If the motors whine

**A bare `ready` sends `Kp = Kd = torque = 0`** — the motor is deliberately limp, and no gain in the
YAML can be the cause. The whine at that moment is the driver's FOC current loop energising, which
these drivers do audibly. Confirm by hand: if the output shaft still back-drives freely, the driver
is idling, not fighting a command. `idle` (`0xFD`) de-energises it and the whine should stop.

Whine that only appears **once a position is commanded** *is* a gain problem. Bisect it live:

```
set_gains 5.0 0.0     # Kd off — whine stops? Kd is amplifying encoder velocity noise; lower it
set_gains 0.0 0.4     # Kp off — whine stops? Kp is too stiff; lower it
gains                 # confirm what every motor actually took
```

Whine whose pitch tracks `control_hz` points at CAN bandwidth instead — see
[CAN bandwidth](#can-bandwidth). At the shipped 500 Hz there is ample headroom, so this only applies
if the rate has been raised.

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

### Velocity feed-forward (why the files' zeros matter)

The shipped trajectory files carry **position setpoints only** — every velocity
and force column is zero. Under the MIT control law that is actively harmful:

```
torque = Kp·(p_des − p_meas) + Kd·(v_des − v_meas) + t_ff
```

With `v_des = 0`, the Kd term becomes `−Kd·v_meas` — damping against **the motion
itself** rather than against tracking error, so it fights the very stride it is
supposed to execute. On `walk_20cm_stride.txt` at `Kd = 0.4`:

Every value below is the **damping torque** `|Kd·(v_des − v_meas)|` in N·m — the
two right-hand columns are the same quantity under the two conditions. (`v_des`
itself is a velocity in rad/s; it is the product with `Kd` that has units of
torque, since `Kd` is N·m·s/rad.)

| gait freq | peak joint speed | damping torque, `v_des = 0` | damping torque, `v_des` estimated |
|---|---|---|---|
| 0.5 | 7.6 rad/s | 3.05 N·m (74% of peak) | 0.05 N·m (1.3%) |
| 1 | 15.3 rad/s | 6.11 N·m (**149%**) | 0.11 N·m (2.6%) |
| 2 | 30.5 rad/s | 12.21 N·m (298%) | 0.22 N·m (5.2%) |
| 3 | 45.8 rad/s | 18.32 N·m (447%) | 0.32 N·m (7.9%) |

At 1 Hz the damping term alone demands more than the motor's entire peak torque,
purely to oppose its own commanded motion — the motor saturates fighting itself
and the leg lags badly.

`get_leg_trajectories_from_file` therefore estimates `vel_setpoint` from the
position derivative (`estimate_leg_trajectory_velocities`). The resampled cycle
is exactly uniform in time and periodic, so a **central difference** is clean:
second-order accurate, and free of the half-sample phase lag a forward
difference would introduce — which matters, because a lagging feed-forward would
push against the motion much like the zeros it replaces. Measured against an
analytic sine derivative the error is **0.003% of peak**, with no spike at the
cycle wrap.

The Kd term then reads `Kd·(v_des − v_meas)`: ≈0 while tracking well, and biting
only on genuine deviation — which is what damping should do.

This runs only when a file supplies no velocities, so a trajectory that does
specify them is respected. Set `TRAJECTORY_ESTIMATE_VELOCITY = False` in
`selqie_python.selqie` to send the file's zeros verbatim.

---

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

These are the ranges the MIT frame quantizes against (`motor_node.LIMITS`). **They must match the
driver's own configuration** — if they disagree, the motor decodes different values than were sent.

| Model | P (rad) | V_MAX (rad/s) | T_MAX (N·m) | Gear | Kt (N·m/A) |
|-------|---------|--------------|-------------|------|-----------|
| AK10-9 | ±12.5 | ±50 | ±65 | 9 | 0.16 |
| AK40-10 † | ±12.5 | ±45.5 | ±4.1 | 10 | 0.056 |
| AK60-6 | ±12.5 | ±45 | ±15 | 6 | — |
| AK70-10 | ±12.5 | ±50 | ±25 | 10 | 0.123 |
| AK80-6 | ±12.5 | ±76 | ±12 | 6 | — |
| AK80-8 | ±12.5 | ±37.5 | ±32 | 8 | — |
| AK80-9 | ±12.5 | ±50 | ±18 | 9 | — |
| AK80-64 | ±12.5 | ±8 | ±144 | 64 | 0.136 |

Kp is always 0–500 and Kd 0–5 (fixed by the protocol, §5.3).

SELQIE Lite 2 uses **AK40-10 V2.0** motors exclusively. † That row is datasheet-verified: 24 slots /
14 pole pairs, KT 0.056 N·m/A, 10:1 reduction, 4.1 N·m peak torque (= 7.3 A peak current, which the
two figures corroborate), and 435 rpm no-load = 45.5 rad/s at the output.

Gear ratio and Kt are used **only to report phase current** alongside torque — MIT mode commands
torque directly, so unlike servo mode neither value affects what is actually commanded. Pole pairs
are not needed at all in MIT mode.

---

## ROS2 Interface

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/motor_state` | `MotorState` | Full feedback at the driver's upload rate |
| `/{joint}/estimate` | `MotorEstimate` | Position/velocity/torque for kinematics |
| `/{joint}/error_code` | `String` | Fault code with human-readable message |
| `/{joint}/gains` | `Float64MultiArray` | Currently active `[position_kp, position_kd, velocity_kd]` |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `/{joint}/command` | `MotorCommand` | High-level command (position/velocity/torque mode) |
| `/{joint}/mit_cmd` | `Float64MultiArray` | Raw bench command `[p, v, kp, kd, τ]` |
| `/{joint}/set_gains` | `Float64MultiArray` | Live gain override `[kp, kd]` (or `[kp, kd, velocity_kd]`) |
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
| `control_hz` | `500.0` | MIT frame rate. From `mit_gains.yaml`. Should match `TRAJECTORY_RESAMPLE_HZ` (selqie_python); see [CAN bandwidth](#can-bandwidth) |
| `position_kp` | `5.0` | POSITION-mode stiffness (0–500). From `mit_gains.yaml` |
| `position_kd` | `0.4` | POSITION-mode damping (0–5). From `mit_gains.yaml` |
| `velocity_kd` | `0.5` | VELOCITY-mode damping (0–5). From `mit_gains.yaml` |
| `torque_limit_scale` | `1.0` | Scale the commanded torque range (0–1) |
| `reverse_polarity` | `false` | Negate position/velocity/torque |
| `cmd_timeout` | `0.5` | Seconds before a stale command releases the motor (0 = disabled) |
| `auto_start` | `false` | Enter MIT mode on node startup |

Gains come from `mit_gains.yaml` unless a launch argument overrides them — see
[Tuning the gains](#tuning-the-gains).

> **Note:** `InnerShaft()`/`OuterShaft()` in `selqie_bringup/launch/actuation.launch.py` both
> currently launch with `reverse_polarity='false'`. As written, no motor launches with reversed
> polarity — polarity is handled inside the five-bar kinematics. Confirm this before relying on the
> "inner/outer" language elsewhere.

---

## Command Timeout Safety

`cmd_timeout` (default 0.5 s) protects against a stalled gait node. If no new command arrives within
that window the motor node:

1. Sets `_neutral_hold = true`
2. Sends an all-zero MIT frame (Kp = Kd = τ = 0), so the motor produces no torque
3. Logs a warning

Set `cmd_timeout: 0.0` to disable (not recommended for hardware runs). Note that on a legged robot,
zero gains mean the leg goes limp.

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

### CAN bandwidth

The driver answers every MIT command with a reply frame, so each motor costs **2 frames per control
cycle**. A standard-ID 8-byte CAN frame is 111 bits, up to 135 once bit stuffing kicks in. With 4
motors on each interface:

| `control_hz` | Bus load (best – worst) | |
|---|---|---|
| 500 | 44 % – 54 % | current setting |
| 700 | 62 % – 76 % | |
| 800 | 71 % – 86 % | node warns from ~740 Hz |
| 1000 | 89 % – 108 % | saturated; frames arrive late and uneven |

The motor node logs a warning at startup if `control_hz` puts the bus over 80 %. These figures assume
the 4/4 split above — moving all 8 motors onto one interface doubles them.

---

## Special Commands

Send a string to `/{joint}/special_cmd`:

| Command | Effect |
|---------|--------|
| `start` | Enter MIT mode (`0xFC`) — **required before any motion command** |
| `exit` | Exit MIT mode (`0xFD`) and release the motor |
| `zero` | Set the current position as zero (`0xFE`); goes limp first so the origin is never redefined under load |
| `clear` | Neutral hold — zero gains and torque until a new command arrives |

Example:
```bash
ros2 topic pub --once /motor0/special_cmd std_msgs/msg/String "data: 'start'"
```

---

## Tests

Pure-protocol and node-level command conversion are covered by unit tests that need neither ROS nor
a CAN bus:

```bash
python3 -m pytest actuation/cubemars_v2_ros/test/test_mit_protocol.py \
                  actuation/cubemars_v2_ros/test/test_motor_node_mit.py \
                  actuation/cubemars_v2_ros/test/test_servo_protocol.py
```
