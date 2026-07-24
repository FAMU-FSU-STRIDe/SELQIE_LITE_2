#!/usr/bin/env python3
"""
ROS2 Node for Cubemars Actuator Control (Servo Mode).

This module provides a ROS2 interface for controlling Cubemars (AK series)
motor actuators via CAN bus using the CubeMars **Servo Mode** protocol
(AK Series Module Driver Manual V1.0.18, section 5).

Unlike the MIT protocol, servo mode has **no Kp / Kd gains** -- the position
and velocity loops run inside the driver.  Each CAN frame commands exactly one
quantity (duty / current / speed / position).  Servo mode also uses different
units from MIT mode; the conversions (radians<->degrees, rad/s<->ERPM,
N.m<->amperes) are implemented in :mod:`cubemars_v2_ros.servo_protocol` so that
the ROS interface below stays in the SI units the rest of the SELQIE stack
expects (rad, rad/s, N.m at the output shaft).

The node publishes motor state data including position, velocity, torque,
current, temperature, and error codes, and subscribes to command topics for
motor control.
"""

import threading
import time

import can
import rclpy
from rclpy.node import Node
from actuation_msgs.msg import MotorCommand, MotorEstimate
from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState

from cubemars_v2_ros import servo_protocol as sp

# ===================== MOTOR SPECIFICATIONS AND CONSTANTS =====================

# Physical limits used to clamp setpoints *before* converting to servo units.
# V: velocity limits (rad/s), T: torque limits (Nm) at the output shaft.
# (Servo mode has no position gain limits -- Kp/Kd are not transmitted.)
LIMITS = {
    "AK10-9": dict(V_MIN=-50.0, V_MAX=50.0, T_MIN=-65.0, T_MAX=65.0),
    "AK60-6": dict(V_MIN=-45.0, V_MAX=45.0, T_MIN=-15.0, T_MAX=15.0),
    "AK70-10": dict(V_MIN=-50.0, V_MAX=50.0, T_MIN=-25.0, T_MAX=25.0),
    "AK80-6": dict(V_MIN=-76.0, V_MAX=76.0, T_MIN=-12.0, T_MAX=12.0),
    "AK80-9": dict(V_MIN=-50.0, V_MAX=50.0, T_MIN=-18.0, T_MAX=18.0),
    "AK80-64": dict(V_MIN=-8.0, V_MAX=8.0, T_MIN=-144.0, T_MAX=144.0),
    "AK80-8": dict(V_MIN=-37.5, V_MAX=37.5, T_MIN=-32.0, T_MAX=32.0),
    "AK40-10": dict(V_MIN=-45.5, V_MAX=45.5, T_MIN=-4.1, T_MAX=4.1),
}

# Motor-side (pre-gearbox) torque constants (Nm/A), as published in each
# motor's datasheet (e.g. AK40-10 KT = 0.056 Nm/A).  Used to convert between
# commanded output torque (Nm) and servo-mode phase current (A).
TORQUE_CONSTANTS = {
    "AK10-9": 0.198,  # Nm/A
    "AK70-10": 0.123,  # Nm/A
    "AK80-64": 0.136,  # Nm/A
    "AK40-10": 0.056,  # Nm/A
}

# Gearbox reduction ratio (output:motor), i.e. the number after the dash in
# the model name.  Servo-mode torque is measured at the output shaft, while the
# current command drives the rotor, so torque<->current needs both the
# motor-side Kt above and this ratio: current = output_torque / (Kt * ratio).
GEAR_RATIOS = {
    "AK10-9": 9,
    "AK60-6": 6,
    "AK70-10": 10,
    "AK80-6": 6,
    "AK80-9": 9,
    "AK80-64": 64,
    "AK80-8": 8,
    "AK40-10": 10,
}

# Rotor pole pairs (poles / 2).  Servo-mode *speed* is reported/commanded in
# ERPM (electrical RPM = mechanical_rotor_RPM * pole_pairs), so converting
# output-shaft rad/s <-> ERPM needs both the gear ratio and the pole pairs:
#   ERPM = rad_s * (60 / 2pi) * gear_ratio * pole_pairs
# These values only affect VELOCITY-mode scaling (position and torque modes are
# unaffected).  AK40-10 = 14 pole pairs (24 slots), confirmed from its
# datasheet.  The other models are best-guess (21 is common for the AK series)
# and are NOT datasheet-verified -- confirm and override with the ``pole_pairs``
# parameter before relying on VELOCITY-mode gaits with those motors.
POLE_PAIRS = {
    "AK10-9": 21,
    "AK60-6": 14,
    "AK70-10": 21,
    "AK80-6": 21,
    "AK80-9": 21,
    "AK80-64": 21,
    "AK80-8": 21,
    "AK40-10": 14,  # datasheet-verified (24 slots / 14 pole pairs)
}

DEFAULT_POLE_PAIRS = 21

# Motor error code mapping for human-readable messages (manual §5.2.1).
MOTOR_ERROR_CODES = {
    0: "No fault - Motor operating normally",
    1: "Motor over-temperature fault - Motor temperature exceeds safe operating limits",
    2: "Over-current fault - Current draw exceeds maximum allowable threshold",
    3: "Over-voltage fault - Input voltage exceeds maximum operating voltage",
    4: "Under-voltage fault - Input voltage below minimum operating voltage",
    5: "Encoder fault - Position feedback sensor malfunction or disconnection",
    6: "MOSFET over-temperature fault - Power transistor temperature exceeds safe limits",
    7: "Motor lock-up - Motor shaft mechanically blocked or seized",
}


def get_error_message(error_code):
    """Return a human-readable message for a motor error code."""
    return MOTOR_ERROR_CODES.get(error_code, f"Unknown error code: {error_code}")


# ===================== MOTOR NODE CLASS =====================


class MotorNode(Node):
    """
    ROS2 Node for controlling a Cubemars actuator via CAN bus in servo mode.

    Handles:
    - Servo-mode CAN communication with the motor (extended frames)
    - Publishing motor state (position, velocity, torque, current, temperature)
    - Receiving commands (generic MotorCommand and special commands)
    - Managing motor enable/disable and neutral-hold behaviour
    """

    def __init__(self):
        """Initialize the motor control node and its parameters."""
        super().__init__("motor_node")

        # ---- ROS Parameters ----
        self.declare_parameter("can_interface", "can0")  # CAN interface name
        self.declare_parameter("can_id", 1)  # CAN node ID for this motor
        self.declare_parameter("motor_type", "AK70-10")  # Motor model
        self.declare_parameter("control_hz", 20.0)  # Control loop frequency
        self.declare_parameter("joint_name", "joint")  # Name for this joint/motor
        self.declare_parameter("auto_start", False)  # Whether to auto-enable the motor
        self.declare_parameter("reverse_polarity", False)  # Reverse motor direction
        # pole_pairs / gear_ratio: 0 means "use the per-motor table default".
        # Only affect VELOCITY-mode ERPM scaling and torque<->current scaling.
        self.declare_parameter("pole_pairs", 0)
        self.declare_parameter("gear_ratio", 0.0)
        self.declare_parameter("cmd_timeout", 0.5)  # seconds before a stale cmd is cleared
        # POSITION-mode streaming. Selectable at runtime via the special commands
        # "pos" / "pos_spd" (the UI sets pos for all gaits and pos_spd for the
        # stand/ready hold).
        #  "pos_spd" (default): SET_POS_SPD with a trajectory-derived speed
        #            feed-forward. Smooth, and the right choice for the gentle
        #            stand/ready move and slow gaits. Its acceleration field is
        #            protocol-capped (~245 rad/s^2 at the AK40-10 output), so
        #            above ~1-1.5x gait frequency the motor cannot keep up and
        #            loses positional accuracy.
        #  "pos":    plain SET_POS. The driver drives to each streamed setpoint
        #            with the motor's full physical acceleration, so it tracks
        #            position accurately at every gait frequency. Pair it with a
        #            high control_hz so the setpoint stream is fine-grained
        #            (smooth) rather than a coarse slam-and-wait staircase.
        self.declare_parameter("position_mode", "pos_spd")
        # Acceleration limit (ERPM/s) for pos_spd streaming. Defaults to the
        # protocol maximum so acceleration is not the bottleneck when the gait
        # frequency is increased -- the velocity feed-forward (which scales with
        # frequency) then governs the speed, while the speed cap still prevents
        # the max-speed slam that caused ringing. Note: the SET_POS_SPD accel
        # field is protocol-capped at this value (~245 rad/s^2 at the output for
        # AK40-10), so very high gait frequencies will still saturate here; use
        # position_mode:"pos" for uncapped (but unshaped) speed.
        self.declare_parameter("pos_spd_accel", sp.POS_SPD_ACCEL_MAX)
        # Minimum approach speed (rad/s) for pos_spd streaming. A held/static
        # position setpoint (e.g. the "stand" pose) produces zero position change
        # between ticks, so the velocity feed-forward is zero; without a floor,
        # SET_POS_SPD with speed 0 would never move the motor to the target.
        self.declare_parameter("pos_spd_min_speed", 2.0)

        # Get parameters
        self.iface = self.get_parameter("can_interface").value
        self.can_id = int(self.get_parameter("can_id").value)
        self.motor_type = self.get_parameter("motor_type").value
        if self.motor_type not in LIMITS:
            self.get_logger().error(
                f"Unsupported motor type: {self.motor_type}. "
                f"Supported types: {list(LIMITS.keys())}"
            )
            raise ValueError(f"Unsupported motor type: {self.motor_type}")
        self.R = LIMITS[self.motor_type]  # Physical limits for this motor
        self.joint_name = self.get_parameter("joint_name").value
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.control_dt = 1.0 / self.control_hz  # Control period
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.reverse_polarity = bool(self.get_parameter("reverse_polarity").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.position_mode = str(self.get_parameter("position_mode").value).lower()
        if self.position_mode not in ("pos", "pos_spd"):
            self.get_logger().warn(
                f"Unknown position_mode '{self.position_mode}'; using 'pos_spd'"
            )
            self.position_mode = "pos_spd"
        self.pos_spd_accel = float(self.get_parameter("pos_spd_accel").value)
        self.pos_spd_min_speed = float(self.get_parameter("pos_spd_min_speed").value)

        # Servo-unit conversion constants (with per-motor fallbacks).
        pole_pairs = int(self.get_parameter("pole_pairs").value)
        self.pole_pairs = pole_pairs if pole_pairs > 0 else POLE_PAIRS.get(
            self.motor_type, DEFAULT_POLE_PAIRS
        )
        gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.gear_ratio = gear_ratio if gear_ratio > 0 else float(
            GEAR_RATIOS.get(self.motor_type, 1)
        )
        self.kt = TORQUE_CONSTANTS.get(self.motor_type)  # Nm/A, motor-side
        # Current limit (A) that maps to this motor's torque limit; capped at
        # the servo encoding maximum (±60 A).
        if self.kt:
            torque_current = abs(
                sp.torque_to_current(self.R["T_MAX"], self.kt, self.gear_ratio)
            )
            self.current_max_a = min(torque_current, sp.CURRENT_MAX_A)
        else:
            self.current_max_a = sp.CURRENT_MAX_A

        # Log parameters for debugging
        self.get_logger().info(
            f"""
            Motor: {self.joint_name}
            Can_id: {self.can_id}
            Motor type: {self.motor_type}
            Control Hz: {self.control_hz}
            Auto Start: {self.auto_start}
            Reverse Polarity: {self.reverse_polarity}
            Gear ratio: {self.gear_ratio}
            Pole pairs: {self.pole_pairs}
            Kt (Nm/A): {self.kt}
            Current limit (A): {self.current_max_a:.2f}
            Mode: SERVO (no Kp/Kd)
            """
        )

        if not self.kt:
            self.get_logger().warn(
                f"No torque constant for {self.motor_type}; TORQUE-mode commands and "
                f"torque feedback will be zero. Add it to TORQUE_CONSTANTS to enable."
            )

        self.node_id = self.can_id & 0xFF  # Servo source node ID (low byte of frames)
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)
            try:
                # Servo feedback frames are extended (29-bit) with the motor's
                # node ID in the low byte; match those.
                self.bus.set_filters(
                    [{"can_id": self.node_id, "can_mask": 0xFF, "extended": True}]
                )
            except Exception:
                pass  # Some interfaces don't support filtering
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize CAN bus on interface '{self.iface}': {e}"
            )
            raise

        # ---- ROS Publishers and Subscribers ----
        self.pub_err = self.create_publisher(
            String, f"/{self.joint_name}/error_code", 10
        )
        self.pub_state = self.create_publisher(
            MotorState, f"/{self.joint_name}/motor_state", 10
        )
        self.pub_estimate = self.create_publisher(
            MotorEstimate, f"/{self.joint_name}/estimate", 10
        )

        # Subscribers
        # Generic actuation command used by the leg kinematics stack.
        self.sub_motor_command = self.create_subscription(
            MotorCommand, f"/{self.joint_name}/command", self.on_motor_command, 10
        )
        # Raw servo command (bench testing): [mode, value].
        #   mode 3 = position (rad), 2 = velocity (rad/s), 1 = torque (Nm),
        #   0 = duty [-1, 1].  A legacy 5-element MIT tuple [p, v, kp, kd, t] is
        #   also accepted -- the gains are ignored (servo mode has no Kp/Kd).
        self.sub_raw_cmd = self.create_subscription(
            Float64MultiArray, f"/{self.joint_name}/servo_cmd", self.on_raw_cmd, 10
        )
        # Special commands: start, exit, zero, clear
        self.sub_special = self.create_subscription(
            String, f"/{self.joint_name}/special_cmd", self.on_special, 10
        )

        # ---- Internal State ----
        self._lock = threading.Lock()  # Thread safety for command access
        # Cached command: (control_mode, value_a, value_b). value_a/value_b are
        # already in servo-facing SI units (rad, rad/s, Nm, or duty).
        self.cmd_mode = MotorCommand.CONTROL_MODE_POSITION
        self.cmd_pos = 0.0  # rad
        self.cmd_vel = 0.0  # rad/s
        self.cmd_torq = 0.0  # Nm
        self.cmd_duty = 0.0  # [-1, 1]
        self._started = False  # Whether the motor is enabled
        self._neutral_hold = True  # When True, release torque regardless of cmd cache
        self._last_cmd_time = None  # Monotonic timestamp of last received command

        # Velocity feed-forward state for pos_spd streaming: the last
        # polarity-applied position (rad) actually commanded, used to derive the
        # trajectory speed from consecutive setpoints. Reset whenever the motor
        # stops being driven so the first move after a pause is not a huge step.
        self._last_ff_pos_rad = None
        self._speed_cap_erpm = abs(
            sp.rads_to_erpm(self.R["V_MAX"], self.gear_ratio, self.pole_pairs)
        )
        self._min_speed_erpm = abs(
            sp.rads_to_erpm(self.pos_spd_min_speed, self.gear_ratio, self.pole_pairs)
        )

        # Absolute position tracking (unwrapping), in radians at the output shaft.
        self._last_pos_rad = None
        self._p_abs = 0.0
        self._span_rad = sp.deg_to_rad(sp.FB_POS_SPAN_DEG)  # feedback wrap span

        # ---- Timers ----
        self.create_timer(self.control_dt, self._tick_control)

        # ---- CAN Receiver Thread ----
        self._stop = False
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

        # Optional auto-enable on initialization
        if self.auto_start and not self._started:
            self._started = True
            self.get_logger().info(f"Auto-starting motor {self.joint_name}")

    # ---- Command Callbacks ----
    def on_motor_command(self, msg):
        """
        Handle the generic MotorCommand used by the SELQIE leg-control stack.

        MotorCommand carries setpoints in output-shaft SI units (rad, rad/s,
        Nm).  The control mode selects which single servo frame is sent by the
        control loop:
          - POSITION -> servo position mode (degrees)
          - VELOCITY -> servo speed mode (ERPM)
          - TORQUE   -> servo current mode (amperes)
        """
        p = float(msg.pos_setpoint)
        v = float(msg.vel_setpoint)
        t = float(msg.torq_setpoint)

        if msg.control_mode not in (
            MotorCommand.CONTROL_MODE_POSITION,
            MotorCommand.CONTROL_MODE_VELOCITY,
            MotorCommand.CONTROL_MODE_TORQUE,
        ):
            self.get_logger().warn(
                f"Unsupported MotorCommand control_mode {msg.control_mode}; command ignored"
            )
            return

        with self._lock:
            self.cmd_mode = msg.control_mode
            self.cmd_pos = p
            self.cmd_vel = v
            self.cmd_torq = t
            self.cmd_duty = 0.0
            self._neutral_hold = False  # New command cancels any previous "clear" hold
            self._last_cmd_time = time.monotonic()

    def on_raw_cmd(self, msg):
        """
        Handle a raw servo command for bench testing.

        Accepts ``[mode, value]`` where mode is 3=position (rad), 2=velocity
        (rad/s), 1=torque (Nm), 0=duty.  For backward compatibility a 5-element
        MIT tuple ``[p, v, kp, kd, t]`` is also accepted; the Kp/Kd gains are
        ignored (servo mode has no gains) and the command is treated as a
        position move to ``p``.
        """
        data = list(map(float, msg.data))
        if len(data) == 5:
            # Legacy MIT tuple [p, v, kp, kd, t]: gains have no meaning in servo
            # mode, so drive to the requested position.
            mode = float(MotorCommand.CONTROL_MODE_POSITION)
            value = data[0]
        elif len(data) == 2:
            mode, value = data[0], data[1]
        else:
            self.get_logger().warn(
                "servo_cmd expects [mode, value] (or legacy [p, v, kp, kd, t])"
            )
            return

        mode = int(mode)
        with self._lock:
            self.cmd_pos = 0.0
            self.cmd_vel = 0.0
            self.cmd_torq = 0.0
            self.cmd_duty = 0.0
            if mode == MotorCommand.CONTROL_MODE_POSITION:
                self.cmd_mode = MotorCommand.CONTROL_MODE_POSITION
                self.cmd_pos = value
            elif mode == MotorCommand.CONTROL_MODE_VELOCITY:
                self.cmd_mode = MotorCommand.CONTROL_MODE_VELOCITY
                self.cmd_vel = value
            elif mode == MotorCommand.CONTROL_MODE_TORQUE:
                self.cmd_mode = MotorCommand.CONTROL_MODE_TORQUE
                self.cmd_torq = value
            elif mode == 0:
                self.cmd_mode = 0  # duty
                self.cmd_duty = value
            else:
                self.get_logger().warn(f"servo_cmd unknown mode {mode}; ignored")
                return
            self._neutral_hold = False
            self._last_cmd_time = time.monotonic()

    def on_special(self, msg):
        """Handle a special command: start | exit | zero | clear | pos | pos_spd."""
        m = msg.data.strip().lower()

        if m in ("pos", "pos_spd"):
            # Switch the POSITION streaming submode at runtime. The UI selects
            # pos (accurate) for all gaits and pos_spd (smooth) for the
            # stand/ready hold.
            if m != self.position_mode:
                with self._lock:
                    self.position_mode = m
                    self._last_ff_pos_rad = None  # restart feed-forward cleanly
                self.get_logger().info(
                    f"Position mode -> {m} for motor {self.joint_name}"
                )
            return

        if m == "start":
            # Enable command output.
            if not self._started:
                self._started = True
                self.get_logger().info(f"Starting motor {self.joint_name}")

        elif m == "exit":
            # Release torque and stop driving the motor.
            self._send_current(0.0)
            with self._lock:
                self._neutral_hold = True
            self._started = False
            self.get_logger().info(f"Exiting (releasing) motor {self.joint_name}")

        elif m == "zero":
            # Set the current position as the (temporary) origin.
            self._send_origin(0)
            with self._lock:
                # Reset unwrapping so abs_position restarts from zero.
                self._last_pos_rad = None
                self._p_abs = 0.0
            self.get_logger().info(f"Zeroing encoder for motor {self.joint_name}")

        elif m == "clear":
            # Neutral hold: release torque (zero current) until a new command.
            with self._lock:
                self._neutral_hold = True
                self.cmd_pos = 0.0
                self.cmd_vel = 0.0
                self.cmd_torq = 0.0
                self.cmd_duty = 0.0
            self._send_current(0.0)
            self.get_logger().info(f"Clearing commands for motor {self.joint_name}")

        else:
            self.get_logger().warn(
                f"Unknown special command: '{m}'. "
                f"Valid options: start|exit|zero|clear|pos|pos_spd"
            )

    # ---- Control Loop ----
    def _tick_control(self):
        """Periodic control loop that sends one servo frame to the motor."""
        if self.cmd_timeout > 0:
            with self._lock:
                if self._last_cmd_time is not None:
                    elapsed = time.monotonic() - self._last_cmd_time
                    if elapsed > self.cmd_timeout:
                        self._neutral_hold = True
                        self._last_cmd_time = None
                        self.get_logger().warn(
                            f"No command for {elapsed:.2f}s on {self.joint_name}, clearing"
                        )

        with self._lock:
            neutral = self._neutral_hold
            mode = self.cmd_mode
            pos = self.cmd_pos
            vel = self.cmd_vel
            torq = self.cmd_torq
            duty = self.cmd_duty

        # Do not drive the motor until it has been started.  Neutral hold and
        # the un-started state both release torque (zero current), matching the
        # MIT node's "all zeros" behaviour.
        if not self._started or neutral:
            self._last_ff_pos_rad = None  # next position move restarts the feed-forward
            self._send_current(0.0)
            return

        if mode == MotorCommand.CONTROL_MODE_POSITION and self.position_mode == "pos_spd":
            self._send_position_velocity(pos)
            return
        # Any other mode abandons the position feed-forward history.
        self._last_ff_pos_rad = None
        if mode == MotorCommand.CONTROL_MODE_POSITION:
            self._send_position(pos)
        elif mode == MotorCommand.CONTROL_MODE_VELOCITY:
            self._send_velocity(vel)
        elif mode == MotorCommand.CONTROL_MODE_TORQUE:
            self._send_torque(torq)
        elif mode == 0:
            self._send_duty(duty)
        else:
            self._send_current(0.0)

    # ---- Servo frame senders (all inputs are output-shaft SI units) ----
    def _send(self, can_id, data):
        """Transmit an extended (29-bit) CAN frame."""
        try:
            self.bus.send(
                can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
            )
        except can.CanError:
            self.get_logger().error(
                f"Failed to send CAN frame to motor {self.joint_name}"
            )

    def _send_position(self, pos_rad):
        """Command an output-shaft position (rad) via plain position mode."""
        if self.reverse_polarity:
            pos_rad = -pos_rad
        can_id, data = sp.pack_pos(self.node_id, sp.rad_to_deg(pos_rad))
        self._send(can_id, data)

    def _send_position_velocity(self, pos_rad):
        """Command an output-shaft position (rad) with a velocity feed-forward.

        Streams SET_POS_SPD frames whose speed is derived from the change in the
        commanded position over one control period, so the motor tracks the
        trajectory at its own speed instead of slamming to each setpoint at the
        motor's maximum speed (which rings on wide/fast gaits such as swim).
        """
        if self.reverse_polarity:
            pos_rad = -pos_rad

        if self._last_ff_pos_rad is None:
            # First move of a run: no history yet, so there is no feed-forward.
            speed_erpm = 0.0
        else:
            vel_rads = (pos_rad - self._last_ff_pos_rad) * self.control_hz
            speed_erpm = abs(
                sp.rads_to_erpm(vel_rads, self.gear_ratio, self.pole_pairs)
            )
        self._last_ff_pos_rad = pos_rad

        # Floor the approach speed so a held/static setpoint (e.g. "stand") and
        # the first move still travel to the target, then cap it at V_MAX.
        speed_erpm = min(max(speed_erpm, self._min_speed_erpm), self._speed_cap_erpm)

        can_id, data = sp.pack_pos_spd(
            self.node_id, sp.rad_to_deg(pos_rad), speed_erpm, self.pos_spd_accel
        )
        self._send(can_id, data)

    def _send_velocity(self, vel_rads):
        """Command an output-shaft velocity (rad/s)."""
        vel_rads = sp.clamp(vel_rads, self.R["V_MIN"], self.R["V_MAX"])
        if self.reverse_polarity:
            vel_rads = -vel_rads
        erpm = sp.rads_to_erpm(vel_rads, self.gear_ratio, self.pole_pairs)
        can_id, data = sp.pack_rpm(self.node_id, erpm)
        self._send(can_id, data)

    def _send_torque(self, torque_nm):
        """Command an output-shaft torque (Nm) via the current loop."""
        torque_nm = sp.clamp(torque_nm, self.R["T_MIN"], self.R["T_MAX"])
        if self.reverse_polarity:
            torque_nm = -torque_nm
        if self.kt:
            current_a = sp.torque_to_current(torque_nm, self.kt, self.gear_ratio)
        else:
            current_a = 0.0  # Cannot map torque without a torque constant.
        self._send_current(current_a)

    def _send_current(self, current_a):
        """Command a raw phase current (A)."""
        can_id, data = sp.pack_current(self.node_id, current_a, self.current_max_a)
        self._send(can_id, data)

    def _send_duty(self, duty):
        """Command a raw duty cycle [-1, 1]."""
        if self.reverse_polarity:
            duty = -duty
        can_id, data = sp.pack_duty(self.node_id, duty)
        self._send(can_id, data)

    def _send_origin(self, mode):
        """Send a set-origin frame (0 = temporary, 1 = permanent)."""
        can_id, data = sp.pack_origin(self.node_id, mode)
        self._send(can_id, data)

    def _rx_loop(self):
        """Background thread that receives and processes servo status frames."""
        while not self._stop:
            rx = self.bus.recv(timeout=0.1)
            if not rx or not rx.is_extended_id or len(rx.data) != 8:
                continue

            packet_id, node_id = sp.parse_status_id(rx.arbitration_id)
            # Only accept status (0x29) frames addressed from our motor.
            if node_id != self.node_id or packet_id != sp.CAN_PACKET_STATUS:
                continue

            parsed = sp.parse_status(rx.data)
            if not parsed:
                continue
            pos_deg, spd_erpm, current_a, temp_c, err = parsed

            # Convert servo units -> output-shaft SI units.
            pos_rad = sp.deg_to_rad(pos_deg)
            vel_rads = sp.erpm_to_rads(spd_erpm, self.gear_ratio, self.pole_pairs)
            torque_nm = (
                sp.current_to_torque(current_a, self.kt, self.gear_ratio)
                if self.kt
                else 0.0
            )

            # Apply reverse polarity to received values if configured.
            if self.reverse_polarity:
                pos_rad = -pos_rad
                vel_rads = -vel_rads
                current_a = -current_a
                torque_nm = -torque_nm

            # ---- Position unwrapping (continuous rotation tracking) ----
            if self._last_pos_rad is None:
                self._p_abs = pos_rad
            else:
                dp = pos_rad - self._last_pos_rad
                if dp > 0.5 * self._span_rad:
                    dp -= self._span_rad
                elif dp < -0.5 * self._span_rad:
                    dp += self._span_rad
                self._p_abs += dp
            self._last_pos_rad = pos_rad

            # ---- Publish error code with human-readable message ----
            error_code = String()
            error_code.data = f"Error Code {err}: {get_error_message(err)}"
            self.pub_err.publish(error_code)

            # ---- Publish complete motor state ----
            ms = MotorState()
            ms.name = self.joint_name
            ms.position = pos_rad  # rad (raw, wrapped within feedback range)
            ms.abs_position = self._p_abs  # rad (unwrapped)
            ms.velocity = vel_rads  # rad/s
            ms.torque = torque_nm  # Nm (output shaft)
            ms.current = current_a  # A (motor phase current)
            ms.temperature = int(temp_c)  # degC
            self.pub_state.publish(ms)

            # ---- Publish MotorEstimate for leg kinematics (unwrapped angle) ----
            me = MotorEstimate()
            me.pos_estimate = self._p_abs
            me.vel_estimate = vel_rads
            me.torq_estimate = torque_nm
            self.pub_estimate.publish(me)

    def destroy_node(self):
        """Clean up resources when the node is shutting down."""
        # Release torque so the motor does not hold a stale command.
        self._send_current(0.0)
        self._stop = True

        try:
            self._rx.join(timeout=0.3)
        except Exception:
            pass

        try:
            if hasattr(self.bus, "shutdown"):
                self.bus.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    """Main entry point for the motor node."""
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
