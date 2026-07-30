#!/usr/bin/env python3
"""
ROS 2 node for CAN control of CubeMars AK40-10 V2.0 actuators in **MIT mode**.

Implements the MIT communication protocol from the *AK Series Module Driver
Manual* V1.0.18 §5.3. The wire format, special codes and unit conversions live
in :mod:`cubemars_v2_ros.mit_protocol`; this file is the ROS plumbing, the
gain handling and the safety behaviour.

Control law
-----------
The driver runs this onboard every cycle (manual §5.3 block diagram)::

    torque = Kp * (p_des - p_meas) + Kd * (v_des - v_meas) + t_ff

clamped to the motor's torque limit, then fed to the FOC current loop. Unlike
servo mode, **Kp and Kd travel in every CAN frame**, so they can be retuned live
from ROS without an R-LINK USB session.

Adjusting gains
---------------
Three ways, in increasing order of convenience:

1. Edit ``actuation_bringup/config/mit_gains.yaml`` -- one file, all motors.
2. Pass ``position_kp:=... position_kd:=...`` to the launch file.
3. Publish to ``/{joint}/set_gains`` (``Float64MultiArray`` ``[kp, kd]``) to
   change them live, mid-run. Current values are echoed on ``/{joint}/gains``.

ROS interface
-------------
Publishers
    ``/{joint}/motor_state``  full feedback (MotorState)
    ``/{joint}/estimate``     position/velocity/torque for the kinematics stack
    ``/{joint}/error_code``   fault string
    ``/{joint}/gains``        currently active ``[kp, kd]``
Subscribers
    ``/{joint}/command``      MotorCommand (rad, rad/s, N.m)
    ``/{joint}/mit_cmd``      raw ``[p, v, kp, kd, t]`` for bench testing
    ``/{joint}/set_gains``    ``[kp, kd]`` live gain override
    ``/{joint}/special_cmd``  ``start`` | ``exit`` | ``zero`` | ``clear``
"""

import threading
import time

import can
import rclpy
from rclpy.node import Node
from actuation_msgs.msg import MotorCommand, MotorEstimate
from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState

from cubemars_v2_ros import mit_protocol as mit

# ===================== MOTOR SPECIFICATIONS =====================
#
# P/V/T are the ranges the MIT frame quantizes against, so they MUST match the
# driver's own configuration -- if they disagree, the motor decodes different
# values than were sent. Kp/Kd ranges are fixed by the protocol (§5.3).
#
# AK40-10 figures are datasheet-verified: 24 slots / 14 pole pairs, KT
# 0.056 N.m/A, 10:1 reduction, 4.1 N.m peak torque (= 7.3 A peak current), and
# 435 rpm no-load = 45.5 rad/s at the output.

_KP_RANGE = dict(KP_MIN=0.0, KP_MAX=500.0)
_KD_RANGE = dict(KD_MIN=0.0, KD_MAX=5.0)

LIMITS = {
    "AK40-10": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-45.5, V_MAX=45.5,
                    T_MIN=-4.1, T_MAX=4.1, **_KP_RANGE, **_KD_RANGE),
    "AK10-9": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-50.0, V_MAX=50.0,
                   T_MIN=-65.0, T_MAX=65.0, **_KP_RANGE, **_KD_RANGE),
    "AK60-6": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-45.0, V_MAX=45.0,
                   T_MIN=-15.0, T_MAX=15.0, **_KP_RANGE, **_KD_RANGE),
    "AK70-10": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-50.0, V_MAX=50.0,
                    T_MIN=-25.0, T_MAX=25.0, **_KP_RANGE, **_KD_RANGE),
    "AK80-6": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-76.0, V_MAX=76.0,
                   T_MIN=-12.0, T_MAX=12.0, **_KP_RANGE, **_KD_RANGE),
    "AK80-8": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-37.5, V_MAX=37.5,
                   T_MIN=-32.0, T_MAX=32.0, **_KP_RANGE, **_KD_RANGE),
    "AK80-9": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-50.0, V_MAX=50.0,
                   T_MIN=-18.0, T_MAX=18.0, **_KP_RANGE, **_KD_RANGE),
    "AK80-64": dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-8.0, V_MAX=8.0,
                    T_MIN=-144.0, T_MAX=144.0, **_KP_RANGE, **_KD_RANGE),
}

# Motor-side torque constants (N.m/A) and gear reductions, used only to report
# phase current alongside torque. MIT mode commands torque directly.
TORQUE_CONSTANTS = {"AK40-10": 0.056, "AK10-9": 0.16,
                    "AK70-10": 0.123, "AK80-64": 0.136}
GEAR_RATIOS = {"AK40-10": 10, "AK10-9": 9, "AK60-6": 6, "AK70-10": 10,
               "AK80-6": 6, "AK80-8": 8, "AK80-9": 9, "AK80-64": 64}


class MotorNode(Node):
    """Drives one CubeMars actuator over CAN using the MIT protocol."""

    def __init__(self):
        """Set up parameters, CAN, ROS interface and the control timer."""
        super().__init__("motor_node")

        # ---- Parameters ----
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("can_id", 1)
        self.declare_parameter("motor_type", "AK40-10")
        self.declare_parameter("control_hz", 500.0)
        self.declare_parameter("joint_name", "joint")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("reverse_polarity", False)
        self.declare_parameter("cmd_timeout", 0.5)

        # Gains. Defaults are deliberately soft: the AK40-10 peaks at 4.1 N.m,
        # so Kp=20 saturates on a 0.2 rad error. See mit_gains.yaml.
        self.declare_parameter("position_kp", 5.0)
        self.declare_parameter("position_kd", 0.4)
        self.declare_parameter("velocity_kd", 0.5)
        self.declare_parameter("torque_limit_scale", 1.0)

        self.iface = self.get_parameter("can_interface").value
        self.can_id = int(self.get_parameter("can_id").value)
        self.motor_type = self.get_parameter("motor_type").value
        if self.motor_type not in LIMITS:
            raise ValueError(
                f"Unsupported motor type {self.motor_type}; "
                f"supported: {sorted(LIMITS)}")

        self.joint_name = self.get_parameter("joint_name").value
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.reverse_polarity = bool(self.get_parameter("reverse_polarity").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        # Apply the torque-limit scale to the packing range, so a reduced limit
        # also improves the 12-bit torque resolution over the range in use.
        self.R = dict(LIMITS[self.motor_type])
        scale = max(0.0, min(1.0, float(self.get_parameter("torque_limit_scale").value)))
        if scale < 1.0:
            self.R["T_MIN"] *= scale
            self.R["T_MAX"] *= scale

        self.kt = TORQUE_CONSTANTS.get(self.motor_type)
        self.gear_ratio = GEAR_RATIOS.get(self.motor_type, 1)

        self._gain_lock = threading.Lock()
        self.position_kp = float(self.get_parameter("position_kp").value)
        self.position_kd = float(self.get_parameter("position_kd").value)
        self.velocity_kd = float(self.get_parameter("velocity_kd").value)

        self.get_logger().info(
            f"\n  MIT mode | {self.joint_name} | {self.motor_type} | can_id={self.can_id}"
            f"\n  interface={self.iface} @ {self.control_hz:.0f} Hz"
            f"\n  gains: position kp={self.position_kp} kd={self.position_kd}, "
            f"velocity kd={self.velocity_kd}"
            f"\n  torque limit: +/-{self.R['T_MAX']:.2f} Nm (scale {scale:.2f})"
            f"\n  position resolution: {mit.position_resolution(self.R)*1e3:.3f} mrad"
            f"\n  tune live:  ros2 topic pub --once /{self.joint_name}/set_gains "
            f"std_msgs/msg/Float64MultiArray \"{{data: [kp, kd]}}\"")

        # ---- CAN ----
        # MIT mode uses STANDARD (11-bit) IDs: commands go to the motor ID,
        # replies come back on 0x00 + drive ID (§5.3).
        self.arb_id = self.can_id & 0x7FF
        self.reply_id = (mit.REPLY_ID_BASE + self.can_id) & 0x7FF
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)
            try:
                self.bus.set_filters(
                    [{"can_id": self.reply_id, "can_mask": 0x7FF, "extended": False}])
            except Exception:
                pass  # not all interfaces support filtering
        except Exception as e:
            self.get_logger().error(f"CAN init failed on '{self.iface}': {e}")
            raise

        # ---- ROS interface ----
        j = self.joint_name
        self.pub_state = self.create_publisher(MotorState, f"/{j}/motor_state", 10)
        self.pub_estimate = self.create_publisher(MotorEstimate, f"/{j}/estimate", 10)
        self.pub_err = self.create_publisher(String, f"/{j}/error_code", 10)
        self.pub_gains = self.create_publisher(Float64MultiArray, f"/{j}/gains", 10)

        self.create_subscription(MotorCommand, f"/{j}/command", self.on_command, 10)
        self.create_subscription(Float64MultiArray, f"/{j}/mit_cmd", self.on_mit_cmd, 10)
        self.create_subscription(Float64MultiArray, f"/{j}/set_gains", self.on_set_gains, 10)
        self.create_subscription(String, f"/{j}/special_cmd", self.on_special, 10)

        # ---- State ----
        self._lock = threading.Lock()
        # Cached MIT command: position, velocity, kp, kd, torque.
        self._cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._started = False
        self._neutral_hold = True
        self._last_cmd_time = None

        # Unwrapped absolute position, for joints that pass +/-12.5 rad.
        self._last_pos = None
        self._pos_abs = 0.0
        self._pos_span = self.R["P_MAX"] - self.R["P_MIN"]

        self.create_timer(1.0 / self.control_hz, self._tick)
        # Republish the active gains at a low rate so a client that connects
        # after startup -- the terminal, or `ros2 topic echo` -- can always read
        # the current values instead of having to wait for the next change.
        self.create_timer(1.0, self._publish_gains)

        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        if self.auto_start:
            self._enter_mit_mode()

    # ------------------------------------------------------------------ #
    # Gains                                                              #
    # ------------------------------------------------------------------ #
    def on_set_gains(self, msg):
        """Live gain override: ``[kp, kd]``, or ``[kp, kd, velocity_kd]``."""
        d = list(map(float, msg.data))
        if len(d) not in (2, 3):
            self.get_logger().warn("set_gains expects [kp, kd] or [kp, kd, velocity_kd]")
            return

        kp = mit.clamp(d[0], self.R["KP_MIN"], self.R["KP_MAX"])
        kd = mit.clamp(d[1], self.R["KD_MIN"], self.R["KD_MAX"])
        if (kp, kd) != (d[0], d[1]):
            self.get_logger().warn(
                f"gains clipped to protocol range: kp<=({self.R['KP_MAX']}), "
                f"kd<=({self.R['KD_MAX']})")

        with self._gain_lock:
            self.position_kp = kp
            self.position_kd = kd
            if len(d) == 3:
                self.velocity_kd = mit.clamp(d[2], self.R["KD_MIN"], self.R["KD_MAX"])

        self.get_logger().info(f"gains -> kp={kp:.3f} kd={kd:.3f}")
        self._publish_gains()

    def _publish_gains(self):
        """Echo the active gains so they can be inspected with `topic echo`."""
        msg = Float64MultiArray()
        with self._gain_lock:
            msg.data = [self.position_kp, self.position_kd, self.velocity_kd]
        self.pub_gains.publish(msg)

    # ------------------------------------------------------------------ #
    # Commands                                                           #
    # ------------------------------------------------------------------ #
    def on_command(self, msg):
        """Map a MotorCommand onto the MIT 5-tuple using the active gains."""
        p = float(msg.pos_setpoint)
        v = float(msg.vel_setpoint)
        t = float(msg.torq_setpoint)

        with self._gain_lock:
            pos_kp, pos_kd, vel_kd = self.position_kp, self.position_kd, self.velocity_kd

        if msg.control_mode == MotorCommand.CONTROL_MODE_POSITION:
            kp, kd = pos_kp, pos_kd
        elif msg.control_mode == MotorCommand.CONTROL_MODE_VELOCITY:
            # No position target to hold, so stiffness is zero and Kd tracks the
            # velocity setpoint.
            p, kp, kd = 0.0, 0.0, vel_kd
        elif msg.control_mode == MotorCommand.CONTROL_MODE_TORQUE:
            # Pure feed-forward: gains off so torque passes straight through.
            p, v, kp, kd = 0.0, 0.0, 0.0, 0.0
        else:
            self.get_logger().warn(f"unsupported control_mode {msg.control_mode}")
            return

        self._set_cmd([p, v, kp, kd, t])

    def on_mit_cmd(self, msg):
        """Raw bench command: ``[p, v, kp, kd, t]``, gains included."""
        d = list(map(float, msg.data))
        if len(d) != 5:
            self.get_logger().warn("mit_cmd expects [p, v, kp, kd, t]")
            return
        self._set_cmd(d)

    def _set_cmd(self, cmd):
        with self._lock:
            self._cmd = cmd
            self._neutral_hold = False
            self._last_cmd_time = time.monotonic()

    def on_special(self, msg):
        """``start`` | ``exit`` | ``zero`` | ``clear``."""
        m = msg.data.strip().lower()

        if m == "start":
            self._enter_mit_mode()
        elif m == "exit":
            with self._lock:
                self._neutral_hold = True
            self._send(mit.special_frame(mit.MIT_EXIT_MODE))
            self._started = False
            self.get_logger().info("exited MIT mode (motor released)")
        elif m == "zero":
            # Zeroing under load would redefine the origin mid-motion, so make
            # the motor limp first and reset the unwrap accumulator with it.
            self._go_neutral()
            self._send(mit.special_frame(mit.MIT_ZERO_POSITION))
            with self._lock:
                self._last_pos = None
                self._pos_abs = 0.0
            self.get_logger().info("position zeroed")
        elif m == "clear":
            self._go_neutral()
            self.get_logger().info("commands cleared (holding neutral)")
        else:
            self.get_logger().warn(
                f"unknown special command '{m}'; use start|exit|zero|clear")

    def _enter_mit_mode(self):
        """Enter MIT control mode -- required before any motion command."""
        if self._started:
            return
        self._send(mit.special_frame(mit.MIT_ENTER_MODE))
        self._started = True
        self._go_neutral()
        self.get_logger().info("entered MIT mode")
        self._publish_gains()

    def _go_neutral(self):
        """Hold an all-zero command: no stiffness, no damping, no torque."""
        with self._lock:
            self._cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
            self._neutral_hold = True

    # ------------------------------------------------------------------ #
    # Control loop                                                       #
    # ------------------------------------------------------------------ #
    def _tick(self):
        """Send one MIT frame at the configured rate."""
        if self.cmd_timeout > 0:
            with self._lock:
                if self._last_cmd_time is not None:
                    elapsed = time.monotonic() - self._last_cmd_time
                    if elapsed > self.cmd_timeout:
                        self._cmd = [0.0, 0.0, 0.0, 0.0, 0.0]
                        self._neutral_hold = True
                        self._last_cmd_time = None
                        self.get_logger().warn(
                            f"no command for {elapsed:.2f}s; holding neutral")

        with self._lock:
            p, v, kp, kd, t = ([0.0] * 5) if self._neutral_hold else list(self._cmd)

        if not self._started:
            return  # never drive before MIT mode has been entered

        if self.reverse_polarity:
            p, v, t = -p, -v, -t

        self._send(mit.pack_command(p, v, kp, kd, t, self.R))

    def _send(self, data):
        """Transmit a standard-ID CAN frame to this motor."""
        try:
            self.bus.send(can.Message(arbitration_id=self.arb_id, data=data,
                                      is_extended_id=False))
        except can.CanError:
            self.get_logger().error("CAN send failed")

    # ------------------------------------------------------------------ #
    # Feedback                                                           #
    # ------------------------------------------------------------------ #
    def _rx_loop(self):
        """Receive and republish MIT reply frames."""
        while not self._stop:
            rx = self.bus.recv(timeout=0.1)
            if not rx or rx.is_extended_id or len(rx.data) != 8:
                continue
            if rx.arbitration_id != self.reply_id:
                continue

            parsed = mit.parse_reply(rx.data, self.R)
            if not parsed:
                continue
            drive_id, pos, vel, torque, temp, err = parsed
            if drive_id != (self.can_id & 0xFF):
                continue

            if self.reverse_polarity:
                pos, vel, torque = -pos, -vel, -torque

            # Unwrap position so a joint crossing +/-12.5 rad stays continuous.
            with self._lock:
                if self._last_pos is None:
                    self._pos_abs = pos
                else:
                    dp = pos - self._last_pos
                    if dp > 0.5 * self._pos_span:
                        dp -= self._pos_span
                    elif dp < -0.5 * self._pos_span:
                        dp += self._pos_span
                    self._pos_abs += dp
                self._last_pos = pos
                pos_abs = self._pos_abs

            err_msg = String()
            err_msg.data = f"Error Code {err}: {mit.error_message(err)}"
            self.pub_err.publish(err_msg)

            state = MotorState()
            state.name = self.joint_name
            state.position = pos
            state.abs_position = pos_abs
            state.velocity = vel
            state.torque = torque
            state.current = (mit.torque_to_current(torque, self.kt, self.gear_ratio)
                             if self.kt else 0.0)
            state.temperature = int(temp)
            self.pub_state.publish(state)

            est = MotorEstimate()
            est.pos_estimate = pos_abs
            est.vel_estimate = vel
            est.torq_estimate = torque
            self.pub_estimate.publish(est)

    def destroy_node(self):
        """Release the motor and shut the CAN interface down cleanly."""
        try:
            self._send(mit.special_frame(mit.MIT_EXIT_MODE))
        except Exception:
            pass
        self._stop = True
        try:
            self._rx_thread.join(timeout=0.3)
        except Exception:
            pass
        try:
            if hasattr(self.bus, "shutdown"):
                self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
