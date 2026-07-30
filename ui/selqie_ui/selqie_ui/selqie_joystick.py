#!/usr/bin/env python3
"""
SELQIE joystick teleoperation node.

Run this INSTEAD of ``selqie_terminal`` to drive the robot with a game
controller::

    ros2 run joy joy_node            # publishes /joy on the client device
    ros2 run selqie_ui selqie_joystick

It subscribes to ``joy`` (sensor_msgs/Joy) and reuses the same
``selqie_python.SELQIE`` framework the terminal uses, so it publishes the exact
same ``gait`` / ``cmd_vel`` topics and issues the same motor special commands --
nothing new in the pipeline, just a controller front-end.

Controls (Xbox layout defaults; every button/axis index is a ROS parameter):

    Motor state
      Start  -> ready   (enable motors, hold a stand pose)
      Back   -> idle    (release motors)
      L3     -> zero    (set encoder origin; ONLY allowed while idle)

    Gait select (motion only happens while the deadman is held)
      A -> walk    B -> swim    X -> stand    Y -> jump    LB -> sink

    Motion (hold RB = deadman to enable; release = stand still)
      Left stick Y  -> forward / reverse   (lin_x)
      Right stick X -> steer               (ang_z, walk)
      Right stick Y -> up / down           (lin_z, swim/jump)
      D-pad up/down -> speed governor (scales the max speed)

Safety (to avoid weird motor behavior):
  * Deadman -- motion is commanded only while the deadman button is held.
    Releasing it, a joy dropout, or a controller disconnect commands a stand
    hold with zero velocity.
  * Stick deadzone rejects centering drift.
  * ``zero`` is refused unless the motors are idle, so the encoder origin is
    never redefined under load (which would jerk every joint).
  * ``ready`` always enters a zero-velocity stand hold, so enabling never
    lurches into a stale gait.
"""

import time

import rclpy

from sensor_msgs.msg import Joy
from selqie_python.selqie import SELQIE, QOS_FAST

# Gaits that translate/steer from the sticks vs. the ones that just hold/settle.
_STEER_GAIT = 'walk'                 # uses lin_x + ang_z
_VERTICAL_GAITS = ('swim', 'jump')   # use lin_x + lin_z
_STATIC_GAITS = ('stand', 'sink')    # zero cmd_vel (the gait itself does the work)
_ALL_GAITS = (_STEER_GAIT,) + _VERTICAL_GAITS + _STATIC_GAITS


class SELQIEJoystick:
    """Joystick front-end that drives the shared SELQIE interface."""

    def __init__(self, selqie: SELQIE = None):
        """Build the controller, wiring a joy subscription and control timer."""
        self._selqie = selqie if selqie is not None else SELQIE('selqie_joystick')
        self._selqie.init()
        node = self._selqie

        def _p(name, default):
            return node.declare_parameter(name, default).value

        # ---- Button map (Xbox defaults; -1 disables) ----
        self._btn_walk = int(_p('button_walk', 0))    # A
        self._btn_swim = int(_p('button_swim', 1))    # B
        self._btn_stand = int(_p('button_stand', 2))  # X
        self._btn_jump = int(_p('button_jump', 3))    # Y
        self._btn_sink = int(_p('button_sink', 4))    # LB
        self._btn_deadman = int(_p('deadman_button', 5))  # RB (-1 = always live)
        self._btn_idle = int(_p('button_idle', 6))    # Back
        self._btn_ready = int(_p('button_ready', 7))  # Start
        self._btn_zero = int(_p('button_zero', 9))    # L3

        self._gait_buttons = {
            self._btn_walk: 'walk',
            self._btn_swim: 'swim',
            self._btn_stand: 'stand',
            self._btn_jump: 'jump',
            self._btn_sink: 'sink',
        }

        # ---- Axis map (Xbox defaults) ----
        self._axis_forward = int(_p('axis_forward', 1))    # left stick Y
        self._axis_steer = int(_p('axis_steer', 3))        # right stick X
        self._axis_vertical = int(_p('axis_vertical', 4))  # right stick Y
        self._axis_speed = int(_p('axis_speed_scale', 7))  # D-pad Y (-1 disables)

        # Flip these if a stick reads inverted on your controller.
        self._sign_forward = float(_p('axis_forward_sign', 1.0))
        self._sign_steer = float(_p('axis_steer_sign', 1.0))
        self._sign_vertical = float(_p('axis_vertical_sign', 1.0))

        # ---- Scales / limits ----
        self._max_linear = float(_p('max_linear_speed', 0.3))
        self._max_vertical = float(_p('max_vertical_speed', 0.3))
        self._max_angular = float(_p('max_angular_speed', 0.5))
        self._deadzone = float(_p('deadzone', 0.1))
        self._speed_step = float(_p('speed_scale_step', 0.25))
        self._speed_scale = float(_p('speed_scale_initial', 1.0))
        self._joy_timeout = float(_p('joy_timeout', 0.5))
        rate_hz = float(_p('publish_rate', 50.0))

        # ---- State ----
        self._ready = False
        self._selected_gait = 'stand'
        self._effective_gait = 'stand'  # last gait actually published
        self._last_joy = None
        self._last_joy_time = 0.0
        self._prev_buttons = []
        self._prev_speed_dir = 0

        # ---- ROS wiring on the shared node ----
        node.create_subscription(Joy, 'joy', self._on_joy, QOS_FAST())
        node.create_timer(1.0 / rate_hz, self._control_tick)

        node.get_logger().info(self._controls_summary())

    # ------------------------------------------------------------------ #
    # Joy input                                                          #
    # ------------------------------------------------------------------ #
    def _on_joy(self, msg: Joy):
        """Handle edge-triggered buttons; cache axes for the control timer."""
        self._last_joy = msg
        self._last_joy_time = time.monotonic()

        # Discrete (edge-triggered) actions.
        if self._rising(msg, self._btn_ready):
            self._do_ready()
        if self._rising(msg, self._btn_idle):
            self._do_idle()
        if self._rising(msg, self._btn_zero):
            self._do_zero()
        for btn, gait in self._gait_buttons.items():
            if self._rising(msg, btn):
                self._select_gait(gait)

        self._handle_speed_scale(msg)
        self._prev_buttons = list(msg.buttons)

    def _control_tick(self):
        """Publish gait + cmd_vel at a steady rate from the latest joy state."""
        joy = self._last_joy
        stale = joy is None or (time.monotonic() - self._last_joy_time) > self._joy_timeout
        active = self._ready and not stale and self._deadman_held(joy)

        # When not actively driving, command a stand hold (zero velocity).
        gait = self._selected_gait if active else 'stand'
        if gait != self._effective_gait:
            self._publish_gait(gait)

        if active:
            lin_x, lin_z, ang_z = self._compute_cmd_vel(joy)
        else:
            lin_x, lin_z, ang_z = 0.0, 0.0, 0.0
        self._selqie.set_control_command_velocity(lin_x, lin_z, ang_z)

    # ------------------------------------------------------------------ #
    # Discrete actions                                                   #
    # ------------------------------------------------------------------ #
    def _do_ready(self):
        """Enable the motors into a safe, zero-velocity stand hold."""
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_ready(i)
        self._ready = True
        self._selected_gait = 'stand'
        self._selqie.set_control_gait('stand')
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)
        self._effective_gait = 'stand'
        self._selqie.get_logger().info('READY: motors enabled, holding stand')

    def _do_idle(self):
        """Release the motors."""
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_idle(i)
        self._ready = False
        self._selqie.get_logger().info('IDLE: motors released')

    def _do_zero(self):
        """Zero the encoders -- only permitted while idle."""
        if self._ready:
            self._selqie.get_logger().warn(
                'Refusing to zero while motors are ready; press idle first')
            return
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_position_zero(i)
        self._selqie.get_logger().info('ZERO: encoder origins set')

    def _select_gait(self, gait: str):
        """Select the gait that will run while the deadman is held."""
        if gait == self._selected_gait:
            return
        self._selected_gait = gait
        self._selqie.get_logger().info(f'Gait selected: {gait}')

    def _handle_speed_scale(self, msg: Joy):
        """Step the speed governor from the D-pad (edge-triggered)."""
        if self._axis_speed < 0 or self._axis_speed >= len(msg.axes):
            return
        value = msg.axes[self._axis_speed]
        direction = 1 if value > 0.5 else (-1 if value < -0.5 else 0)
        if direction != 0 and direction != self._prev_speed_dir:
            self._speed_scale = _clamp(
                self._speed_scale + direction * self._speed_step, self._speed_step, 1.0)
            self._selqie.get_logger().info(f'Speed scale: {self._speed_scale:.2f}')
        self._prev_speed_dir = direction

    # ------------------------------------------------------------------ #
    # Command computation                                                #
    # ------------------------------------------------------------------ #
    def _compute_cmd_vel(self, joy: Joy):
        """Map the sticks to (lin_x, lin_z, ang_z) for the selected gait."""
        fwd = self._deadband(self._axis(joy, self._axis_forward)) * self._sign_forward
        steer = self._deadband(self._axis(joy, self._axis_steer)) * self._sign_steer
        vert = self._deadband(self._axis(joy, self._axis_vertical)) * self._sign_vertical

        scale = self._speed_scale
        lin_x = fwd * self._max_linear * scale
        ang_z = steer * self._max_angular * scale
        lin_z = vert * self._max_vertical * scale

        gait = self._selected_gait
        if gait == _STEER_GAIT:                 # walk
            return lin_x, 0.0, ang_z
        if gait in _VERTICAL_GAITS:             # swim, jump
            return lin_x, lin_z, 0.0
        return 0.0, 0.0, 0.0                    # stand, sink

    def _publish_gait(self, gait: str):
        """Publish a gait change."""
        self._selqie.set_control_gait(gait)
        self._effective_gait = gait

    # ------------------------------------------------------------------ #
    # Small helpers                                                      #
    # ------------------------------------------------------------------ #
    def _deadman_held(self, joy: Joy) -> bool:
        if self._btn_deadman < 0:
            return True  # deadman disabled -> always live
        return joy is not None and self._button(joy, self._btn_deadman)

    def _rising(self, joy: Joy, idx: int) -> bool:
        """True on a 0->1 transition of button ``idx``."""
        if idx < 0 or idx >= len(joy.buttons):
            return False
        prev = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
        return joy.buttons[idx] == 1 and prev == 0

    @staticmethod
    def _button(joy: Joy, idx: int) -> bool:
        return 0 <= idx < len(joy.buttons) and joy.buttons[idx] == 1

    @staticmethod
    def _axis(joy: Joy, idx: int) -> float:
        if joy is None or idx < 0 or idx >= len(joy.axes):
            return 0.0
        return float(joy.axes[idx])

    def _deadband(self, value: float) -> float:
        """Zero out small values and rescale so motion starts smoothly at 0."""
        if abs(value) < self._deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    def _controls_summary(self) -> str:
        return (
            '\n=== SELQIE joystick teleop ===\n'
            f'  ready=btn{self._btn_ready}  idle=btn{self._btn_idle}  '
            f'zero=btn{self._btn_zero} (idle only)\n'
            f'  gaits: walk=btn{self._btn_walk} swim=btn{self._btn_swim} '
            f'stand=btn{self._btn_stand} jump=btn{self._btn_jump} '
            f'sink=btn{self._btn_sink}\n'
            f'  deadman=btn{self._btn_deadman} (hold to move)\n'
            f'  forward=axis{self._axis_forward} steer=axis{self._axis_steer} '
            f'vertical=axis{self._axis_vertical}\n'
            '  Press ready, hold the deadman, select a gait, and steer.\n'
        )


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def main():
    """Entry point: spin the joystick node."""
    rclpy.init()
    controller = SELQIEJoystick()
    try:
        rclpy.spin(controller._selqie)
    except KeyboardInterrupt:
        pass
    finally:
        controller._selqie.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
