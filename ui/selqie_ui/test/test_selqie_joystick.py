#!/usr/bin/env python3
"""
Logic tests for the SELQIE joystick teleop node.

Neither ROS nor a joystick is required: ``rclpy`` / ``sensor_msgs`` /
``selqie_python`` are stubbed, and a fake SELQIE interface records the calls the
controller makes so the button/axis handling and safety rules can be asserted.
"""

import sys
import types

import pytest


# --------------------------- module stubs ---------------------------- #

def _install_stubs():
    if 'selqie_python.selqie' in sys.modules and getattr(
            sys.modules['selqie_python.selqie'], '_selqie_stub', False):
        return

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: True
    sys.modules['rclpy'] = rclpy

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.Joy = type('Joy', (), {})
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg

    selqie_python = types.ModuleType('selqie_python')
    selqie_mod = types.ModuleType('selqie_python.selqie')
    selqie_mod._selqie_stub = True
    selqie_mod.SELQIE = type('SELQIE', (), {})
    selqie_mod.QOS_FAST = lambda: None
    selqie_python.selqie = selqie_mod
    sys.modules['selqie_python'] = selqie_python
    sys.modules['selqie_python.selqie'] = selqie_mod


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class FakeSelqie:
    """Records the interface calls the joystick controller makes."""

    NUM_MOTORS = 8

    def __init__(self):
        self.timer_cb = None
        self.joy_cb = None
        self.cmd_vels = []
        self.gaits = []
        self.pos_modes = []
        self.ready_calls = []
        self.idle_calls = []
        self.zero_calls = []

    def init(self):
        pass

    def declare_parameter(self, name, default):
        return _Param(default)

    def create_subscription(self, msg_type, topic, cb, qos):
        self.joy_cb = cb
        return None

    def create_timer(self, period, cb):
        self.timer_cb = cb
        return None

    def get_logger(self):
        return _Logger()

    def set_control_command_velocity(self, lx, lz, az):
        self.cmd_vels.append((lx, lz, az))

    def set_control_gait(self, gait):
        self.gaits.append(gait)

    def set_all_motors_position_mode(self, mode):
        self.pos_modes.append(mode)

    def set_motor_ready(self, i):
        self.ready_calls.append(i)

    def set_motor_idle(self, i):
        self.idle_calls.append(i)

    def set_motor_position_zero(self, i):
        self.zero_calls.append(i)


def _joy(buttons=(), axes=None):
    """Build a Joy-like object. ``buttons`` = pressed indices; ``axes`` = {idx: v}."""
    b = [0] * 16
    for i in buttons:
        b[i] = 1
    a = [0.0] * 16
    for i, v in (axes or {}).items():
        a[i] = v
    return types.SimpleNamespace(buttons=b, axes=a)


@pytest.fixture
def controller():
    _install_stubs()
    from selqie_ui import selqie_joystick as sj

    fake = FakeSelqie()
    jc = sj.SELQIEJoystick(selqie=fake)
    return jc, fake


# Xbox default indices (must match the node defaults).
BTN_WALK, BTN_SWIM, BTN_STAND, BTN_JUMP, BTN_SINK = 0, 1, 2, 3, 4
BTN_DEADMAN, BTN_IDLE, BTN_READY, BTN_ZERO = 5, 6, 7, 9
AXIS_FORWARD, AXIS_STEER, AXIS_VERTICAL = 1, 3, 4


def _press(jc, button, **kw):
    """Simulate a clean press: neutral frame then the button held."""
    jc._on_joy(_joy())
    jc._on_joy(_joy(buttons=(button,), **kw))


def test_ready_enables_and_holds_stand(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    assert fake.ready_calls == list(range(8))
    assert jc._ready is True
    assert jc._selected_gait == 'stand'
    assert fake.gaits[-1] == 'stand'
    assert fake.pos_modes[-1] == 'pos_spd'
    assert fake.cmd_vels[-1] == (0.0, 0.0, 0.0)


def test_walk_with_deadman_streams_velocity(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    assert jc._selected_gait == 'walk'

    # Hold deadman + full forward + steer, then run a control tick.
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,),
                    axes={AXIS_FORWARD: 1.0, AXIS_STEER: 1.0}))
    jc._control_tick()

    assert fake.gaits[-1] == 'walk'
    # Moving gaits use the acceleration-shaped submode too, now that the stride
    # generators fill in each setpoint's velocity for it to travel at.
    assert fake.pos_modes[-1] == 'pos_spd'
    lin_x, lin_z, ang_z = fake.cmd_vels[-1]
    assert lin_x == pytest.approx(0.3)   # max_linear default, deadband(1.0)=1.0
    assert lin_z == 0.0                  # walk ignores vertical
    assert ang_z == pytest.approx(0.5)   # max_angular default


def test_deadman_release_commands_stand_stop(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,), axes={AXIS_FORWARD: 1.0}))
    jc._control_tick()

    # Release the deadman (still ready): must stand and stop.
    jc._on_joy(_joy(axes={AXIS_FORWARD: 1.0}))
    jc._control_tick()
    assert fake.gaits[-1] == 'stand'
    assert fake.cmd_vels[-1] == (0.0, 0.0, 0.0)


def test_swim_uses_vertical_axis(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_SWIM)
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,),
                    axes={AXIS_FORWARD: 1.0, AXIS_VERTICAL: 1.0, AXIS_STEER: 1.0}))
    jc._control_tick()
    lin_x, lin_z, ang_z = fake.cmd_vels[-1]
    assert lin_x == pytest.approx(0.3)
    assert lin_z == pytest.approx(0.3)   # swim uses vertical
    assert ang_z == 0.0                  # swim ignores steer


def test_zero_refused_when_ready_allowed_when_idle(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_ZERO)
    assert fake.zero_calls == []         # refused while ready

    _press(jc, BTN_IDLE)
    assert jc._ready is False
    _press(jc, BTN_ZERO)
    assert fake.zero_calls == list(range(8))  # allowed while idle


def test_stale_joy_stops_motion(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,), axes={AXIS_FORWARD: 1.0}))
    jc._control_tick()
    assert fake.cmd_vels[-1][0] == pytest.approx(0.3)

    # Simulate a joy dropout: last message is old.
    import time
    jc._last_joy_time = time.monotonic() - 100.0
    jc._control_tick()
    assert fake.cmd_vels[-1] == (0.0, 0.0, 0.0)
    assert fake.gaits[-1] == 'stand'


def test_deadzone_rejects_drift(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    # Tiny stick offset below the 0.1 deadzone must produce no motion.
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,), axes={AXIS_FORWARD: 0.05}))
    jc._control_tick()
    assert fake.cmd_vels[-1] == (0.0, 0.0, 0.0)


def test_not_ready_never_streams_motion(controller):
    jc, fake = controller
    # No ready pressed: even with deadman + full stick, stay stopped.
    _press(jc, BTN_WALK)
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,), axes={AXIS_FORWARD: 1.0}))
    jc._control_tick()
    assert fake.cmd_vels[-1] == (0.0, 0.0, 0.0)


def test_reverse_is_negative_forward(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    jc._on_joy(_joy(buttons=(BTN_DEADMAN,), axes={AXIS_FORWARD: -1.0}))
    jc._control_tick()
    assert fake.cmd_vels[-1][0] == pytest.approx(-0.3)


def test_speed_scale_dpad(controller):
    jc, fake = controller
    _press(jc, BTN_READY)
    _press(jc, BTN_WALK)
    start = jc._speed_scale
    # D-pad down (axis 7 = -1) steps the governor down once.
    jc._on_joy(_joy())
    jc._on_joy(_joy(axes={7: -1.0}))
    assert jc._speed_scale == pytest.approx(start - jc._speed_step)
