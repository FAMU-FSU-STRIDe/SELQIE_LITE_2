#!/usr/bin/env python3
"""
Integration tests for ``MotorNode`` MIT-mode command handling.

``rclpy`` and ``python-can`` are not required: lightweight stub modules are
injected into ``sys.modules`` so the real ``MotorNode`` can be constructed
against a fake CAN bus that records transmitted frames. This exercises the full
command -> MIT-frame path, including how gains reach the wire and how they can
be retuned live.
"""

import sys
import types

import pytest

from cubemars_v2_ros import mit_protocol as mit


# --------------------------- module stubs -------------------------------


class _FakeMessage:
    def __init__(self, arbitration_id=0, data=b"", is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


class _FakeBus:
    """Records sent frames; recv never yields anything."""

    def __init__(self, *args, **kwargs):
        self.sent = []

    def set_filters(self, filters):
        self.filters = filters

    def send(self, msg):
        self.sent.append(msg)

    def recv(self, timeout=0.0):
        return None

    def shutdown(self):
        pass


def _install_stubs():
    if "can" in sys.modules and getattr(sys.modules["can"], "_selqie_stub", False):
        return

    can_mod = types.ModuleType("can")
    can_mod._selqie_stub = True
    can_mod.Message = _FakeMessage
    can_mod.CanError = type("CanError", (Exception,), {})
    iface = types.ModuleType("can.interface")
    iface.Bus = _FakeBus
    can_mod.interface = iface
    sys.modules["can"] = can_mod
    sys.modules["can.interface"] = iface

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda *a, **k: None
    rclpy_mod.shutdown = lambda *a, **k: None
    rclpy_mod.spin = lambda *a, **k: None
    rclpy_mod.ok = lambda: True
    node_mod = types.ModuleType("rclpy.node")

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

    class _Node:
        def __init__(self, name):
            if not hasattr(self, "_params"):
                self._params = {}
            self._log = _Logger()
            self.published = []

        def declare_parameter(self, name, default):
            self._params.setdefault(name, default)

        def get_parameter(self, name):
            return _Param(self._params.get(name))

        def get_logger(self):
            return self._log

        def create_publisher(self, msg_type, topic, qos):
            sink = self.published

            class _Pub:
                def publish(self, msg):
                    sink.append((topic, msg))
            return _Pub()

        def create_subscription(self, *a, **k):
            return None

        def create_timer(self, *a, **k):
            return None

    node_mod.Node = _Node
    rclpy_mod.node = node_mod
    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod

    def _msg_class(fields, consts=None):
        def __init__(self):
            for f, v in fields.items():
                setattr(self, f, list(v) if isinstance(v, list) else v)
        cls = type("Msg", (), {"__init__": __init__})
        for k, v in (consts or {}).items():
            setattr(cls, k, v)
        return cls

    actuation_msgs = types.ModuleType("actuation_msgs")
    actuation_msgs_msg = types.ModuleType("actuation_msgs.msg")
    actuation_msgs_msg.MotorCommand = _msg_class(
        dict(control_mode=0, input_mode=0, pos_setpoint=0.0,
             vel_setpoint=0.0, torq_setpoint=0.0),
        dict(CONTROL_MODE_TORQUE=1, CONTROL_MODE_VELOCITY=2,
             CONTROL_MODE_POSITION=3, INPUT_MODE_PASSTHROUGH=1),
    )
    actuation_msgs_msg.MotorEstimate = _msg_class(
        dict(pos_estimate=0.0, vel_estimate=0.0, torq_estimate=0.0))
    actuation_msgs.msg = actuation_msgs_msg
    sys.modules["actuation_msgs"] = actuation_msgs
    sys.modules["actuation_msgs.msg"] = actuation_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float64MultiArray = _msg_class(dict(data=[]))
    std_msgs_msg.String = _msg_class(dict(data=""))
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    motor_interfaces = types.ModuleType("motor_interfaces")
    motor_interfaces_msg = types.ModuleType("motor_interfaces.msg")
    motor_interfaces_msg.MotorState = _msg_class(
        dict(name="", position=0.0, abs_position=0.0, velocity=0.0,
             torque=0.0, current=0.0, temperature=0))
    motor_interfaces.msg = motor_interfaces_msg
    sys.modules["motor_interfaces"] = motor_interfaces
    sys.modules["motor_interfaces.msg"] = motor_interfaces_msg


@pytest.fixture
def make_node():
    _install_stubs()
    from cubemars_v2_ros import motor_node as mn

    created = []

    def _factory(**params):
        node = mn.MotorNode.__new__(mn.MotorNode)
        defaults = dict(
            can_interface="can0", can_id=1, motor_type="AK40-10",
            control_hz=500.0, joint_name="motor0", auto_start=True,
            reverse_polarity=False, cmd_timeout=0.0,
            position_kp=5.0, position_kd=0.4, velocity_kd=0.5,
            torque_limit_scale=1.0,
        )
        defaults.update(params)
        node._params = dict(defaults)
        mn.MotorNode.__init__(node)
        node._stop = True
        created.append(node)
        return node, mn

    yield _factory
    for node in created:
        node._stop = True


def _drive_frames(node):
    """Frames that are motion commands, i.e. not the 0xFF-prefixed specials."""
    return [m for m in node.bus.sent if not m.data.startswith(b"\xff" * 7)]


def _decode(frame, limits):
    """Decode a MIT command frame back into (p, v, kp, kd, t)."""
    d = frame.data
    p_i = (d[0] << 8) | d[1]
    v_i = (d[2] << 4) | (d[3] >> 4)
    kp_i = ((d[3] & 0x0F) << 8) | d[4]
    kd_i = (d[5] << 4) | (d[6] >> 4)
    t_i = ((d[6] & 0x0F) << 8) | d[7]
    return (
        mit.uint_to_float(p_i, limits["P_MIN"], limits["P_MAX"], 16),
        mit.uint_to_float(v_i, limits["V_MIN"], limits["V_MAX"], 12),
        mit.uint_to_float(kp_i, limits["KP_MIN"], limits["KP_MAX"], 12),
        mit.uint_to_float(kd_i, limits["KD_MIN"], limits["KD_MAX"], 12),
        mit.uint_to_float(t_i, limits["T_MIN"], limits["T_MAX"], 12),
    )


# --------------------------- mode entry ---------------------------------


def test_auto_start_enters_mit_mode(make_node):
    node, _ = make_node(auto_start=True)
    # Entering MIT mode is mandatory before motion (manual 5.3).
    assert node.bus.sent[0].data == mit.special_frame(mit.MIT_ENTER_MODE)
    assert node.bus.sent[0].is_extended_id is False


def test_start_exit_zero_special_frames(make_node):
    node, mn = make_node(auto_start=False)

    def special(text):
        s = mn.String()
        s.data = text
        node.on_special(s)

    special("start")
    assert node.bus.sent[-1].data == mit.special_frame(mit.MIT_ENTER_MODE)
    special("zero")
    assert node.bus.sent[-1].data == mit.special_frame(mit.MIT_ZERO_POSITION)
    special("exit")
    assert node.bus.sent[-1].data == mit.special_frame(mit.MIT_EXIT_MODE)


def test_no_drive_frames_before_entering_mit_mode(make_node):
    node, mn = make_node(auto_start=False)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_command(cmd)
    node._tick()
    assert _drive_frames(node) == []


# --------------------------- gains on the wire --------------------------


def test_position_mode_sends_configured_gains(make_node):
    node, mn = make_node(position_kp=7.5, position_kd=0.8)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_command(cmd)
    node._tick()

    p, v, kp, kd, t = _decode(_drive_frames(node)[-1], node.R)
    assert p == pytest.approx(1.0, abs=1e-3)
    assert kp == pytest.approx(7.5, abs=0.2)
    assert kd == pytest.approx(0.8, abs=0.01)


def test_velocity_mode_zeroes_kp_and_uses_velocity_kd(make_node):
    node, mn = make_node(velocity_kd=1.25)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_VELOCITY
    cmd.vel_setpoint = 10.0
    node.on_command(cmd)
    node._tick()

    p, v, kp, kd, t = _decode(_drive_frames(node)[-1], node.R)
    assert kp == 0.0                       # no position target to hold
    assert kd == pytest.approx(1.25, abs=0.01)
    assert v == pytest.approx(10.0, abs=0.05)


def test_torque_mode_is_pure_feedforward(make_node):
    node, mn = make_node()
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_TORQUE
    cmd.torq_setpoint = 2.0
    node.on_command(cmd)
    node._tick()

    p, v, kp, kd, t = _decode(_drive_frames(node)[-1], node.R)
    assert kp == 0.0
    assert kd == 0.0
    assert t == pytest.approx(2.0, abs=0.01)


# --------------------------- live gain tuning ---------------------------


def test_set_gains_topic_changes_subsequent_commands(make_node):
    node, mn = make_node(position_kp=5.0, position_kd=0.4)

    gains = mn.Float64MultiArray()
    gains.data = [12.0, 1.5]
    node.on_set_gains(gains)

    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 0.5
    node.on_command(cmd)
    node._tick()

    _, _, kp, kd, _ = _decode(_drive_frames(node)[-1], node.R)
    assert kp == pytest.approx(12.0, abs=0.2)
    assert kd == pytest.approx(1.5, abs=0.01)


def test_set_gains_clamps_to_protocol_range(make_node):
    node, mn = make_node()
    gains = mn.Float64MultiArray()
    gains.data = [10000.0, 99.0]   # far beyond Kp<=500, Kd<=5
    node.on_set_gains(gains)
    assert node.position_kp == 500.0
    assert node.position_kd == 5.0


def test_set_gains_rejects_bad_length(make_node):
    node, mn = make_node(position_kp=5.0)
    bad = mn.Float64MultiArray()
    bad.data = [1.0]
    node.on_set_gains(bad)
    assert node.position_kp == 5.0   # unchanged


def test_set_gains_publishes_current_values(make_node):
    node, mn = make_node()
    gains = mn.Float64MultiArray()
    gains.data = [3.0, 0.3]
    node.on_set_gains(gains)
    echoed = [m for topic, m in node.published if topic.endswith("/gains")]
    assert echoed and echoed[-1].data[:2] == [3.0, 0.3]


# --------------------------- safety -------------------------------------


def test_cmd_timeout_releases_motor(make_node):
    node, mn = make_node(cmd_timeout=0.001)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_command(cmd)

    import time
    time.sleep(0.01)
    node._tick()

    _, _, kp, kd, t = _decode(_drive_frames(node)[-1], node.R)
    assert (kp, kd) == (0.0, 0.0)
    assert t == pytest.approx(0.0, abs=1e-2)


def test_clear_holds_neutral(make_node):
    node, mn = make_node()
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_command(cmd)

    s = mn.String()
    s.data = "clear"
    node.on_special(s)
    node._tick()

    _, _, kp, kd, t = _decode(_drive_frames(node)[-1], node.R)
    assert (kp, kd) == (0.0, 0.0)


def test_reverse_polarity_negates_position(make_node):
    node, mn = make_node(reverse_polarity=True)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_command(cmd)
    node._tick()

    p = _decode(_drive_frames(node)[-1], node.R)[0]
    assert p == pytest.approx(-1.0, abs=1e-3)


def test_torque_limit_scale_shrinks_range(make_node):
    node, _ = make_node(torque_limit_scale=0.5)
    # AK40-10 peak is 4.1 Nm; at half scale the packing range halves too.
    assert node.R["T_MAX"] == pytest.approx(2.05)
    assert node.R["T_MIN"] == pytest.approx(-2.05)


def test_uses_standard_not_extended_can_ids(make_node):
    # MIT mode uses 11-bit standard IDs, unlike servo mode's 29-bit extended.
    node, _ = make_node(can_id=5)
    assert node.arb_id == 5
    assert node.reply_id == 5
    assert all(m.is_extended_id is False for m in node.bus.sent)


# --------------------- terminal round-trip (live retune) ----------------


def test_terminal_style_retune_takes_effect_without_restart(make_node):
    """What `set_gains 12 1.5` in selqie_terminal must achieve.

    The terminal publishes [kp, kd] on /motorN/set_gains; the node applies it to
    every subsequent command with no restart. This walks that whole path.
    """
    node, mn = make_node(position_kp=5.0, position_kd=0.4)

    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 0.5

    # Before: the launch/YAML gains are on the wire.
    node.on_command(cmd)
    node._tick()
    _, _, kp_before, kd_before, _ = _decode(_drive_frames(node)[-1], node.R)
    assert kp_before == pytest.approx(5.0, abs=0.2)

    # The terminal retunes mid-run...
    retune = mn.Float64MultiArray()
    retune.data = [12.0, 1.5]
    node.on_set_gains(retune)

    # ...and the very next command already carries the new gains.
    node.on_command(cmd)
    node._tick()
    _, _, kp_after, kd_after, _ = _decode(_drive_frames(node)[-1], node.R)
    assert kp_after == pytest.approx(12.0, abs=0.2)
    assert kd_after == pytest.approx(1.5, abs=0.01)
    assert kp_after != kp_before


def test_set_gains_can_also_set_velocity_kd(make_node):
    node, mn = make_node(velocity_kd=0.5)
    retune = mn.Float64MultiArray()
    retune.data = [4.0, 0.3, 2.0]        # kp, kd, velocity_kd
    node.on_set_gains(retune)
    assert node.velocity_kd == pytest.approx(2.0)

    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_VELOCITY
    cmd.vel_setpoint = 5.0
    node.on_command(cmd)
    node._tick()
    _, _, kp, kd, _ = _decode(_drive_frames(node)[-1], node.R)
    assert kp == 0.0                      # velocity mode still zeroes stiffness
    assert kd == pytest.approx(2.0, abs=0.01)


def test_gains_are_republished_for_late_subscribers(make_node):
    """The node heartbeats its gains so `gains` in the terminal always has data."""
    node, _ = make_node()
    node.published.clear()
    node._publish_gains()                 # what the 1 Hz timer calls
    echoed = [m for topic, m in node.published if topic.endswith("/gains")]
    assert echoed
    assert len(echoed[-1].data) == 3      # [kp, kd, velocity_kd]
