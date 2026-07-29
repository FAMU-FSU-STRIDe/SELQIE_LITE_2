#!/usr/bin/env python3
"""
Unit tests for the constant-rate trajectory resampling used by
``SELQIE.get_leg_trajectories_from_file`` (``resample_leg_trajectory`` and its
lerp helpers in ``selqie_python.selqie``).

Neither ROS nor a real robot is required: the ROS message/package modules
``selqie.py`` imports at module scope are stubbed with minimal duck-typed
classes so the real resampling logic can be imported and exercised directly.
"""

import math
import sys
import types

import pytest


def _install_stubs():
    if 'rclpy' in sys.modules and getattr(sys.modules['rclpy'], '_selqie_stub', False):
        return

    def _vec3_class():
        class Vector3:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
        return Vector3

    Vector3 = _vec3_class()

    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod._selqie_stub = True
    rclpy_mod.init = lambda *a, **k: None
    rclpy_mod.shutdown = lambda *a, **k: None
    rclpy_mod.spin = lambda *a, **k: None
    rclpy_mod.spin_once = lambda *a, **k: None
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = type('Node', (), {'__init__': lambda self, *a, **k: None})
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = type('QoSProfile', (), {'__init__': lambda self, **k: None})
    qos_mod.QoSReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1)
    rclpy_mod.node = node_mod
    rclpy_mod.qos = qos_mod
    sys.modules['rclpy'] = rclpy_mod
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rclpy.qos'] = qos_mod

    ament_mod = types.ModuleType('ament_index_python')
    ament_pkg_mod = types.ModuleType('ament_index_python.packages')
    ament_pkg_mod.get_package_share_directory = lambda name: '/tmp'
    ament_mod.packages = ament_pkg_mod
    sys.modules['ament_index_python'] = ament_mod
    sys.modules['ament_index_python.packages'] = ament_pkg_mod

    def _msg_module(name, fields_by_class):
        mod = types.ModuleType(name)
        for cls_name, fields in fields_by_class.items():
            def make_init(f):
                def __init__(self):
                    for k, v in f.items():
                        setattr(self, k, v() if callable(v) else v)
                return __init__
            setattr(mod, cls_name, type(cls_name, (), {'__init__': make_init(fields)}))
        return mod

    std_msgs = _msg_module('std_msgs.msg', {
        'Bool': dict(data=False), 'Empty': dict(), 'Float32': dict(data=0.0),
        'String': dict(data=""), 'UInt32MultiArray': dict(data=[]),
    })
    sys.modules['std_msgs'] = types.ModuleType('std_msgs')
    sys.modules['std_msgs.msg'] = std_msgs

    geometry_msgs = _msg_module('geometry_msgs.msg', {
        'Twist': dict(), 'PoseStamped': dict(), 'PoseWithCovarianceStamped': dict(),
        'Quaternion': dict(w=1.0, x=0.0, y=0.0, z=0.0),
    })
    geometry_msgs.Vector3 = Vector3
    sys.modules['geometry_msgs'] = types.ModuleType('geometry_msgs')
    sys.modules['geometry_msgs.msg'] = geometry_msgs

    nav_msgs = _msg_module('nav_msgs.msg', {'Odometry': dict()})
    sys.modules['nav_msgs'] = types.ModuleType('nav_msgs')
    sys.modules['nav_msgs.msg'] = nav_msgs

    sensor_msgs = _msg_module('sensor_msgs.msg', {'Image': dict(), 'Imu': dict()})
    sys.modules['sensor_msgs'] = types.ModuleType('sensor_msgs')
    sys.modules['sensor_msgs.msg'] = sensor_msgs

    actuation_msgs = _msg_module('actuation_msgs.msg', {
        'MotorCommand': dict(
            control_mode=0, input_mode=0, pos_setpoint=0.0,
            vel_setpoint=0.0, torq_setpoint=0.0,
        ),
    })
    sys.modules['actuation_msgs'] = types.ModuleType('actuation_msgs')
    sys.modules['actuation_msgs.msg'] = actuation_msgs

    motor_interfaces = _msg_module('motor_interfaces.msg', {'MotorState': dict()})
    sys.modules['motor_interfaces'] = types.ModuleType('motor_interfaces')
    sys.modules['motor_interfaces.msg'] = motor_interfaces

    leg_control_msgs = _msg_module('leg_control_msgs.msg', {
        'LegCommand': dict(
            control_mode=0, pos_setpoint=Vector3,
            vel_setpoint=Vector3, force_setpoint=Vector3,
        ),
        'LegTrajectory': dict(timing=list, commands=list),
        'LegEstimate': dict(
            pos_estimate=Vector3, vel_estimate=Vector3, force_estimate=Vector3,
        ),
    })
    sys.modules['leg_control_msgs'] = types.ModuleType('leg_control_msgs')
    sys.modules['leg_control_msgs.msg'] = leg_control_msgs

    robot_localization = types.ModuleType('robot_localization')
    robot_localization_srv = types.ModuleType('robot_localization.srv')
    robot_localization_srv.SetPose = type('SetPose', (), {})
    robot_localization.srv = robot_localization_srv
    sys.modules['robot_localization'] = robot_localization
    sys.modules['robot_localization.srv'] = robot_localization_srv


_install_stubs()
from selqie_python import selqie as sq  # noqa: E402


def _cmd(x, y, z, mode=3):
    """Build a LegCommand-like object with the given position setpoint."""
    c = sq.LegCommand()
    c.control_mode = mode
    c.pos_setpoint.x, c.pos_setpoint.y, c.pos_setpoint.z = x, y, z
    c.vel_setpoint.x, c.vel_setpoint.y, c.vel_setpoint.z = 0.0, 0.0, 0.0
    c.force_setpoint.x, c.force_setpoint.y, c.force_setpoint.z = 0.0, 0.0, 0.0
    return c


def _uniform_cycle(n, dt, amplitude=1.0):
    """Build a uniformly-spaced, periodic (x[0]==x[-1]) sine-like cycle."""
    times = [i * dt for i in range(n)]
    commands = [_cmd(amplitude * math.sin(2 * math.pi * i / n), 0.0, -0.1) for i in range(n)]
    return times, commands


# --------------------------- basic behaviour ---------------------------


def test_empty_and_single_point_pass_through():
    assert sq.resample_leg_trajectory([], [], 1000.0) == ([], [])
    t, c = sq.resample_leg_trajectory([0.0], [_cmd(1, 2, 3)], 1000.0)
    assert t == [0.0]
    assert c[0].pos_setpoint.x == 1


def test_resampled_times_are_evenly_spaced_at_target_rate():
    times, commands = _uniform_cycle(n=500, dt=0.002)  # 1s cycle @ freq=1
    new_times, new_commands = sq.resample_leg_trajectory(times, commands, 1000.0)

    assert len(new_times) == len(new_commands)
    diffs = [new_times[i + 1] - new_times[i] for i in range(len(new_times) - 1)]
    assert all(d == pytest.approx(0.001, abs=1e-9) for d in diffs)
    assert new_times[0] == pytest.approx(0.0)
    assert new_times[-1] < 1.0  # stays within one cycle


def test_upsamples_at_low_effective_rate_denser_than_original():
    # 500 points over a 1s cycle (base "frequency=1" case) resampled to 1000Hz
    # should produce MORE points than the original file (denser, not sparser).
    times, commands = _uniform_cycle(n=500, dt=0.002)
    new_times, _ = sq.resample_leg_trajectory(times, commands, 1000.0)
    assert len(new_times) > len(times)
    assert len(new_times) == pytest.approx(1000, abs=1)


def test_downsamples_at_high_effective_rate_bounded_by_resample_hz():
    # Simulate a 5x-compressed cycle (500 points/1s file run at frequency=5):
    # duration 0.2s, spacing 0.0004s. Resampled to 1000Hz must yield ~200
    # points -- bounded, not 2500 (500*5) as the pre-resampling code would need.
    times, commands = _uniform_cycle(n=500, dt=0.002 / 5.0)
    new_times, new_commands = sq.resample_leg_trajectory(times, commands, 1000.0)
    assert len(new_times) == pytest.approx(200, abs=1)
    assert len(new_commands) == len(new_times)


def test_no_two_consecutive_points_ever_exceed_the_target_spacing():
    # "No skipped points" at any compression: consecutive resampled samples
    # must never be spaced more than 1/resample_hz apart (within FP tolerance).
    resample_hz = 1000.0
    for freq in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
        times, commands = _uniform_cycle(n=500, dt=0.002 / freq)
        new_times, _ = sq.resample_leg_trajectory(times, commands, resample_hz)
        gaps = [new_times[i + 1] - new_times[i] for i in range(len(new_times) - 1)]
        assert all(g <= (1.0 / resample_hz) + 1e-9 for g in gaps), f'freq={freq}'


def test_interpolated_values_track_the_original_curve():
    times, commands = _uniform_cycle(n=500, dt=0.002, amplitude=2.0)
    new_times, new_commands = sq.resample_leg_trajectory(times, commands, 1000.0)
    # Compare against the analytic sine the cycle was built from.
    for t, c in list(zip(new_times, new_commands))[::37]:
        expected = 2.0 * math.sin(2 * math.pi * t / 1.0)
        assert c.pos_setpoint.x == pytest.approx(expected, abs=0.05)


def test_max_points_caps_low_frequency_message_size():
    # A 2s cycle (500-point/1s file run at frequency=0.5) at 1000Hz would be
    # 2000 points; the cap must hold it to 1000 while still covering the cycle.
    times, commands = _uniform_cycle(n=500, dt=0.002 / 0.5)  # 2s cycle
    uncapped, _ = sq.resample_leg_trajectory(times, commands, 1000.0)
    capped, capped_cmds = sq.resample_leg_trajectory(times, commands, 1000.0, 1000)

    assert len(uncapped) == pytest.approx(2000, abs=1)
    assert len(capped) == 1000
    assert len(capped_cmds) == 1000
    # Still spans the whole cycle (does not truncate the stride).
    assert capped[-1] == pytest.approx(uncapped[-1], abs=0.01)


def test_max_points_keeps_even_spacing():
    times, commands = _uniform_cycle(n=500, dt=0.002 / 0.5)
    capped, _ = sq.resample_leg_trajectory(times, commands, 1000.0, 1000)
    gaps = [capped[i + 1] - capped[i] for i in range(len(capped) - 1)]
    assert all(g == pytest.approx(gaps[0], abs=1e-9) for g in gaps)


def test_max_points_does_not_bind_at_high_frequency():
    # A 5x-compressed cycle only needs ~200 points, well under the cap, so the
    # cap must not perturb it at all.
    times, commands = _uniform_cycle(n=500, dt=0.002 / 5.0)
    uncapped, _ = sq.resample_leg_trajectory(times, commands, 1000.0)
    capped, _ = sq.resample_leg_trajectory(times, commands, 1000.0, 1000)
    assert len(capped) == len(uncapped)
    assert capped == pytest.approx(uncapped)


def test_max_points_is_exactly_min_of_rate_limit_and_cap():
    # Precise contract: the cap composes with the rate limit, never fights it.
    for freq in (0.25, 0.5, 1.0, 2.0, 5.0):
        times, commands = _uniform_cycle(n=500, dt=0.002 / freq)
        uncapped, _ = sq.resample_leg_trajectory(times, commands, 1000.0)
        capped, _ = sq.resample_leg_trajectory(times, commands, 1000.0, 1000)
        assert len(capped) == min(len(uncapped), 1000), f'freq={freq}'


def test_cap_does_not_make_low_frequency_sparser_than_source():
    # At the low frequencies where the cap actually binds (and where the
    # twitching was reported), the result must still be no coarser than the
    # source file's own density. At high frequency the *rate* limit dominates
    # and intentionally goes below source density -- that is the CAN bandwidth
    # bound, not the cap, and is covered by the downsampling test above.
    source_pts = 500
    for freq in (0.25, 0.5, 1.0):
        times, commands = _uniform_cycle(n=source_pts, dt=0.002 / freq)
        capped, _ = sq.resample_leg_trajectory(times, commands, 1000.0, 1000)
        assert len(capped) >= source_pts, f'freq={freq}'


def test_max_points_disabled_by_zero_or_negative():
    times, commands = _uniform_cycle(n=500, dt=0.002 / 0.5)
    base, _ = sq.resample_leg_trajectory(times, commands, 1000.0)
    for disabled in (0, -1):
        got, _ = sq.resample_leg_trajectory(times, commands, 1000.0, disabled)
        assert len(got) == len(base)


def test_control_mode_is_held_not_blended():
    times = [0.0, 0.1, 0.2]
    commands = [_cmd(0, 0, 0, mode=3), _cmd(1, 0, 0, mode=3), _cmd(0, 0, 0, mode=3)]
    _, new_commands = sq.resample_leg_trajectory(times, commands, 50.0)
    assert all(c.control_mode == 3 for c in new_commands)


def test_wraparound_gap_blends_toward_first_sample():
    # Non-periodic sequence so the closing-gap blend is visible: last sample
    # holds until it starts blending toward the first sample's value.
    times = [0.0, 0.1, 0.2]
    commands = [_cmd(0, 0, 0), _cmd(5, 0, 0), _cmd(10, 0, 0)]
    new_times, new_commands = sq.resample_leg_trajectory(times, commands, 100.0)
    # Points beyond t_end=0.2 (up to period=0.3) should blend from 10 -> 0.
    tail = [(t, c.pos_setpoint.x) for t, c in zip(new_times, new_commands) if t > 0.2]
    assert len(tail) > 0
    xs = [x for _, x in tail]
    assert xs[0] < 10.0 and xs[0] > 0.0  # partway blended, not a hard jump
    assert all(x2 <= x1 for x1, x2 in zip(xs, xs[1:]))  # monotonically decreasing toward 0


def test_non_uniform_spacing_is_respected():
    # Mirrors the real trajectory files: mostly-uniform steps with one wider
    # trailing interval. The resampler must use actual timestamps, not assume
    # a fixed step derived from the first interval.
    times = [0.0, 0.002, 0.004, 0.006, 0.007]  # last step is 1ms not 2ms
    commands = [_cmd(0, 0, 0), _cmd(1, 0, 0), _cmd(2, 0, 0), _cmd(3, 0, 0), _cmd(4, 0, 0)]
    new_times, new_commands = sq.resample_leg_trajectory(times, commands, 2000.0)
    # A sample at t=0.0065 should interpolate between commands[3] (t=0.006,x=3)
    # and commands[4] (t=0.007,x=4), not misread the spacing as uniform 2ms.
    idx = min(range(len(new_times)), key=lambda i: abs(new_times[i] - 0.0065))
    assert new_commands[idx].pos_setpoint.x == pytest.approx(3.5, abs=0.05)


def test_frequency_scaling_matches_manual_precompression():
    # Resampling a trajectory pre-scaled by frequency=2 should be equivalent
    # to resampling the base (freq=1) trajectory's times all halved.
    base_times, commands = _uniform_cycle(n=100, dt=0.01)
    scaled_times = [t / 2.0 for t in base_times]
    new_times, new_commands = sq.resample_leg_trajectory(scaled_times, commands, 200.0)
    assert new_times[-1] <= base_times[-1] / 2.0 + 1e-9
    assert len(new_times) == pytest.approx(100, abs=1)  # half the duration, same rate -> half pts
