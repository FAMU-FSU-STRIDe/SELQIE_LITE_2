#!/usr/bin/env python3
"""
Unit tests for the CubeMars MIT-mode CAN protocol (manual V1.0.18 §5.3).

Pure protocol: no ROS, no CAN bus. Checks the bit layout against the manual's
frame tables, the quantization round-trips, and the special command codes.
"""

import math

import pytest

from cubemars_v2_ros import mit_protocol as mit


# AK40-10 V2.0 ranges (datasheet-verified peak torque 4.1 N.m).
LIMITS = dict(P_MIN=-12.5, P_MAX=12.5, V_MIN=-45.5, V_MAX=45.5,
              T_MIN=-4.1, T_MAX=4.1,
              KP_MIN=0.0, KP_MAX=500.0, KD_MIN=0.0, KD_MAX=5.0)


# --------------------------- special commands ---------------------------


def test_special_frames_match_manual():
    # §5.3: seven 0xFF bytes then the code.
    assert mit.special_frame(mit.MIT_ENTER_MODE) == b"\xff" * 7 + b"\xfc"
    assert mit.special_frame(mit.MIT_EXIT_MODE) == b"\xff" * 7 + b"\xfd"
    assert mit.special_frame(mit.MIT_ZERO_POSITION) == b"\xff" * 7 + b"\xfe"
    assert len(mit.special_frame(mit.MIT_ENTER_MODE)) == 8


def test_special_command_codes():
    assert (mit.MIT_ENTER_MODE, mit.MIT_EXIT_MODE, mit.MIT_ZERO_POSITION) == (0xFC, 0xFD, 0xFE)


# --------------------------- quantization -------------------------------


def test_float_uint_endpoints():
    # lo -> 0, hi -> full scale.
    assert mit.float_to_uint(-12.5, -12.5, 12.5, 16) == 0
    assert mit.float_to_uint(12.5, -12.5, 12.5, 16) == 65535
    assert mit.float_to_uint(0.0, -12.5, 12.5, 16) == pytest.approx(32767, abs=1)


def test_float_uint_roundtrip_within_resolution():
    step = mit.position_resolution(LIMITS)
    for value in (-12.5, -3.3, 0.0, 1.234, 12.5):
        u = mit.float_to_uint(value, LIMITS["P_MIN"], LIMITS["P_MAX"], 16)
        back = mit.uint_to_float(u, LIMITS["P_MIN"], LIMITS["P_MAX"], 16)
        assert back == pytest.approx(value, abs=step)


def test_float_to_uint_clamps_out_of_range():
    assert mit.float_to_uint(999.0, -12.5, 12.5, 16) == 65535
    assert mit.float_to_uint(-999.0, -12.5, 12.5, 16) == 0


def test_position_resolution_is_16_bit_over_range():
    # 25 rad span across 16 bits.
    assert mit.position_resolution(LIMITS) == pytest.approx(25.0 / 65535, rel=1e-9)


# --------------------------- command packing ----------------------------


def test_pack_command_length_and_layout():
    data = mit.pack_command(0.0, 0.0, 0.0, 0.0, 0.0, LIMITS)
    assert len(data) == 8

    # Reconstruct each field from the manual's bit layout.
    p_i = (data[0] << 8) | data[1]
    v_i = (data[2] << 4) | (data[3] >> 4)
    kp_i = ((data[3] & 0x0F) << 8) | data[4]
    kd_i = (data[5] << 4) | (data[6] >> 4)
    t_i = ((data[6] & 0x0F) << 8) | data[7]

    # 0.0 sits mid-range for the symmetric fields, and at 0 for the gains.
    assert p_i == pytest.approx(32767, abs=1)
    assert v_i == pytest.approx(2047, abs=1)
    assert t_i == pytest.approx(2047, abs=1)
    assert kp_i == 0
    assert kd_i == 0


def test_pack_command_roundtrips_through_layout():
    p, v, kp, kd, t = 1.5, -10.0, 40.0, 2.0, 1.25
    data = mit.pack_command(p, v, kp, kd, t, LIMITS)

    p_i = (data[0] << 8) | data[1]
    v_i = (data[2] << 4) | (data[3] >> 4)
    kp_i = ((data[3] & 0x0F) << 8) | data[4]
    kd_i = (data[5] << 4) | (data[6] >> 4)
    t_i = ((data[6] & 0x0F) << 8) | data[7]

    u2f = mit.uint_to_float
    assert u2f(p_i, LIMITS["P_MIN"], LIMITS["P_MAX"], 16) == pytest.approx(p, abs=1e-3)
    assert u2f(v_i, LIMITS["V_MIN"], LIMITS["V_MAX"], 12) == pytest.approx(v, abs=0.05)
    assert u2f(kp_i, 0.0, 500.0, 12) == pytest.approx(kp, abs=0.2)
    assert u2f(kd_i, 0.0, 5.0, 12) == pytest.approx(kd, abs=0.01)
    assert u2f(t_i, LIMITS["T_MIN"], LIMITS["T_MAX"], 12) == pytest.approx(t, abs=0.01)


def test_pack_command_gains_clamped_to_protocol_ceiling():
    # Kp > 500 / Kd > 5 must saturate, never wrap.
    data = mit.pack_command(0.0, 0.0, 9999.0, 9999.0, 0.0, LIMITS)
    kp_i = ((data[3] & 0x0F) << 8) | data[4]
    kd_i = (data[5] << 4) | (data[6] >> 4)
    assert kp_i == 4095
    assert kd_i == 4095


def test_pack_command_all_bytes_in_range():
    data = mit.pack_command(12.5, 45.5, 500.0, 5.0, 4.1, LIMITS)
    assert all(0 <= b <= 255 for b in data)


# --------------------------- reply parsing ------------------------------


def _build_reply(drive_id, p, v, t, temp, err):
    """Encode a reply frame the way the driver would."""
    p_i = mit.float_to_uint(p, LIMITS["P_MIN"], LIMITS["P_MAX"], 16)
    v_i = mit.float_to_uint(v, LIMITS["V_MIN"], LIMITS["V_MAX"], 12)
    t_i = mit.float_to_uint(t, LIMITS["T_MIN"], LIMITS["T_MAX"], 12)
    return bytes([
        drive_id & 0xFF,
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
        temp & 0xFF,
        err & 0xFF,
    ])


def test_parse_reply_roundtrip():
    frame = _build_reply(3, 1.25, -12.0, 2.0, 41, 0)
    drive_id, pos, vel, torque, temp, err = mit.parse_reply(frame, LIMITS)
    assert drive_id == 3
    assert pos == pytest.approx(1.25, abs=1e-3)
    assert vel == pytest.approx(-12.0, abs=0.05)
    assert torque == pytest.approx(2.0, abs=0.01)
    assert temp == 41
    assert err == 0


def test_parse_reply_negative_temperature():
    frame = _build_reply(1, 0.0, 0.0, 0.0, (-15) & 0xFF, 2)
    *_, temp, err = mit.parse_reply(frame, LIMITS)
    assert temp == -15
    assert err == 2


def test_parse_reply_rejects_bad_length():
    assert mit.parse_reply(b"\x00" * 7, LIMITS) is None
    assert mit.parse_reply(None, LIMITS) is None


def test_parse_reply_extracts_error_codes():
    for code, text in mit.ERROR_CODES.items():
        frame = _build_reply(1, 0.0, 0.0, 0.0, 20, code)
        *_, err = mit.parse_reply(frame, LIMITS)
        assert err == code
        assert mit.error_message(err) == text


def test_error_message_unknown_code():
    assert "Unknown" in mit.error_message(99)


# --------------------------- unit helpers -------------------------------


def test_torque_to_current_ak40_10():
    # 4.1 N.m / (0.056 * 10) = 7.32 A, matching the datasheet's 7.3 A peak.
    assert mit.torque_to_current(4.1, 0.056, 10) == pytest.approx(7.32, abs=0.02)


def test_torque_to_current_zero_guard():
    assert mit.torque_to_current(1.0, 0.0, 10) == 0.0


def test_deg_rad_roundtrip():
    for rad in (-math.pi, 0.0, 1.0, math.pi):
        assert mit.deg_to_rad(mit.rad_to_deg(rad)) == pytest.approx(rad, abs=1e-12)
