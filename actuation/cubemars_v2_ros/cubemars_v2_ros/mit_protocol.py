#!/usr/bin/env python3
"""
CubeMars AK-series **MIT mode** CAN protocol.

Implements the MIT ("Mini-Cheetah") communication protocol from the
*AK Series Module Driver Manual* V1.0.18 §5.3, for the AK40-10 V2.0 driver.

Kept free of ROS / python-can imports so the packing, unpacking and unit
conversion can be unit-tested on their own. ``motor_node.py`` imports from here.

Why MIT mode
------------
The driver supports two CAN protocols. Servo mode (§5.1-5.2) runs its position
and velocity loops *inside* the driver, with gains reachable only over R-LINK --
nothing tunable is transmitted on the bus. MIT mode instead sends the gains in
**every frame**, so Kp/Kd can be changed live from ROS without a USB session.

The driver closes this loop onboard each control cycle (manual §5.3 block
diagram)::

    iq_ref = Kp * (p_des - p_meas) + Kd * (v_des - v_meas) + t_ff

then clamps ``iq_ref`` to the torque limit and hands it to the FOC current loop.
Setting Kp = Kd = 0 therefore gives pure feed-forward torque control; setting
t_ff = 0 with Kp > 0 gives position control with Kd as damping.

Frame formats (§5.3)
--------------------
Both directions are **CAN 2.0 standard (11-bit)** frames with DLC 8, at
**1 Mbit/s** -- note this differs from servo mode, which uses extended 29-bit
IDs. Command frames are addressed to the motor's ID; reply frames come back on
``0x00 + drive ID``.

Command (host -> driver), all fields big-endian bit-packed::

    DATA[0]  position  bits 15-8
    DATA[1]  position  bits 7-0
    DATA[2]  velocity  bits 11-4
    DATA[3]  velocity  bits 3-0  (high nibble) | Kp bits 11-8 (low nibble)
    DATA[4]  Kp        bits 7-0
    DATA[5]  Kd        bits 11-4
    DATA[6]  Kd        bits 3-0  (high nibble) | torque bits 11-8 (low nibble)
    DATA[7]  torque    bits 7-0

Reply (driver -> host)::

    DATA[0]  drive ID
    DATA[1]  position bits 15-8
    DATA[2]  position bits 7-0
    DATA[3]  velocity bits 11-4
    DATA[4]  velocity bits 3-0 (high nibble) | current bits 11-8 (low nibble)
    DATA[5]  current  bits 7-0
    DATA[6]  motor temperature
    DATA[7]  error code

.. note::
   The reply table in the manual mislabels DATA[2]-DATA[4] (it repeats "motor
   position" three times where speed and torque belong). The layout used here is
   the one the firmware actually implements and that the manual's own field
   widths imply: 16-bit position, 12-bit speed, 12-bit current.
"""

import math

# ===================== SPECIAL COMMAND CODES (§5.3) =====================
#
# Each is seven 0xFF bytes followed by the code. "When using CAN communication
# to control the motor, you must enter the motor MIT mode first!"

MIT_ENTER_MODE = 0xFC   # Enter motor control mode
MIT_EXIT_MODE = 0xFD    # Exit motor control mode
MIT_ZERO_POSITION = 0xFE  # Set current motor position as zero

# Sending the enter-mode frame also serves as a state request when the motor is
# otherwise idle (manual §5.3).
MIT_SPECIAL_PREFIX = b"\xff" * 7

# Field widths, in bits (§5.3 frame tables).
POS_BITS = 16
VEL_BITS = 12
KP_BITS = 12
KD_BITS = 12
TORQUE_BITS = 12

# Reply frames arrive on 0x00 + drive ID.
REPLY_ID_BASE = 0x00


def special_frame(code):
    """Build one of the 8-byte special command frames (enter/exit/zero)."""
    return MIT_SPECIAL_PREFIX + bytes([int(code) & 0xFF])


# ===================== SCALAR HELPERS =====================


def clamp(x, lo, hi):
    """Clamp ``x`` to the inclusive range ``[lo, hi]``."""
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x, lo, hi, bits):
    """Quantize a float in ``[lo, hi]`` to an unsigned integer of ``bits``.

    This is the MIT protocol's scaling: the range maps linearly onto the full
    unsigned range, so ``lo`` -> 0 and ``hi`` -> 2**bits - 1.

    Rounds to nearest rather than truncating. Every field here has a symmetric
    range, so the midpoint lands on a half code (2**bits - 1 is odd): truncating
    would push an exact 0.0 command one LSB below centre, which the motor decodes
    as a small negative value. For torque that is a standing bias the driver
    keeps regulating to even when the command is "limp".
    """
    span = hi - lo
    if span <= 0:
        return 0
    x = clamp(x, lo, hi)
    return int((x - lo) * ((1 << bits) - 1) / span + 0.5)


def uint_to_float(u, lo, hi, bits):
    """Inverse of :func:`float_to_uint`."""
    span = hi - lo
    if span <= 0:
        return lo
    u = clamp(int(u), 0, (1 << bits) - 1)
    return lo + (u * span) / ((1 << bits) - 1)


# ===================== COMMAND PACKING =====================


def pack_command(p_des, v_des, kp, kd, t_ff, limits):
    """Pack one MIT command frame.

    Args:
        p_des: desired position, rad at the output shaft
        v_des: desired velocity, rad/s at the output shaft
        kp: position gain (stiffness)
        kd: velocity gain (damping)
        t_ff: feed-forward torque, N.m at the output shaft
        limits: mapping with P_MIN/P_MAX, V_MIN/V_MAX, KP_MIN/KP_MAX,
            KD_MIN/KD_MAX, T_MIN/T_MAX -- every field is quantized against these,
            so they must match what the driver is configured for or the motor
            will decode different values than were intended.

    Returns:
        ``bytes`` of length 8, ready to send in a standard-ID CAN frame.
    """
    p_i = float_to_uint(p_des, limits["P_MIN"], limits["P_MAX"], POS_BITS)
    v_i = float_to_uint(v_des, limits["V_MIN"], limits["V_MAX"], VEL_BITS)
    kp_i = float_to_uint(kp, limits["KP_MIN"], limits["KP_MAX"], KP_BITS)
    kd_i = float_to_uint(kd, limits["KD_MIN"], limits["KD_MAX"], KD_BITS)
    t_i = float_to_uint(t_ff, limits["T_MIN"], limits["T_MAX"], TORQUE_BITS)

    return bytes([
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])


# ===================== REPLY PARSING =====================


def parse_reply(data, limits):
    """Parse an 8-byte MIT reply frame.

    Returns ``(drive_id, position_rad, velocity_rads, torque_nm, temp_c, error)``
    or ``None`` if the payload is not 8 bytes.

    Temperature is a signed value in degrees C; the error code is 0-7 (see
    :data:`ERROR_CODES`).
    """
    if data is None or len(data) != 8:
        return None

    drive_id = data[0]
    p_i = (data[1] << 8) | data[2]
    v_i = (data[3] << 4) | (data[4] >> 4)
    t_i = ((data[4] & 0x0F) << 8) | data[5]
    temp_c = data[6] - 256 if data[6] & 0x80 else data[6]
    error = data[7] & 0xFF

    position = uint_to_float(p_i, limits["P_MIN"], limits["P_MAX"], POS_BITS)
    velocity = uint_to_float(v_i, limits["V_MIN"], limits["V_MAX"], VEL_BITS)
    torque = uint_to_float(t_i, limits["T_MIN"], limits["T_MAX"], TORQUE_BITS)

    return drive_id, position, velocity, torque, temp_c, error


# ===================== ERROR CODES (§5.2.1, shared with servo mode) ==========

ERROR_CODES = {
    0: "No fault",
    1: "Motor over-temperature fault",
    2: "Over-current fault",
    3: "Over-voltage fault",
    4: "Under-voltage fault",
    5: "Encoder fault",
    6: "MOSFET over-temperature fault",
    7: "Motor stall",
}


def error_message(code):
    """Human-readable message for a motor error code."""
    return ERROR_CODES.get(code, f"Unknown error code: {code}")


# ===================== UNIT / RESOLUTION HELPERS =====================


def position_resolution(limits):
    """Smallest representable position step, rad (16-bit over the P range)."""
    return (limits["P_MAX"] - limits["P_MIN"]) / ((1 << POS_BITS) - 1)


def torque_to_current(torque_nm, kt, gear_ratio):
    """Output-shaft torque (N.m) -> motor phase current Iq (A).

    ``tau = Iq * Kt * gear_ratio``. Only needed for reporting -- MIT mode
    commands torque directly, unlike servo mode which commands current.
    """
    denom = kt * gear_ratio
    return 0.0 if denom == 0 else torque_nm / denom


def rad_to_deg(rad):
    """Radians -> degrees."""
    return rad * (180.0 / math.pi)


def deg_to_rad(deg):
    """Degrees -> radians."""
    return deg * (math.pi / 180.0)
