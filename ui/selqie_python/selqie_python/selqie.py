import os
import bisect
import math
import time
from threading import Thread, Event, Lock
from typing import Optional
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, String, UInt32MultiArray
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from actuation_msgs.msg import MotorCommand
from motor_interfaces.msg import MotorState
from leg_control_msgs.msg import *
from robot_localization.srv import SetPose

def QOS_FAST() -> QoSProfile:
    """Get a QoSProfile with best-effort reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

def QOS_RELIABLE() -> QoSProfile:
    """Get a QoSProfile with reliable reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )

def QUAT2EUL(q : Quaternion) -> list[float]:
    """Convert a Quaternion message to Euler angles."""
    q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return [roll, pitch, yaw]

def EUL2QUAT(eul) -> Quaternion:
    """Convert Euler angles to a Quaternion message."""
    cy = math.cos(eul[2] * 0.5)
    sy = math.sin(eul[2] * 0.5)
    cp = math.cos(eul[1] * 0.5)
    sp = math.sin(eul[1] * 0.5)
    cr = math.cos(eul[0] * 0.5)
    sr = math.sin(eul[0] * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def _lerp_vector3(a: Vector3, b: Vector3, frac: float) -> Vector3:
    """Linearly interpolate between two Vector3 messages."""
    v = Vector3()
    v.x = a.x + (b.x - a.x) * frac
    v.y = a.y + (b.y - a.y) * frac
    v.z = a.z + (b.z - a.z) * frac
    return v

def _lerp_leg_command(a: 'LegCommand', b: 'LegCommand', frac: float) -> 'LegCommand':
    """Linearly interpolate two LegCommands. `control_mode` is categorical, so it
    is held from `a` rather than blended."""
    msg = LegCommand()
    msg.control_mode = a.control_mode
    msg.pos_setpoint = _lerp_vector3(a.pos_setpoint, b.pos_setpoint, frac)
    msg.vel_setpoint = _lerp_vector3(a.vel_setpoint, b.vel_setpoint, frac)
    msg.force_setpoint = _lerp_vector3(a.force_setpoint, b.force_setpoint, frac)
    return msg

def leg_trajectory_period(times: list[float]) -> float:
    """Cycle duration of a trajectory whose samples span ``times``.

    The samples cover [t_first, t_last], which is one sample-step short of a
    full cycle -- the next repetition's first sample belongs one step after
    t_last. Adding the average step back recovers the true period, so repeating
    the cycle is seamless instead of losing a step per repetition.
    """
    n = len(times)
    if n < 2:
        return 0.0
    span = times[-1] - times[0]
    return span + (span / (n - 1))


def resample_leg_trajectory(times: list[float], commands: list['LegCommand'],
                            resample_hz: float, max_points: int = 0):
    """Resample one leg's trajectory to a constant setpoint rate.

    A trajectory file stores a gait cycle as fixed timestamped setpoints (e.g.
    500 points/leg). Replaying those exact points after compressing the cycle
    to run at a higher frequency makes the required setpoint rate scale with
    frequency (rate = file_points * frequency), which can exceed what the CAN
    bus / motor nodes can carry -- at 5x on a 500-point/1s file that is 2500
    setpoints/s, well past the ~1500/s a 1 Mbps bus with 4 motors can sustain.

    Instead, this linearly interpolates the trajectory and resamples it to a
    constant ``resample_hz`` rate regardless of the original file's frequency.
    This bounds the delivered rate at *every* run frequency: fewer, evenly
    spaced points per cycle at high frequency, denser at low frequency -- so
    nothing is unevenly dropped or the stride cut short by the transport
    layer, and no single frequency is a bandwidth cliff.

    ``max_points`` (<=0 disables) additionally caps how many points a single
    cycle may contain, which bounds the serialized LegTrajectory message size.
    Rate alone does not: at a *low* run frequency the cycle is long, so a fixed
    rate produces proportionally more points (1000 Hz over a 2 s cycle is 2000
    points/leg, ~164 KB/leg, ~656 KB per republish across 4 legs). Those large
    messages take longer to serialize and push through DDS, and because the 4
    legs are published sequentially they land staggered -- which shows up as
    per-leg twitching at low frequency, since every republish resets the leg to
    the start of its stride. When the cap binds, the step is widened so the
    capped number of points still spans the whole cycle evenly.

    Capping costs little in motion quality, because what matters for smoothness
    is the position delta *per setpoint*, not the absolute rate: foot speed
    scales down with frequency, so a fixed points-per-cycle budget holds that
    delta roughly constant. The cap only ever binds at low frequency, where a
    1000-point budget still leaves the cycle denser than the source files
    (330-500 points). At high frequency the rate limit dominates instead and
    does intentionally go below source density -- that is the CAN bandwidth
    bound described above, and is the trade Option 1 exists to make.

    ``times`` must be sorted ascending (they are the already frequency-scaled
    per-cycle timestamps, i.e. what the caller would have used directly before
    resampling existed). Spacing does not need to be uniform -- the trajectory
    files are not perfectly uniform in practice (a few carry one slightly wider
    trailing interval), so this looks up the true bracketing samples for each
    new time rather than assuming a fixed step.

    The cycle is assumed to loop smoothly: motion just after the last sample
    continues into the first sample of the next repetition, one average
    original sample-step later. That closing gap is filled by blending the
    last sample toward the first. Returns ``(new_times, new_commands)``.
    """
    n = len(times)
    if n == 0:
        return [], []
    if n == 1 or resample_hz <= 0.0:
        return list(times), list(commands)

    t_start = times[0]
    t_end = times[-1]
    avg_step = (t_end - t_start) / (n - 1)
    period = leg_trajectory_period(times)  # includes the wrap-around gap

    step = 1.0 / resample_hz
    num_new = max(1, int(math.ceil(period / step)))

    # Cap the per-cycle point count and widen the step to match, so the capped
    # points still span the full cycle evenly rather than truncating it.
    if max_points > 0 and num_new > max_points:
        num_new = max_points
        step = period / num_new

    new_times = []
    new_commands = []
    for k in range(num_new):
        t = k * step
        if t >= period:
            break
        t_abs = t_start + t

        if t_abs <= t_end:
            # Find the bracketing original samples [i0, i1] via binary search;
            # spacing need not be uniform.
            i1 = bisect.bisect_right(times, t_abs)
            i1 = min(max(i1, 1), n - 1)
            i0 = i1 - 1
            span = times[i1] - times[i0]
            frac = 0.0 if span <= 0.0 else (t_abs - times[i0]) / span
        else:
            # Past the last real sample: blend toward the (identical) first
            # sample of the next cycle to close the loop smoothly.
            i0, i1 = n - 1, 0
            frac = 0.0 if avg_step <= 0.0 else (t_abs - t_end) / avg_step

        new_times.append(t_abs)
        new_commands.append(_lerp_leg_command(commands[i0], commands[i1], frac))

    return new_times, new_commands

class SELQIE(Node):
    """The main class for the SELQIE robot ROS2 interface."""

    ######################
    ### Initialization ###
    ######################

    def __init__(self, name="selqie"):
        super().__init__(name)
        self._stop_event = Event()
        self._battery_lock = Lock()
        
    def init(self):
        """Initialize all SELQIE components"""
        self.init_motors()
        self.init_legs()
        self.init_sensors()
        self.init_localization()
        self.init_mapping()
        self.init_control()
        self.init_vision()
        self.init_led()
        self.init_servo()
        self.init_recording()

    def init_motors(self):
        """Initialize the motor publishers and subscribers."""
        self.NUM_MOTORS = 8
        # Compatibility marker for older callers that inspect this attribute.
        # Motor gains are not read from this value; cubemars_v2_ros.motor_node
        # applies the position/velocity gains configured in the launch file.
        self.DEFAULT_MOTOR_GAINS = (0.0, 0.0)

        self._motor_position_gains = [list(self.DEFAULT_MOTOR_GAINS) for _ in range(self.NUM_MOTORS)]
        self._motor_cmd_publishers = []
        self._motor_special_publishers = []
        self._motor_gain_publishers = []
        self._motor_states = [MotorState() for _ in range(self.NUM_MOTORS)]
        self._motor_errors = [String() for _ in range(self.NUM_MOTORS)]
        self._motor_gains = [Float64MultiArray() for _ in range(self.NUM_MOTORS)]

        for i in range(self.NUM_MOTORS):
            self._motor_cmd_publishers.append(
                self.create_publisher(MotorCommand, f'/motor{i}/command', QOS_RELIABLE())
            )
            self._motor_special_publishers.append(
                self.create_publisher(String, f'/motor{i}/special_cmd', QOS_RELIABLE())
            )
            # MIT gains are settable at runtime; see set_motor_gains().
            self._motor_gain_publishers.append(
                self.create_publisher(Float64MultiArray, f'/motor{i}/set_gains', QOS_RELIABLE())
            )

            motor_gains_callback = lambda msg, i=i: self._motor_gains.__setitem__(i, msg)
            self.create_subscription(
                Float64MultiArray, f'/motor{i}/gains', motor_gains_callback, QOS_RELIABLE()
            )

            motor_state_callback = lambda msg, i=i: self._motor_states.__setitem__(i, msg)
            self.create_subscription(
                MotorState, f'/motor{i}/motor_state', motor_state_callback, QOS_FAST()
            )

            motor_error_callback = lambda msg, i=i: self._motor_errors.__setitem__(i, msg)
            self.create_subscription(
                String, f'/motor{i}/error_code', motor_error_callback, QOS_FAST()
            )

    def init_legs(self):
        """Initialize the leg publishers and subscribers."""
        self.LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
        self.NUM_LEGS = len(self.LEG_NAMES)
        self.DEFAULT_LEG_POSITION = [0.0, 0.0, -0.18914]
        self.TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('leg_trajectory_publisher'), 'trajectories')
        # Trajectory files loaded via get_leg_trajectories_from_file are resampled
        # to this constant setpoint rate (Hz), independent of the run frequency
        # (see resample_leg_trajectory). Keep this <= the cubemars motor nodes'
        # control_hz launch parameter, since a higher resample rate than the
        # motor node consumes buys nothing -- the node just samples its latest
        # cached command each control tick.
        self.TRAJECTORY_RESAMPLE_HZ = 500.0
        # Lead time (s) stamped onto a trajectory's start_time so all 4 legs
        # begin from one shared phase reference. It only has to cover the
        # spread of the 4 sequential publishes' arrival times.
        self.TRAJECTORY_START_DELAY = 0.1
        # Hard cap on resampled points per leg per cycle, which bounds the
        # serialized LegTrajectory message size (~84 bytes/point/leg, x4 legs
        # per republish). Rate alone does not bound size: a low run frequency
        # means a long cycle, so a fixed rate keeps adding points. Large
        # messages serialize/deliver slowly and, since the 4 legs are published
        # sequentially, land staggered -- which shows up as per-leg twitching at
        # low frequency. 1000 keeps every frequency at or above the source
        # files' own density (they carry 330-500 points/cycle).
        self.TRAJECTORY_MAX_POINTS = 1000

        self._leg_command_publishers = []
        for i in range(self.NUM_LEGS):
            self._leg_command_publishers.append(self.create_publisher(LegCommand, f'leg{self.LEG_NAMES[i]}/command', QOS_RELIABLE()))

        self._leg_estimates = [LegEstimate() for _ in range(self.NUM_LEGS)]
        self._leg_estimate_subscribers = []
        for i in range(self.NUM_LEGS):
            leg_estimate_callback = lambda msg, i=i: self._leg_estimates.__setitem__(i, msg)
            self._leg_estimate_subscribers.append(self.create_subscription(LegEstimate, f'leg{self.LEG_NAMES[i]}/estimate', leg_estimate_callback, QOS_FAST()))

        self._leg_trajectory_publishers = []
        for i in range(self.NUM_LEGS):
            self._leg_trajectory_publishers.append(self.create_publisher(LegTrajectory, f'leg{self.LEG_NAMES[i]}/trajectory', QOS_RELIABLE()))

    def init_sensors(self):
        """Initialize the sensor publishers and subscribers."""
        self._imu = Imu()
        imu_callback = lambda msg: setattr(self, '_imu', msg)
        self._imu_sub = self.create_subscription(Imu, 'imu', imu_callback, QOS_RELIABLE())

        self._pressure = Float32()
        pressure_callback = lambda msg: setattr(self, '_pressure', msg)
        self._pressure_sub = self.create_subscription(Float32, 'bar100/pressure', pressure_callback, QOS_RELIABLE())

        self._water_temperature = Float32()
        temperature_callback = lambda msg: setattr(self, '_water_temperature', msg)
        self._temperature_sub = self.create_subscription(Float32, 'bar100/temperature', temperature_callback, QOS_RELIABLE())
        
        self._battery_voltage: Optional[float] = None
        self._battery_voltage_stamp: Optional[float] = None
        
        self.create_subscription(
            Float32,
            '/tinybms/pack_voltage',
            self._on_battery_voltage,
            10,
        )
        
    def snapshot_battery_voltage(self) -> tuple[Optional[float], Optional[float]]:
        with self._battery_lock:
            return self._battery_voltage, self._battery_voltage_stamp
        
    def _on_battery_voltage(self, msg: Float32) -> None:
        with self._battery_lock:
            self._battery_voltage = float(msg.data)
            self._battery_voltage_stamp = time.time()

    def init_localization(self):
        """Initialize the localization publishers and subscribers."""
        self._odom = Odometry()
        odom_callback = lambda msg: setattr(self, '_odom', msg)
        self._odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, QOS_RELIABLE())

        self._set_pose_client = self.create_client(SetPose, 'set_pose')

        self._imu_calibrate_pub = self.create_publisher(Empty, 'imu/calibrate', QOS_RELIABLE())

    def init_mapping(self):
        """Initialize the mapping publishers and subscribers"""
        self._reset_map_pub = self.create_publisher(Empty, "map/reset", QOS_RELIABLE())

    def init_control(self):
        """Initialize the control publishers and subscribers."""
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QOS_RELIABLE())

        self._goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', QOS_RELIABLE())

        self._gait_pub = self.create_publisher(String, 'gait', QOS_RELIABLE())

        self._gait = String()
        gait_callback = lambda msg: setattr(self, '_gait', msg)
        self._gait_sub = self.create_subscription(String, 'gait', gait_callback, QOS_RELIABLE())

    def init_vision(self):
        """Initialize the camera and light publishers and subscribers."""
        self._lights_pwm_pub = self.create_publisher(Float32, 'lights/pwm', QOS_RELIABLE())

        self._camera_left_image = Image()
        camera_left_callback = lambda msg: setattr(self, '_camera_left_image', msg)
        self._camera_left_sub = self.create_subscription(Image, 'stereo/left/image_raw', camera_left_callback, QOS_FAST())

        self._camera_right_image = Image()
        camera_right_callback = lambda msg: setattr(self, '_camera_right_image', msg)
        self._camera_right_sub = self.create_subscription(Image, 'stereo/right/image_raw', camera_right_callback, QOS_FAST())

    def init_led(self):
        """Initialize the WS2812B LED publisher."""
        self._led_pub = self.create_publisher(UInt32MultiArray, 'led_colors', QOS_RELIABLE())

    def init_servo(self):
        """Initialize the latch servo publisher."""
        self._latch_pub = self.create_publisher(Bool, 'servo/latch', QOS_RELIABLE())

    def init_recording(self):
        motor_topics = [f'motor{i}/{suffix}'
                        for i in range(self.NUM_MOTORS)
                        for suffix in ('motor_state', 'error_code', 'command', 'special_cmd')]
        leg_topics = [f'leg{name}/{suffix}'
                      for name in self.LEG_NAMES
                      for suffix in ('command', 'estimate', 'trajectory')]
        gait_vel_estimate_topics = [f'vel_estimate/{gait}' for gait in ('walk', 'swim', 'jump', 'stand', 'sink')]

        self.ROSBAG_RECORD_TOPICS = motor_topics + leg_topics + [
            "stereo/left/image_raw", "stereo/right/image_raw", "lights/pwm",
            "imu/data", "imu/data/calibrated", "bar100/pressure", "bar100/temperature", "bar100/pose",
            "/tinybms/pack_voltage",
            "gait", "cmd_vel/raw", "cmd_vel", "goal_pose", "goal_pose/local",
            "gait/transition", "gait_planner/path", "walk_planner/path",
            "odom", "led_colors", "servo/latch",
        ] + gait_vel_estimate_topics
        self.ROSBAG_SAVE_FOLDER = '/home/selqie/rosbags'
        self._rosbag_process = None

    ########################
    ### ROS2 Spin Thread ###
    ########################
    
    def _spin_loop(self):
        """ROS2 spinning in a background thread."""
        while not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def spin(self):
        """Start the ROS2 spinning loop."""
        rclpy.spin(self)
        
    def spin_background(self):
        """Start the ROS2 spinning thread."""
        self._spin_thread = Thread(target=self._spin_loop)
        self._spin_thread.start()
        

    def stop(self):
        """Stop the ROS2 spinning thread and clean up."""
        self._stop_event.set()
        self._spin_thread.join()
        self.destroy_node()

    #######################
    ### Motor Functions ###
    #######################

    def send_motor_special_command(self, motor_idx : int, command : str):
        """Send a special command string to a motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        msg = String()
        msg.data = command
        self._motor_special_publishers[motor_idx].publish(msg)

    def set_motor_idle(self, motor_idx : int):
        """Place motor in idle mode."""
        self.send_motor_special_command(motor_idx, 'exit')

    def set_motor_ready(self, motor_idx : int):
        """Place motor in active MIT mode."""
        self.send_motor_special_command(motor_idx, 'start')

    def set_motor_clear_errors(self, motor_idx : int):
        """Clear motor faults and hold neutral command."""
        self.send_motor_special_command(motor_idx, 'clear')

    def set_motor_position_zero(self, motor_idx : int):
        """Set the motor's current Cubemars encoder position to zero."""
        self.send_motor_special_command(motor_idx, 'zero')

    def send_motor_command(self, motor_idx : int, position : float, velocity : float, kp : float, kd : float, torque : float):
        """Send a Cubemars MotorCommand; gains are owned by the motor launch file."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        _ = (kp, kd)  # Kept for API compatibility; motor_node uses launch-file gain parameters.
        cmd = MotorCommand()
        cmd.control_mode = MotorCommand.CONTROL_MODE_POSITION
        cmd.input_mode = MotorCommand.INPUT_MODE_PASSTHROUGH
        cmd.pos_setpoint = float(position)
        cmd.vel_setpoint = float(velocity)
        cmd.torq_setpoint = float(torque)
        self._motor_cmd_publishers[motor_idx].publish(cmd)

    def set_motor_position(self, motor_idx : int, pos : float):
        """Set motor position using the gains configured on the Cubemars launch file."""
        self.send_motor_command(motor_idx, pos, 0.0, 0.0, 0.0, 0.0)

    def set_motor_gains(self, motor_idx : int, kp : float, kd : float, _unused=None):
        """Set one motor's MIT gains live -- no relaunch, no R-LINK session.

        The driver applies, every control cycle:

            torque = kp * (pos_setpoint - pos_measured)
                   + kd * (vel_setpoint - vel_measured)
                   + torq_setpoint

        so ``kp`` is stiffness and ``kd`` is damping. Protocol ranges are
        kp 0-500 and kd 0-5; the motor node clips anything outside them. Values
        set this way are not persisted -- put anything you want to keep in
        ``actuation_bringup/config/mit_gains.yaml``.
        """
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        msg = Float64MultiArray()
        msg.data = [float(kp), float(kd)]
        self._motor_gain_publishers[motor_idx].publish(msg)

    def set_all_motor_gains(self, kp : float, kd : float):
        """Set the MIT gains on every motor at once."""
        for i in range(self.NUM_MOTORS):
            self.set_motor_gains(i, kp, kd)

    def get_motor_gains(self, motor_idx : int) -> list:
        """Latest gains reported by a motor: ``[kp, kd, velocity_kd]``."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        return list(self._motor_gains[motor_idx].data)

    def get_motor_info(self, motor_idx : int) -> String:
        """Get the latest motor error/status string message."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        return self._motor_errors[motor_idx]

    def get_motor_error_name(self, motor_idx : int) -> str:
        """Get latest human-readable error text for a motor."""
        return self.get_motor_info(motor_idx).data

    def get_motor_estimate(self, motor_idx : int) -> MotorState:
        """Get latest Cubemars MotorState for a motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        return self._motor_states[motor_idx]

    #####################
    ### Leg Functions ###
    #####################

    def send_leg_command(self, leg_idx : int, command : LegCommand):
        """Send a LegCommand message to the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        self._leg_command_publishers[leg_idx].publish(command)

    def set_leg_position(self, leg_idx : int, x : float, y : float, z : float):
        """Set the position of the leg."""
        command = LegCommand()
        command.control_mode = LegCommand.CONTROL_MODE_POSITION
        command.pos_setpoint.x = x
        command.pos_setpoint.y = y
        command.pos_setpoint.z = z
        self.send_leg_command(leg_idx, command)

    def set_leg_force(self, leg_idx : int, fx : float, fy : float, fz : float):
        """Set the force of the leg."""
        command = LegCommand()
        command.control_mode = LegCommand.CONTROL_MODE_FORCE
        command.force_setpoint.x = fx
        command.force_setpoint.y = fy
        command.force_setpoint.z = fz
        self.send_leg_command(leg_idx, command)

    def set_leg_position_default(self, leg_idx : int):
        """Set the leg to the default position."""
        self.set_leg_position(leg_idx, *self.DEFAULT_LEG_POSITION)

    def get_leg_estimate(self, leg_idx : int) -> LegEstimate:
        """Get the latest LegEstimate message from the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        return self._leg_estimates[leg_idx]
    
    def send_leg_trajectory(self, leg_idx : int, trajectory : LegTrajectory):
        """Send a LegTrajectory message to the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        self._leg_trajectory_publishers[leg_idx].publish(trajectory)
    
    def get_leg_trajectories_from_file(self, rel_file : str, frequency : float) -> list[LegTrajectory]:
        """Get a list of LegTrajectory messages from a file.

        The file's per-cycle setpoints are frequency-scaled and then resampled
        to a constant ``TRAJECTORY_RESAMPLE_HZ`` rate (see
        ``resample_leg_trajectory``), so the delivered setpoint rate stays
        bounded at any run frequency instead of scaling with it.
        """
        file = os.path.join(self.TRAJECTORIES_FOLDER, rel_file)
        if not os.path.exists(file):
            raise FileNotFoundError(f'File {file} does not exist')
        if frequency <= 0.0:
            raise ValueError(f'frequency must be positive, got {frequency}')

        raw_times = [[] for _ in range(self.NUM_LEGS)]
        raw_commands = [[] for _ in range(self.NUM_LEGS)]
        with open(file) as f:
            for line in f:
                parts = line.split()
                if len(parts) != 13:
                    raise ValueError(f'Invalid file line: {line}')
                leg_id = int(parts[1])
                if (leg_id >= self.NUM_LEGS) or (leg_id < 0):
                    raise ValueError(f'Expected leg ids between 0 and {self.NUM_LEGS - 1}')
                time = float(parts[0]) / 1000.0 / frequency
                msg = LegCommand()
                msg.control_mode = int(parts[2])
                msg.pos_setpoint.x = float(parts[4])
                msg.pos_setpoint.y = float(parts[5])
                msg.pos_setpoint.z = float(parts[6])
                msg.vel_setpoint.x = float(parts[7])
                msg.vel_setpoint.y = float(parts[8])
                msg.vel_setpoint.z = float(parts[9])
                msg.force_setpoint.x = float(parts[10])
                msg.force_setpoint.y = float(parts[11])
                msg.force_setpoint.z = float(parts[12])
                raw_times[leg_id].append(time)
                raw_commands[leg_id].append(msg)

        leg_trajectories = [LegTrajectory() for _ in range(self.NUM_LEGS)]
        for leg_id in range(self.NUM_LEGS):
            new_times, new_commands = resample_leg_trajectory(
                raw_times[leg_id], raw_commands[leg_id], self.TRAJECTORY_RESAMPLE_HZ,
                self.TRAJECTORY_MAX_POINTS)
            leg_trajectories[leg_id].timing = new_times
            leg_trajectories[leg_id].commands = new_commands
            # Tell the publisher the true cycle duration so it can repeat the
            # stride natively, phase-exactly, without the sender republishing.
            leg_trajectories[leg_id].period = leg_trajectory_period(new_times)
        return leg_trajectories

    def get_leg_trajectory_period(self, trajectories : list[LegTrajectory]) -> float:
        """Cycle duration (s) of a set of leg trajectories, 0.0 if unavailable."""
        for traj in trajectories:
            if traj is not None and traj.period > 0.0:
                return float(traj.period)
        return 0.0

    def run_leg_trajectories(self, trajectories : list[LegTrajectory],
                             loops : int = 1, start_delay : float = None):
        """Run a list of LegTrajectory messages.

        ``loops`` is played natively by leg_trajectory_publisher_node, so the
        stride repeats without the caller republishing once per cycle. That
        matters because every republish resets a leg to the start of its
        stride: doing it per cycle turns any timing jitter into a mid-stride
        snap-back (a twitch), whereas repeating in the publisher wraps only
        once the previous repetition has genuinely finished.

        ``start_delay`` seconds are added to the current time and stamped on
        every leg's message as a shared start instant. The 4 legs are published
        sequentially and each message takes non-zero time to serialize and
        traverse DDS, so without a common anchor each leg would begin at its
        own arrival time -- a phase stagger between legs. The delay just needs
        to cover the spread of those arrivals.
        """
        loops = max(1, int(loops))
        if start_delay is None:
            start_delay = self.TRAJECTORY_START_DELAY
        start_time = self.get_clock().now().nanoseconds / 1e9 + max(0.0, start_delay)
        for i in range(len(trajectories)):
            if trajectories[i] is not None:
                trajectories[i].loops = loops
                trajectories[i].start_time = start_time
                self.send_leg_trajectory(i, trajectories[i])

    ############################
    ### Sensor Data Functions ##
    ############################

    def get_imu(self) -> Imu:
        """Get the latest Imu message."""
        return self._imu
    
    def get_pressure(self) -> Float32:
        """Get the latest depth message."""
        return self._pressure
    
    def get_water_temperature(self) -> Float32:
        """Get the latest water temperature message."""
        return self._water_temperature

    ##############################
    ### Localization Functions ###
    ##############################

    def get_localization(self) -> Odometry:
        """Get the latest Odometry message."""
        return self._odom
    
    def send_localization_set_pose(self, pose : PoseWithCovarianceStamped):
        """Send a PoseWithCovarianceStamped message to the set_pose service."""
        req = SetPose.Request()
        req.pose = pose
        self._set_pose_client.call_async(req)

    def set_localization_pose(self, x : float, y : float, z : float, theta : float):
        """Set the pose of the robot."""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = z
        pose.pose.pose.orientation = EUL2QUAT([0.0, 0.0, theta])
        self.send_localization_set_pose(pose)

    def set_localization_pose_zero(self):
        """Set the pose of the robot to zero."""
        self.set_localization_pose(0.0, 0.0, 0.0, 0.0)

    def send_localization_calibrate_imu(self):
        """Send an Empty message to the imu/calibrate topic."""
        self._imu_calibrate_pub.publish(Empty())
    
    #########################
    ### Mapping Functions ###
    #########################

    def send_mapping_reset(self):
        self._reset_map_pub.publish(Empty())

    #########################
    ### Control Functions ###
    #########################

    def send_control_command_velocity(self, cmd_vel : Twist):
        """Send a Twist message to the cmd_vel topic."""
        self._cmd_vel_pub.publish(cmd_vel)

    def set_control_command_velocity(self, linear_x : float, linear_z : float, angular_z : float):
        """Set the linear x, z, and angular z velocities of the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.z = angular_z
        self.send_control_command_velocity(cmd_vel)
    
    def send_control_goal_pose(self, goal_pose : PoseStamped):
        """Send a PoseStamped message to the goal_pose topic."""
        self._goal_pose_pub.publish(goal_pose)

    def set_control_goal_pose(self, x : float, y : float, theta : float):
        """Set the goal pose of the robot."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        self.send_control_goal_pose(goal_pose)
    
    def send_control_gait(self, gait : String):
        """Send a String message to the gait topic."""
        self._gait_pub.publish(gait)
        
    def set_control_gait(self, gait : str):
        """Set the gait of the robot."""
        msg = String()
        msg.data = gait
        self.send_control_gait(msg)

    def get_control_gait(self) -> String:
        """Get the latest gait message."""
        return self._gait
    
    ########################
    ### Vision Functions ###
    ########################

    def send_vision_lights_pwm(self, pwm : Float32):
        """Send a Float32 message to the lights pwm topic."""
        if pwm.data < 0.0 or pwm.data > 100.0:
            raise ValueError(f"Invalid PWM value {pwm.data}")
        self._lights_pwm_pub.publish(pwm)

    def set_vision_lights_brightness(self, brightness : float):
        """Set the brightness of the lights."""
        pwm = Float32()
        pwm.data = (1100.0 + 8.0 * brightness) / 200.0
        self.send_vision_lights_pwm(pwm)

    def get_vision_camera_left(self) -> Image:
        """Get the latest image from the left camera."""
        return self._camera_left_image

    def get_vision_camera_right(self) -> Image:
        """Get the latest image from the right camera."""
        return self._camera_right_image

    ###################
    ### LED Functions ###
    ###################

    def set_led_color(self, r: int, g: int, b: int):
        """Set the WS2812B LED color. r, g, b in range 0-255."""
        packed = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)
        msg = UInt32MultiArray()
        msg.data = [packed]
        self._led_pub.publish(msg)

    def set_led_off(self):
        """Turn the WS2812B LED off."""
        self.set_led_color(0, 0, 0)

    ##########################
    ### Servo / Latch Functions ###
    ##########################

    def latch_open(self):
        """Open the latch servo."""
        msg = Bool()
        msg.data = True
        self._latch_pub.publish(msg)

    def latch_close(self):
        """Close the latch servo."""
        msg = Bool()
        msg.data = False
        self._latch_pub.publish(msg)

    ################################
    ### Data Recording Functions ###
    ################################

    def is_recording(self) -> bool:
        """Check if the rosbag recording process is running."""
        return self._rosbag_process is not None

    def start_recording(self, tag: Optional[str] = None):
        """Start recording rosbag data to the specified output folder.

        If provided, `tag` is appended to the timestamp-based bag name
        (e.g. to describe the gait, frequency, and number of loops being run).
        """
        if self.is_recording():
            return
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        bag_name = f'{timestamp}_{tag}' if tag else timestamp
        self._rosbag_process = subprocess.Popen(['ros2', 'bag', 'record', '-o',
                                                 os.path.join(self.ROSBAG_SAVE_FOLDER, bag_name)]
                                                 + self.ROSBAG_RECORD_TOPICS, stdin=subprocess.DEVNULL)

    def stop_recording(self):
        """Stop the rosbag recording process."""
        if not self.is_recording():
            return
        self._rosbag_process.terminate()
        self._rosbag_process.wait()
        self._rosbag_process = None
