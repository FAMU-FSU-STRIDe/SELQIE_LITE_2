import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Single place to tune Kp/Kd for every motor. Launch arguments below override
# anything set here, and gains can also be changed live at runtime by
# publishing to /motorN/set_gains -- see the file itself for the tuning guide.
MIT_GAINS_FILE = os.path.join(
    get_package_share_directory('actuation_bringup'), 'config', 'mit_gains.yaml')


def launch_setup(context, *args, **kwargs):
    motor_id = LaunchConfiguration('motor_id').perform(context)
    interface = LaunchConfiguration('interface').perform(context)
    motor_type = LaunchConfiguration('motor_type').perform(context)
    control_hz = LaunchConfiguration('control_hz').perform(context)
    auto_start = LaunchConfiguration('auto_start').perform(context)
    cmd_timeout = LaunchConfiguration('cmd_timeout').perform(context)
    reverse_polarity = LaunchConfiguration('reverse_polarity').perform(context)
    position_kp = LaunchConfiguration('position_kp').perform(context)
    position_kd = LaunchConfiguration('position_kd').perform(context)
    velocity_kd = LaunchConfiguration('velocity_kd').perform(context)
    torque_limit_scale = LaunchConfiguration('torque_limit_scale').perform(context)

    joint_name = f'motor{motor_id}'

    # Start from the shared gains file, then apply any per-launch overrides.
    overrides = {
        'can_interface': interface,
        'can_id': int(motor_id),
        'motor_type': motor_type,
        'joint_name': joint_name,
        'auto_start': auto_start.lower() in ('true', '1', 'yes'),
        'cmd_timeout': float(cmd_timeout),
        'reverse_polarity': reverse_polarity.lower() in ('true', '1', 'yes'),
    }
    # Only override a gain when it was actually given, so the YAML stays the
    # single source of truth unless someone deliberately overrides it.
    for name, value in (('control_hz', control_hz),
                        ('position_kp', position_kp),
                        ('position_kd', position_kd),
                        ('velocity_kd', velocity_kd),
                        ('torque_limit_scale', torque_limit_scale)):
        if value != '':
            overrides[name] = float(value)

    return [
        Node(
            package='cubemars_v2_ros',
            executable='motor_node',
            name=f'{joint_name}_node',
            output='screen',
            parameters=[MIT_GAINS_FILE, overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_id', default_value='0', description='Motor index (0-7).'
        ),
        DeclareLaunchArgument(
            'interface', default_value='can0', description='CAN interface name.'
        ),
        DeclareLaunchArgument(
            'motor_type', default_value='AK40-10', description='Cubemars motor type.'
        ),
        DeclareLaunchArgument(
            'control_hz', default_value='',
            description='Override the MIT command rate (Hz). Blank = use '
                        'mit_gains.yaml, which documents the CAN bus budget.',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Enter MIT mode automatically on startup.',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.5',
            description='Seconds without a command before the motor is released '
                        '(0 = disabled).',
        ),
        DeclareLaunchArgument(
            'reverse_polarity',
            default_value='false',
            description='Invert motor direction (true for inner shafts).',
        ),
        # Gain overrides. Empty means "use the value from mit_gains.yaml".
        DeclareLaunchArgument(
            'position_kp', default_value='',
            description='Override POSITION-mode Kp (stiffness). Blank = use mit_gains.yaml.',
        ),
        DeclareLaunchArgument(
            'position_kd', default_value='',
            description='Override POSITION-mode Kd (damping). Blank = use mit_gains.yaml.',
        ),
        DeclareLaunchArgument(
            'velocity_kd', default_value='',
            description='Override VELOCITY-mode Kd. Blank = use mit_gains.yaml.',
        ),
        DeclareLaunchArgument(
            'torque_limit_scale', default_value='',
            description='Scale the commanded torque limit (0-1). Blank = use mit_gains.yaml.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
