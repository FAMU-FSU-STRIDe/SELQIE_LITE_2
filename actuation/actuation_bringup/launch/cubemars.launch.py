from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    motor_id = LaunchConfiguration('motor_id').perform(context)
    interface = LaunchConfiguration('interface').perform(context)
    motor_type = LaunchConfiguration('motor_type').perform(context)
    control_hz = LaunchConfiguration('control_hz').perform(context)
    auto_start = LaunchConfiguration('auto_start').perform(context)
    pole_pairs = LaunchConfiguration('pole_pairs').perform(context)
    gear_ratio = LaunchConfiguration('gear_ratio').perform(context)
    position_mode = LaunchConfiguration('position_mode').perform(context)
    pos_spd_accel = LaunchConfiguration('pos_spd_accel').perform(context)
    pos_spd_min_speed = LaunchConfiguration('pos_spd_min_speed').perform(context)
    cmd_timeout = LaunchConfiguration('cmd_timeout').perform(context)
    reverse_polarity = LaunchConfiguration('reverse_polarity').perform(context)

    joint_name = f'motor{motor_id}'

    return [
        Node(
            package='cubemars_v2_ros',
            executable='motor_node',
            name=f'{joint_name}_node',
            output='screen',
            parameters=[{
                'can_interface': interface,
                'can_id': int(motor_id),
                'motor_type': motor_type,
                'control_hz': float(control_hz),
                'joint_name': joint_name,
                'auto_start': auto_start.lower() in ('true', '1', 'yes'),
                'pole_pairs': int(pole_pairs),
                'gear_ratio': float(gear_ratio),
                'position_mode': position_mode,
                'pos_spd_accel': float(pos_spd_accel),
                'pos_spd_min_speed': float(pos_spd_min_speed),
                'cmd_timeout': float(cmd_timeout),
                'reverse_polarity': reverse_polarity.lower() in ('true', '1', 'yes'),
            }],
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
            'control_hz',
            default_value='500.0',
            description='Control loop / setpoint stream rate in Hz. Higher = finer, smoother '
                        'position streaming for plain SET_POS.',
        ),
        DeclareLaunchArgument(
            'auto_start', default_value='false', description='Enable motor automatically.'
        ),
        DeclareLaunchArgument(
            'pole_pairs',
            default_value='0',
            description='Rotor pole pairs for ERPM (velocity) scaling. 0 = per-motor default.',
        ),
        DeclareLaunchArgument(
            'gear_ratio',
            default_value='0.0',
            description='Gearbox reduction for ERPM/torque scaling. 0 = per-motor default.',
        ),
        DeclareLaunchArgument(
            'position_mode',
            default_value='pos_spd',
            description="Startup POSITION streaming: 'pos_spd' (feed-forward, smooth) or 'pos' "
                        "(plain). Switchable at runtime via the pos/pos_spd special commands.",
        ),
        DeclareLaunchArgument(
            'pos_spd_accel',
            default_value='327670.0',
            description='Acceleration limit (ERPM/s) for pos_spd streaming; protocol max ~327670.',
        ),
        DeclareLaunchArgument(
            'pos_spd_min_speed',
            default_value='2.0',
            description='Minimum approach speed (rad/s) for pos_spd so held poses (stand) move.',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.5',
            description='Seconds without a command before motor is released (0 = disabled).',
        ),
        DeclareLaunchArgument(
            'reverse_polarity',
            default_value='false',
            description='Invert motor direction (true for inner shafts).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
