from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up joystick teleop for SELQIE.

    Runs the ``selqie_joystick`` controller, and (optionally) a ``joy_node`` to
    publish ``/joy`` from a locally-connected controller. On a split setup, run
    ``joy_node`` on the client device and launch this with ``start_joy:=false``.
    """
    start_joy = DeclareLaunchArgument(
        'start_joy', default_value='true',
        description='Also start a joy_node for a locally-connected controller.'
    )
    joy_dev = DeclareLaunchArgument(
        'joy_dev', default_value='0',
        description='Joystick device id for joy_node (/dev/input/jsN).'
    )
    deadzone = DeclareLaunchArgument(
        'joy_deadzone', default_value='0.05',
        description='joy_node analog deadzone (the node applies its own too).'
    )

    return LaunchDescription([
        start_joy,
        joy_dev,
        deadzone,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_joy')),
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'deadzone': LaunchConfiguration('joy_deadzone'),
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='selqie_ui',
            executable='selqie_joystick',
            name='selqie_joystick',
            output='screen',
        ),
    ])
