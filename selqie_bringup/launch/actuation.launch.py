import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

CAN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'can.launch.py')

CUBEMARS_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'cubemars.launch.py')

# ── MIT-mode notes ────────────────────────────────────────────────────────────
# The CubeMars motors run in MIT mode, so Kp/Kd are sent in every CAN frame
# rather than configured over R-LINK. There are no gain constants here on
# purpose: they live in actuation_bringup/config/mit_gains.yaml, which every
# motor node loads, and can also be retuned live from selqie_terminal
# (`set_gains`) without restarting anything.
# ──────────────────────────────────────────────────────────────────────────────


def CanLaunch(interface: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CAN_LAUNCH_FILE),
        launch_arguments={'interface': interface}.items()
    )


def CubemarsLaunch(motor_id: str, interface: str, reverse_polarity: str = 'false'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CUBEMARS_LAUNCH_FILE),
        launch_arguments={
            'motor_id':         motor_id,
            'interface':        interface,
            'reverse_polarity': reverse_polarity,
        }.items()
    )


def InnerShaft(motor_id: str, interface: str):
    return CubemarsLaunch(motor_id, interface)


def OuterShaft(motor_id: str, interface: str):
    return CubemarsLaunch(motor_id, interface, reverse_polarity='false')


def generate_launch_description():
    return LaunchDescription([
        CanLaunch('can0'),
        CanLaunch('can1'),
        InnerShaft('0', 'can0'),   # FL inner
        OuterShaft('1', 'can0'),   # FL outer
        InnerShaft('2', 'can1'),   # RL inner
        OuterShaft('3', 'can1'),   # RL outer
        InnerShaft('4', 'can1'),   # RR inner
        OuterShaft('5', 'can1'),   # RR outer
        InnerShaft('6', 'can0'),   # FR inner
        OuterShaft('7', 'can0'),   # FR outer
    ])
