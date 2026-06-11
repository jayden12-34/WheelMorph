from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleOp',
            executable='wheels',
            name='wheel_controller',
            output='screen',
        ),
        Node(
            package='teleOp',
            executable='legs',
            name='leg_controller',
            output='screen',
        ),
        Node(
            package='teleOp',
            executable='web_teleop',
            name='web_teleop',
            output='screen',
            on_exit=Shutdown(),
        ),
    ])
