from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
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
            executable='teleop_receiver',
            name='teleop_receiver',
            output='screen',
            on_exit=Shutdown(),
        ),
        ExecuteProcess(
            cmd=['teleop_sender'],
            output='screen',
            on_exit=Shutdown(),
        ),
    ])
