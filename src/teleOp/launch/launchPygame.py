import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_BAG_DIR = os.path.join(
    os.path.expanduser('~/ros_bags'),
    'teleop_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'),
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag',
            default_value='false',
            description='Record a ROS bag (true/false)',
        ),
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
            cmd=['ros2', 'bag', 'record', '-a', '-o', _BAG_DIR],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),
        ExecuteProcess(
            cmd=['teleop_sender'],
            output='screen',
            on_exit=Shutdown(),
        ),
    ])
