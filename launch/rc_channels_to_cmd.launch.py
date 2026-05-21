#!/usr/bin python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    static_component_container = Node(
        package='rc_channels_to_cmd',
        executable='RcChannelsToCmdExecutable',
        output='screen',
        sigterm_timeout = '30',
        parameters=[
            os.path.join(get_package_share_directory('rc_channels_to_cmd'), 'params', 'params.yaml')
        ]
    )
    
    return LaunchDescription([
        static_component_container,
    ])
