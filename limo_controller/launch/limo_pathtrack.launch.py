#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,  OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch.logging

def render_path_tracking(context: LaunchContext, launch_description: LaunchDescription, path_tracking: LaunchConfiguration):
    path_tracking_str = context.perform_substitution(path_tracking)

    if path_tracking_str == 'manual':
        path_tracking_node = Node(
        	package="teleop_twist_keyboard",
        	executable="teleop_twist_keyboard",
            output='screen',
            prefix='xterm -e'
        )
    else:
        path_tracking_node = Node(
        	package="limo_controller",
        	executable=f"path_tracking_{path_tracking_str}.py",
        )

    launch_description.add_action(path_tracking_node)

def generate_launch_description():

    path_record_node = Node(
        package="limo_controller",
        executable=f"pathplotter.py",
    )

    launch_description = LaunchDescription()

    # Declare a launch argument =====================================================
    path_tracking_launch_arg = DeclareLaunchArgument('tracking', default_value='manual')
    path_tracking = LaunchConfiguration('tracking')

    # Set node ======================================================================
    path_tracking_opaque_function = OpaqueFunction(
        function=render_path_tracking,
        args=[launch_description, path_tracking]
    )

    # Add the actions to the launch description  
    launch_description.add_action(path_tracking_launch_arg)
    launch_description.add_action(path_tracking_opaque_function)
    launch_description.add_action(path_record_node)

    return launch_description