#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,  OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch.logging

def render_drive_type(context: LaunchContext, launch_description: LaunchDescription, drive_type: LaunchConfiguration):
    drive_type_str = context.perform_substitution(drive_type)

    controller = Node(
    	package="limo_controller",
    	executable=f"{drive_type_str}_drive.py"
    )

    launch_description.add_action(controller)

def render_path_tracking(context: LaunchContext, launch_description: LaunchDescription, path_tracking: LaunchConfiguration):
    path_tracking_srt = context.perform_substitution(path_tracking)

    if path_tracking_srt == 'manual':
        path_tracking_node = Node(
        	package="teleop_twist_keyboard",
        	executable="teleop_twist_keyboard",
            output='screen',
            prefix='xterm -e'
        )
    else:
        path_tracking_node = Node(
        	package="limo_controller",
        	executable=f"path_tracking_{path_tracking_srt}.py",
        )

    launch_description.add_action(path_tracking_node)

def generate_launch_description():

    launch_description = LaunchDescription()
       
    # Spawn the robot in the world =================================================
    limo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('limo_model'),
                    "launch",
                    "limo_sim.launch.py"
                )
            ]
        )
    )

    # Declare a launch argument =====================================================
    drive_type_launch_arg = DeclareLaunchArgument('type', default_value='ackermann')
    drive_type = LaunchConfiguration('type')

    path_tracking_launch_arg = DeclareLaunchArgument('tracking', default_value='manual')
    path_tracking = LaunchConfiguration('tracking')

    # Set node ======================================================================
    controller_opaque_function = OpaqueFunction(
        function=render_drive_type,
        args=[launch_description, drive_type]
    )

    path_tracking_opaque_function = OpaqueFunction(
        function=render_path_tracking,
        args=[launch_description, path_tracking]
    )

    # Add the actions to the launch description  
    launch_description.add_action(limo_sim)

    launch_description.add_action(drive_type_launch_arg)
    launch_description.add_action(controller_opaque_function)

    launch_description.add_action(path_tracking_launch_arg)
    launch_description.add_action(path_tracking_opaque_function)

    return launch_description