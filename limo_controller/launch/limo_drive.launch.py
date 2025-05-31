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
    package_name = 'limo_controller'
       
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

    pos_cov_s_launch_arg = DeclareLaunchArgument('pos_cov_s', default_value='0.0')
    pos_cov_d_launch_arg = DeclareLaunchArgument('pos_cov_d', default_value='0.0')
    pos_cov_y_launch_arg = DeclareLaunchArgument('pos_cov_y', default_value='0.0')

    pos_cov_s = LaunchConfiguration('pos_cov_s')
    pos_cov_d = LaunchConfiguration('pos_cov_d')
    pos_cov_y = LaunchConfiguration('pos_cov_y')

    twist_cov_s_launch_arg = DeclareLaunchArgument('twist_cov_s', default_value='0.0')
    twist_cov_d_launch_arg = DeclareLaunchArgument('twist_cov_d', default_value='0.0')
    twist_cov_y_launch_arg = DeclareLaunchArgument('twist_cov_y', default_value='0.0')

    twist_cov_s = LaunchConfiguration('twist_cov_s')
    twist_cov_d = LaunchConfiguration('twist_cov_d')
    twist_cov_y = LaunchConfiguration('twist_cov_y')

    # Set node ======================================================================
    controller_opaque_function = OpaqueFunction(
        function=render_drive_type,
        args=[launch_description, drive_type]
    )

    path_tracking_opaque_function = OpaqueFunction(
        function=render_path_tracking,
        args=[launch_description, path_tracking]
    )

    odometry_single_track = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'single_track',
                   "--pos_cov", pos_cov_s,
                   "--twist_cov", twist_cov_s]
    )

    odometry_double_track = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'double_track',
                   "--pos_cov", pos_cov_d,
                   "--twist_cov", twist_cov_d]
    )

    odometry_yaw_rate = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'yaw_rate',
                   "--pos_cov", pos_cov_y,
                   "--twist_cov", twist_cov_y]
    )


    # Add the actions to the launch description  
    launch_description.add_action(limo_sim)

    launch_description.add_action(drive_type_launch_arg)
    launch_description.add_action(controller_opaque_function)
    launch_description.add_action(pos_cov_s_launch_arg)
    launch_description.add_action(pos_cov_d_launch_arg)
    launch_description.add_action(pos_cov_y_launch_arg)
    launch_description.add_action(twist_cov_s_launch_arg)
    launch_description.add_action(twist_cov_d_launch_arg)
    launch_description.add_action(twist_cov_y_launch_arg)
    launch_description.add_action(odometry_single_track)
    launch_description.add_action(odometry_double_track)
    launch_description.add_action(odometry_yaw_rate)

    launch_description.add_action(path_tracking_launch_arg)
    launch_description.add_action(path_tracking_opaque_function)

    return launch_description