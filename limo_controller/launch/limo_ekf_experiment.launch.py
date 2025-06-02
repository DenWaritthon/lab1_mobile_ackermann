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

def render_odometry_model(context: LaunchContext, launch_description: LaunchDescription, odom_model: LaunchConfiguration):
    odom_model_str = context.perform_substitution(odom_model)

    #EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('limo_controller'), 'config', f'ekf_{odom_model_str}.yaml')
        ]
    )

    launch_description.add_action(ekf_node)

def generate_launch_description():

    launch_description = LaunchDescription()
    package_name = 'limo_controller'

    # Declare a launch argument =====================================================
    path_tracking_launch_arg = DeclareLaunchArgument('tracking', default_value='manual')
    path_tracking = LaunchConfiguration('tracking')

    odom_model_launch_arg = DeclareLaunchArgument('model', default_value='single_track')
    odom_model = LaunchConfiguration('model')

    # Set node ======================================================================
    path_tracking_opaque_function = OpaqueFunction(
        function=render_path_tracking,
        args=[launch_description, path_tracking]
    )

    # GPS Emulator Node
    gps_emulator_node = Node(
        package=package_name,
        executable='gps_emulator.py',
        name='gps_emulator_node',
    )

    ekf_record_node = Node(
        package="limo_controller",
        executable=f"ekfplotter.py",
    )

    odometry_opaque_function = OpaqueFunction(
        function=render_odometry_model,
        args=[launch_description, odom_model]
    )

    # Add the actions to the launch description  
    launch_description.add_action(path_tracking_launch_arg)
    launch_description.add_action(odom_model_launch_arg)
    launch_description.add_action(path_tracking_opaque_function)
    launch_description.add_action(ekf_record_node)
    launch_description.add_action(gps_emulator_node)
    launch_description.add_action(odometry_opaque_function)

    return launch_description