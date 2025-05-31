#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,  OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch.logging

def generate_launch_description():

    launch_description = LaunchDescription()
    package_name = 'limo_controller'

    # Declare a launch argument =====================================================
    v_x_launch_arg = DeclareLaunchArgument('v_x', default_value='0.0')
    w_z_launch_arg = DeclareLaunchArgument('w_z', default_value='0.0')
    time_launch_arg = DeclareLaunchArgument('time', default_value='5.0')

    v_x = LaunchConfiguration('v_x')
    w_z = LaunchConfiguration('w_z')
    time = LaunchConfiguration('time')

    # Set node ======================================================================
    liveplotter = Node(
    	package=package_name,
    	executable="liveplotter.py",
    )

    cmd_vel = Node(
    	package=package_name,
    	executable="cmd_vel.py",
        arguments=["--v_x", v_x,
                   "--w_z", w_z,
                   "--time", time]
    )

    # GPS Emulator Node
    gps_emulator_node = Node(
        package=package_name,
        executable='gps_emulator.py',
        name='gps_emulator_node',
    )

    #EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
        ]
    )

    # Add the actions to the launch description  

    launch_description.add_action(v_x_launch_arg)
    launch_description.add_action(w_z_launch_arg)
    launch_description.add_action(time_launch_arg)
    launch_description.add_action(liveplotter)
    launch_description.add_action(cmd_vel)
    launch_description.add_action(gps_emulator_node)
    launch_description.add_action(ekf_node)


    return launch_description