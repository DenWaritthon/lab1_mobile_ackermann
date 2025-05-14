#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro

def render_drive_type(context: LaunchContext, launch_description: LaunchDescription, drive_type: LaunchConfiguration):
    drive_type_str = context.perform_substitution(drive_type)

    controller = Node(
    	package="limo_controller",
    	executable=f"{drive_type_str}_drive.py"
    )

    launch_description.add_action(controller)

def generate_launch_description():

    # Launch Argument
    drive_type_launch_arg = DeclareLaunchArgument('type', default_value='ackermann')
    v_x_launch_arg = DeclareLaunchArgument('v_x', default_value='0.0')
    w_z_launch_arg = DeclareLaunchArgument('w_z', default_value='0.0')
    time_launch_arg = DeclareLaunchArgument('time', default_value='5.0')

    drive_type = LaunchConfiguration('type')
    v_x = LaunchConfiguration('v_x')
    w_z = LaunchConfiguration('w_z')
    time = LaunchConfiguration('time')
    
    package_name = "limo_controller"

    odometry_single_track = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'single_track']
    )

    odometry_double_track = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'double_track']
    )

    odometry_yaw_rate = Node(
    	package=package_name,
    	executable="odometry.py",
        arguments=["--model", 'yaw_rate']
    )

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

    launch_description = LaunchDescription()

    controller_opaque_function = OpaqueFunction(
        function=render_drive_type,
        args=[launch_description, drive_type]
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(drive_type_launch_arg)
    launch_description.add_action(v_x_launch_arg)
    launch_description.add_action(w_z_launch_arg)
    launch_description.add_action(time_launch_arg)
    launch_description.add_action(controller_opaque_function)
    launch_description.add_action(limo_sim)
    # launch_description.add_action(odometry_single_track)
    # launch_description.add_action(odometry_double_track)
    # launch_description.add_action(odometry_yaw_rate)
    # launch_description.add_action(liveplotter)
    # launch_description.add_action(cmd_vel)
    return launch_description