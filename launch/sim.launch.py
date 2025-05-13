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
    	package="lab1_mobile_ackermann",
    	executable=f"{drive_type_str}_drive.py"
    )

    launch_description.add_action(controller)

def generate_launch_description():

    # Launch Argument
    drive_type_launch_arg = DeclareLaunchArgument('type', default_value='ackermann')
    v_x_launch_arg = DeclareLaunchArgument('v_x', default_value='0.0')
    w_z_launch_arg = DeclareLaunchArgument('w_z', default_value='0.0')
    time_launch_arg = DeclareLaunchArgument('time', default_value='5.0')
    # odom_model_launch_arg = DeclareLaunchArgument('model', default_value='single_track')

    drive_type = LaunchConfiguration('type')
    v_x = LaunchConfiguration('v_x')
    w_z = LaunchConfiguration('w_z')
    time = LaunchConfiguration('time')
    # odom_model = LaunchConfiguration('model')
    
    package_name = "lab1_mobile_ackermann"
    rviz_file_name = "rviz_config.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "world.launch.py"
                )
            ]
        )
    )

    # Spawn the robot in the world
    spawn_x_val = "9.5"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "1.57"

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "limo"
            # "-x", spawn_x_val,
            # "-y", spawn_y_val,
            # "-z", spawn_z_val,
            # "-Y", spawn_yaw_val
        ],
        output = "screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )

    odometry_single_track = Node(
    	package="lab1_mobile_ackermann",
    	executable="odometry.py",
        arguments=["--model", 'single_track']
    )

    odometry_double_track = Node(
    	package="lab1_mobile_ackermann",
    	executable="odometry.py",
        arguments=["--model", 'double_track']
    )

    odometry_yaw_rate = Node(
    	package="lab1_mobile_ackermann",
    	executable="odometry.py",
        arguments=["--model", 'yaw_rate']
    )

    liveplotter = Node(
    	package="lab1_mobile_ackermann",
    	executable="liveplotter.py",
    )

    cmd_vel = Node(
    	package="lab1_mobile_ackermann",
    	executable="cmd_vel.py",
        arguments=["--v_x", v_x,
                   "--w_z", w_z,
                   "--time", time]
    )

    launch_description = LaunchDescription()

    controller_opaque_function = OpaqueFunction(
        function=render_drive_type,
        args=[launch_description, drive_type]
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velocity_controller_spawner,
                on_exit=[position_controller_spawner],
            )
        )
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(drive_type_launch_arg)
    launch_description.add_action(v_x_launch_arg)
    launch_description.add_action(w_z_launch_arg)
    launch_description.add_action(time_launch_arg)
    launch_description.add_action(rsp)
    launch_description.add_action(rviz)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(controller_opaque_function)
    launch_description.add_action(odometry_single_track)
    launch_description.add_action(odometry_double_track)
    launch_description.add_action(odometry_yaw_rate)
    launch_description.add_action(liveplotter)
    launch_description.add_action(cmd_vel)
    launch_description.add_action(world)
    return launch_description
