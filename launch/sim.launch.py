#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
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
    spawn_x_val = '9.5'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '1.57'

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "limo"
            # '-x', spawn_x_val,
            # '-y', spawn_y_val,
            # '-z', spawn_z_val,
            # '-Y', spawn_yaw_val
        ],
        output = "screen"
    )

    controller = Node(
    	package="lab1_mobile_ackermann",
    	executable="ackermann_drive.py"
    )

    # locomotion = Node(
    # 	package="lab1_mobile_ackermann",
    # 	executable="locomotion.py"
    # )

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

    launch_description = LaunchDescription()

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
    launch_description.add_action(rsp)
    launch_description.add_action(rviz)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(controller)
    # launch_description.add_action(locomotion)
    launch_description.add_action(world)
    return launch_description
