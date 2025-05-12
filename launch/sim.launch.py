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

# def render_odometry_model(context: LaunchContext, launch_description: LaunchDescription, odom_model: LaunchConfiguration):
#     odom_model_str = context.perform_substitution(odom_model)

#     odometry = Node(
#     	package="lab1_mobile_ackermann",
#     	executable="odometry.py",
#         arguments=["--model", odom_model_str]
#     )

#     launch_description.add_action(odometry)

def generate_launch_description():

    # Launch Argument
    drive_type_launch_arg = DeclareLaunchArgument('type', default_value='ackermann')
    # odom_model_launch_arg = DeclareLaunchArgument('model', default_value='single_track')

    drive_type = LaunchConfiguration('type')
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

    launch_description = LaunchDescription()

    controller_opaque_function = OpaqueFunction(
        function=render_drive_type,
        args=[launch_description, drive_type]
    )

    # odometry_opaque_function = OpaqueFunction(
    #     function=render_odometry_model,
    #     args=[launch_description, odom_model]
    # )

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

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=position_controller_spawner,
    #             on_exit=[odometry_opaque_function],
    #         )
    #     )
    # )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(drive_type_launch_arg)
    # launch_description.add_action(odom_model_launch_arg)
    launch_description.add_action(rsp)
    launch_description.add_action(rviz)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(controller_opaque_function)
    # launch_description.add_action(odometry_opaque_function)
    launch_description.add_action(odometry_single_track)
    launch_description.add_action(odometry_double_track)
    launch_description.add_action(odometry_yaw_rate)
    launch_description.add_action(liveplotter)
    launch_description.add_action(world)
    return launch_description
