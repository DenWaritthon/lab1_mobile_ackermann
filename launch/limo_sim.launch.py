import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

import launch_ros.actions
import xacro

def generate_launch_description():
    package_name = "lab1_mobile_ackermann"
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory(package_name),
                        "launch",
                        "limo_description.launch.py"
                    )
                ]
            ),
            launch_arguments={"use_sim_time":"true"}.items()
        )

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": False}]
        )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    joint_trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    launch_description = LaunchDescription()

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
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_position_controller_spawner],
                )
            )
        )
    launch_description.add_action(rsp)
    
    return launch_description