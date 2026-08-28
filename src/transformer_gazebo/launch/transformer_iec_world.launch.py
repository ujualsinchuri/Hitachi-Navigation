#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("transformer_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    turtlebot3_gazebo_share = get_package_share_directory(
        "turtlebot3_gazebo"
    )
karmajkhfkljadhs
    world_file = os.path.join(
        package_share,
        "worlds",
        "transformer_iec.world",
    )

    robot_model = LaunchConfiguration("robot_model")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_z = LaunchConfiguration("z")
    spawn_yaw = LaunchConfiguration("yaw")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
        }.items(),
    )

    model_file = [
        turtlebot3_gazebo_share,
        "/models/turtlebot3_",
        robot_model,
        "/model.sdf",
    ]

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_turtlebot3",
        output="screen",
        arguments=[
            "-entity",
            "turtlebot3",
            "-file",
            model_file,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="waffle_pi",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="-1.80",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="-2.42",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
            ),
            gazebo,
            spawn_robot,
        ]
    )
