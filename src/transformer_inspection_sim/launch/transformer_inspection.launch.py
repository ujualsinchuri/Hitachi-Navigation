#!/usr/bin/env python3

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "transformer_inspection_sim"
DOWN_CAMERA_TOPIC = "/down_camera/image_raw"


def create_robot_sdf() -> str:
    turtlebot3_gazebo_share = get_package_share_directory(
        "turtlebot3_gazebo"
    )

    source_sdf = os.path.join(
        turtlebot3_gazebo_share,
        "models",
        "turtlebot3_waffle_pi",
        "model.sdf",
    )

    if not os.path.isfile(source_sdf):
        raise FileNotFoundError(
            f"TurtleBot3 model was not found: {source_sdf}"
        )

    tree = ET.parse(source_sdf)
    root = tree.getroot()
    model = root.find("model")

    if model is None:
        raise RuntimeError(
            "TurtleBot3 SDF does not contain a <model> element."
        )

    old_link = model.find(
        "./link[@name='down_camera_link']"
    )
    old_joint = model.find(
        "./joint[@name='down_camera_joint']"
    )

    if old_link is not None:
        model.remove(old_link)

    if old_joint is not None:
        model.remove(old_joint)

    camera_link = ET.fromstring(
        """
        <link name="down_camera_link">
          <pose>0.35 0.0 0.70 0 0 0</pose>

          <inertial>
            <mass>0.01</mass>
            <inertia>
              <ixx>0.00001</ixx>
              <iyy>0.00001</iyy>
              <izz>0.00001</izz>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyz>0</iyz>
            </inertia>
          </inertial>

          <sensor name="down_camera_sensor" type="camera">
            <pose>0 0 0 0 1.57079632679 0</pose>

            <always_on>true</always_on>
            <update_rate>20.0</update_rate>
            <visualize>false</visualize>

            <camera name="down_camera">
              <horizontal_fov>1.57079632679</horizontal_fov>

              <image>
                <width>800</width>
                <height>600</height>
                <format>R8G8B8</format>
              </image>

              <clip>
                <near>0.02</near>
                <far>5.0</far>
              </clip>
            </camera>

            <plugin
              name="down_camera_plugin"
              filename="libgazebo_ros_camera.so">

              <ros>
                <namespace>/</namespace>
              </ros>

              <camera_name>down_camera</camera_name>
              <frame_name>down_camera_link</frame_name>
            </plugin>
          </sensor>
        </link>
        """
    )

    camera_joint = ET.fromstring(
        """
        <joint name="down_camera_joint" type="fixed">
          <parent>base_link</parent>
          <child>down_camera_link</child>
        </joint>
        """
    )

    model.append(camera_link)
    model.append(camera_joint)

    temp_dir = tempfile.mkdtemp(
        prefix="tb3_transformer_inspection_"
    )

    output_sdf = os.path.join(
        temp_dir,
        "turtlebot3_waffle_pi_inspection.sdf",
    )

    tree.write(
        output_sdf,
        encoding="unicode",
        xml_declaration=False,
    )

    return output_sdf


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory(
        PACKAGE_NAME
    )

    gazebo_ros_share = get_package_share_directory(
        "gazebo_ros"
    )

    world_file = os.path.join(
        package_share,
        "worlds",
        "Transformer.world",
    )

    marker_csv = os.path.join(
        package_share,
        "config",
        "aruco_locations.csv",
    )

    if not os.path.isfile(world_file):
        raise FileNotFoundError(
            f"World file was not found: {world_file}"
        )

    if not os.path.isfile(marker_csv):
        raise FileNotFoundError(
            f"Marker CSV was not found: {marker_csv}"
        )

    robot_sdf = create_robot_sdf()

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    start_line_follower = LaunchConfiguration(
        "start_line_follower"
    )

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

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_turtlebot3",
        output="screen",
        arguments=[
            "-entity",
            "turtlebot3",
            "-file",
            robot_sdf,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-Y",
            yaw,
        ],
    )

    line_follower = Node(
        package=PACKAGE_NAME,
        executable="line_follower",
        name="transformer_line_follower",
        output="screen",
        condition=IfCondition(
            start_line_follower
        ),
        parameters=[
            {
                "camera_topic": DOWN_CAMERA_TOPIC,
                "cmd_vel_topic": "/cmd_vel",
            }
        ],
    )

    aruco_localizer = Node(
        package=PACKAGE_NAME,
        executable="aruco_localizer",
        name="aruco_localizer",
        output="screen",
        parameters=[
            {
                "camera_topic": DOWN_CAMERA_TOPIC,
                "marker_csv": marker_csv,
                "dictionary": "DICT_4X4_50",
                "minimum_marker_bottom_ratio": 0.58,
                "same_marker_cooldown": 1.0,
            }
        ],
    )

    inspection_manager = Node(
        package=PACKAGE_NAME,
        executable="inspection_manager",
        name="inspection_manager",
        output="screen",
        parameters=[
            {
                "stop_duration": 4.0,
            }
        ],
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            line_follower,
            aruco_localizer,
            inspection_manager,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "x",
                default_value="-2.70",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="-0.02",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "start_line_follower",
                default_value="false",
            ),
            gazebo,
            spawn_robot,
            delayed_nodes,
        ]
    )
