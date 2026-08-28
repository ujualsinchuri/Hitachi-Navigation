#!/usr/bin/env python3

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)

from launch.conditions import (
    IfCondition,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import (
    Node,
)


PACKAGE_NAME = "navigate_sys"

DOWN_CAMERA_TOPIC = (
    "/down_camera/image_raw"
)


def remove_element_if_present(
    model,
    xpath: str,
) -> None:

    element = model.find(
        xpath
    )

    if element is not None:
        model.remove(
            element
        )


def create_robot_sdf() -> str:
    """
    Add one webcam-like downward/forward-facing camera
    to TurtleBot3 Waffle Pi.

    Camera is used for:
    - red-line following
    - ArUco detection
    - localization

    Camera configuration approximates the physical USB
    webcam used on the real robot.
    """

    turtlebot_share = (
        get_package_share_directory(
            "turtlebot3_gazebo"
        )
    )

    source_sdf = os.path.join(
        turtlebot_share,
        "models",
        "turtlebot3_waffle_pi",
        "model.sdf",
    )

    if not os.path.isfile(
        source_sdf
    ):

        raise FileNotFoundError(
            f"TurtleBot3 SDF was not found: "
            f"{source_sdf}"
        )

    tree = ET.parse(
        source_sdf
    )

    root = tree.getroot()

    model = root.find(
        "model"
    )

    if model is None:

        raise RuntimeError(
            "TurtleBot3 SDF does not contain "
            "a <model> element."
        )

    # ---------------------------------------------------------
    # Remove previous injected camera
    # ---------------------------------------------------------

    remove_element_if_present(
        model,
        "./link[@name='down_camera_link']",
    )

    remove_element_if_present(
        model,
        "./joint[@name='down_camera_joint']",
    )

    remove_element_if_present(
        model,
        "./link[@name='inspection_camera_link']",
    )

    remove_element_if_present(
        model,
        "./joint[@name='inspection_camera_joint']",
    )

    # ---------------------------------------------------------
    # Create webcam-like camera
    # ---------------------------------------------------------

    down_camera_link = ET.fromstring(
        """
        <link name="down_camera_link">

          <!--
          Webcam position relative to base_link.

          x = 0.10 m forward
          y = 0.00 m centered
          z = 0.34 m above base

          Balanced mounting height for both line look-ahead and
          complete 0.20 m ArUco marker visibility.
          -->

          <pose>
            0.10 0.0 0.34
            0 0 0
          </pose>


          <inertial>

            <mass>
              0.01
            </mass>

            <inertia>

              <ixx>
                0.00001
              </ixx>

              <iyy>
                0.00001
              </iyy>

              <izz>
                0.00001
              </izz>

              <ixy>
                0.0
              </ixy>

              <ixz>
                0.0
              </ixz>

              <iyz>
                0.0
              </iyz>

            </inertia>

          </inertial>


          <!--
          Simple webcam body visualization.
          -->

          <visual name="down_camera_visual">

            <geometry>

              <box>

                <size>
                  0.080 0.035 0.030
                </size>

              </box>

            </geometry>


            <material>

              <ambient>
                0.03 0.03 0.03 1
              </ambient>

              <diffuse>
                0.03 0.03 0.03 1
              </diffuse>

            </material>

          </visual>


          <!--
          CAMERA SENSOR

          1.15 rad ~= 66 degrees downward from forward.

          This leaves about 24 degrees of forward look-ahead,
          giving the line follower enough preview for corners while
          keeping nearby 0.20 m ArUco markers in view.
          -->

          <sensor
            name="down_camera_sensor"
            type="camera">

            <pose>
              0 0 0
              0 1.15 0
            </pose>


            <always_on>
              true
            </always_on>

            <update_rate>
              25.0
            </update_rate>

            <visualize>
              false
            </visualize>


            <camera name="down_camera">

              <!--
              Approximate normal USB webcam FOV.

              1.36136 rad ~= 80 degrees.

              Wider than the 70 degree test so the full marker
              and the red line remain visible together.
              -->

              <horizontal_fov>
                1.36136
              </horizontal_fov>


              <!--
              Standard webcam resolution.
              -->

              <image>

                <width>
                  640
                </width>

                <height>
                  480
                </height>

                <format>
                  R8G8B8
                </format>

              </image>


              <clip>

                <near>
                  0.02
                </near>

                <far>
                  4.0
                </far>

              </clip>

            </camera>


            <plugin
              name="down_camera_plugin"
              filename="libgazebo_ros_camera.so">

              <ros>

                <namespace>
                  /
                </namespace>

              </ros>


              <camera_name>
                down_camera
              </camera_name>


              <frame_name>
                down_camera_link
              </frame_name>

            </plugin>

          </sensor>

        </link>
        """
    )


    # ---------------------------------------------------------
    # Attach camera rigidly to TurtleBot base
    # ---------------------------------------------------------

    down_camera_joint = ET.fromstring(
        """
        <joint
          name="down_camera_joint"
          type="fixed">

          <parent>
            base_link
          </parent>

          <child>
            down_camera_link
          </child>

        </joint>
        """
    )


    model.append(
        down_camera_link
    )

    model.append(
        down_camera_joint
    )


    # ---------------------------------------------------------
    # Write temporary modified TurtleBot SDF
    # ---------------------------------------------------------

    temporary_directory = (
        tempfile.mkdtemp(
            prefix=(
                "navigate_sys_"
                "hardware_camera_"
            )
        )
    )


    output_sdf = os.path.join(
        temporary_directory,
        (
            "turtlebot3_"
            "navigate_sys_"
            "hardware_camera.sdf"
        ),
    )


    tree.write(
        output_sdf,
        encoding="unicode",
        xml_declaration=False,
    )


    return output_sdf


def generate_launch_description() -> LaunchDescription:

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    package_share = (
        get_package_share_directory(
            PACKAGE_NAME
        )
    )


    gazebo_share = (
        get_package_share_directory(
            "gazebo_ros"
        )
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


    if not os.path.isfile(
        world_file
    ):

        raise FileNotFoundError(
            f"World file was not found: "
            f"{world_file}"
        )


    if not os.path.isfile(
        marker_csv
    ):

        raise FileNotFoundError(
            f"ArUco CSV was not found: "
            f"{marker_csv}"
        )


    # ---------------------------------------------------------
    # Create modified robot SDF
    # ---------------------------------------------------------

    robot_sdf = (
        create_robot_sdf()
    )


    start_line_follower = (
        LaunchConfiguration(
            "start_line_follower"
        )
    )


    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                gazebo_share,
                "launch",
                "gazebo.launch.py",
            )

        ),

        launch_arguments={
            "world":
                world_file,

            "verbose":
                "true",

        }.items(),

    )


    # ---------------------------------------------------------
    # Spawn TurtleBot3
    # ---------------------------------------------------------

    spawn_robot = Node(

        package="gazebo_ros",

        executable="spawn_entity.py",

        name=(
            "spawn_navigate_sys_turtlebot3"
        ),

        output="screen",

        arguments=[

            "-entity",
            "turtlebot3",

            "-file",
            robot_sdf,

            "-x",
            LaunchConfiguration(
                "x"
            ),

            "-y",
            LaunchConfiguration(
                "y"
            ),

            "-z",
            LaunchConfiguration(
                "z"
            ),

            "-Y",
            LaunchConfiguration(
                "yaw"
            ),

        ],

    )


    # ---------------------------------------------------------
    # Line follower
    # ---------------------------------------------------------

    line_follower = Node(

        package=PACKAGE_NAME,

        executable=(
            "line_follower"
        ),

        name=(
            "line_follower"
        ),

        output="screen",

        condition=IfCondition(
            start_line_follower
        ),

        parameters=[

            {

                "camera_topic":
                    DOWN_CAMERA_TOPIC,

                "cmd_vel_topic":
                    "/cmd_vel",

            }

        ],

    )


    # ---------------------------------------------------------
    # ArUco localization
    # ---------------------------------------------------------

    aruco_localizer = Node(

        package=PACKAGE_NAME,

        executable=(
            "aruco_localizer"
        ),

        name=(
            "aruco_localizer"
        ),

        output="screen",

        parameters=[

            {

                "camera_topic":
                    DOWN_CAMERA_TOPIC,

                "marker_csv":
                    marker_csv,

                # Trigger close/inspection earlier, before the
                # marker becomes too large or clipped.
                "minimum_marker_bottom_ratio":
                    0.48,

                "minimum_area_ratio":
                    0.0015,

                "same_marker_cooldown":
                    1.0,

            }

        ],

    )


    # ---------------------------------------------------------
    # Inspection manager
    # ---------------------------------------------------------

    inspection_manager = Node(

    package=PACKAGE_NAME,

    executable=(
        "inspection_manager"
    ),

    name=(
        "inspection_manager"
    ),

    output="screen",

    parameters=[

        {

            "stop_duration":
                4.0,

            "marker_csv":
                marker_csv,

            "marker_parent_frame":
                "odom",

        }

    ],

)


    # ---------------------------------------------------------
    # Localization monitor
    # ---------------------------------------------------------

    localization_monitor = Node(

        package=PACKAGE_NAME,

        executable=(
            "localization_monitor"
        ),

        name=(
            "localization_monitor"
        ),

        output="screen",

        parameters=[

            {

                "localized_threshold":
                    0.30,

            }

        ],

    )


    # ---------------------------------------------------------
    # Start perception/controller nodes after Gazebo settles
    # ---------------------------------------------------------

    delayed_nodes = TimerAction(

        period=5.0,

        actions=[

            line_follower,

            aruco_localizer,

            inspection_manager,

            localization_monitor,

        ],

    )


    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription(

        [

            DeclareLaunchArgument(

                "x",

                default_value=(
                    "-2.469052"
                ),

                description=(
                    "Robot starts on the long "
                    "straight line."
                ),

            ),


            DeclareLaunchArgument(

                "y",

                default_value=(
                    "-9.987839"
                ),

            ),


            DeclareLaunchArgument(

                "z",

                default_value=(
                    "0.05"
                ),

            ),


            DeclareLaunchArgument(

                "yaw",

                default_value=(
                    "0.079830"
                ),

                description=(
                    "Robot aligned with "
                    "straight path."
                ),

            ),


            DeclareLaunchArgument(

                "start_line_follower",

                default_value=(
                    "false"
                ),

            ),


            gazebo,

            spawn_robot,

            delayed_nodes,

        ]

    )
