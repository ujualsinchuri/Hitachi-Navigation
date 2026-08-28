from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('hitachi_hardware')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = {'use_sim_time': True}

    ld = LaunchDescription()

    ld.add_action(LogInfo(msg='Starting relocalization launch: static camera tf, aruco_read, qr_localisation, ekf'))

    # Static transform: camera_rgb_frame -> camera_optical_frame
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf',
        output='screen',
        parameters=[use_sim_time],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '-1.570796', '--pitch', '0', '--roll', '-1.570796',
            '--frame-id', 'camera_rgb_frame', '--child-frame-id', 'camera_optical_frame'
        ]
    )
    ld.add_action(camera_tf)

    # aruco detector node
    aruco_node = Node(
        package='hitachi_hardware',
        executable='aruco_read_hardware',
        name='aruco_read',
        output='screen',
        parameters=[use_sim_time],
    )
    ld.add_action(aruco_node)

    # qr_localisation node
    qr_node = Node(
        package='hitachi_hardware',
        executable='qr_localisation_hardware',
        name='qr_localisation',
        output='screen',
        parameters=[use_sim_time],
    )
    ld.add_action(qr_node)

    # EKF node (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, use_sim_time],
    )
    ld.add_action(ekf_node)

    return ld