import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Dynamically find your package
    pkg_share = get_package_share_directory('hitachi_project_environment')
    
    # IMPORTANT: Ensure this filename perfectly matches the file in your worlds/ folder
    custom_world_path = os.path.join(pkg_share, 'worlds', 'transformer_world.world') 
    model_path = os.path.join(pkg_share, 'models')
    
    # 2. Tell Gazebo where to find your model and set the Waffle robot
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', model_path]
    )
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    #Define Rviz config file path
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'transformer_nav.rviz')

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}] # <-- CRITICAL FOR GAZEBO SYNC
    )

    # 3. Find gazebo_ros and turtlebot3_gazebo packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # paths to upstream launches
    gzserver_launch = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    robot_state_pub_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')
    spawn_turtlebot_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')

    # 4. Include gazebo server/client AND pass the custom world file!
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments={'world': custom_world_path}.items() # THIS FIXES THE GREY GRID!
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch)
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_state_pub_launch),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_turtlebot_launch),
        launch_arguments={'x_pose': '0.0', 'y_pose': '-5.0', 'z_pose': '0.01', }.items()
    )

    return LaunchDescription([
        set_model_path,
        set_tb3_model,
        gzserver,
        gzclient,
        robot_state_pub,
        spawn_turtlebot,
        start_rviz
    ])