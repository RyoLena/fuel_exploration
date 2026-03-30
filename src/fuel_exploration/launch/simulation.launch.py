import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    world = LaunchConfiguration('world')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo, 'models'),
    )

    # --- Gazebo (server only, no GUI) ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py'),
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'),
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # --- SLAM Toolbox ---
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # --- Exploration nodes ---
    frontier_detector = Node(
        package='fuel_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'candidate_clearance_m': 0.10,
            'max_view_range_m': 3.5,
            'min_goal_distance_m': 0.25,
            'revisit_block_radius_m': 0.35,
            'goal_reached_tolerance_m': 0.25,
            'path_clearance_m': 0.15,
        }],
    )

    exploration_controller = Node(
        package='fuel_exploration',
        executable='exploration_controller',
        name='exploration_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'lookahead_distance': 0.25,
            'goal_tolerance': 0.20,
            'max_linear_speed': 0.22,
            'max_angular_speed': 1.6,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                turtlebot3_gazebo, 'worlds', 'turtlebot3_house.world'),
        ),
        set_env_vars_resources,
        gzserver_cmd,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
        slam_toolbox_cmd,
        frontier_detector,
        exploration_controller,
    ])
