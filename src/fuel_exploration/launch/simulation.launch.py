import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    fuel_exploration_dir = get_package_share_directory('fuel_exploration')
    fusion_bot_dir = get_package_share_directory('fusion_bot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    world = LaunchConfiguration('world')
    odom_topic = LaunchConfiguration('odom_topic')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')

    fusion_bot_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_bot_dir, 'launch', 'simulation.launch.py'),
        ),
        launch_arguments={
            'world': world,
            'x': x_pose,
            'y': y_pose,
            'z': z_pose,
        }.items(),
    )

    rtabmap_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_bot_dir, 'launch', 'rtabmap_slam.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'map_topic': '/map',
            'rtabmap_viz': rtabmap_viz,
            'rviz': rviz,
        }.items(),
    )

    exploration_nodes_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fuel_exploration_dir, 'launch', 'exploration_nodes.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'candidate_clearance_m': '0.10',
            'max_view_range_m': '3.5',
            'min_cluster_size': '10',
            'min_goal_distance_m': '0.25',
            'revisit_block_radius_m': '0.35',
            'goal_reached_tolerance_m': '0.25',
            'path_clearance_m': '0.15',
            'lookahead_distance': '0.25',
            'goal_tolerance': '0.20',
            'max_linear_speed': '0.22',
            'max_angular_speed': '1.6',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                turtlebot3_gazebo, 'worlds', 'turtlebot3_house.world'),
        ),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        fusion_bot_sim_cmd,
        rtabmap_slam_cmd,
        exploration_nodes_cmd,
    ])
