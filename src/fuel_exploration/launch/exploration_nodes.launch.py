from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    path_topic = LaunchConfiguration('path_topic')
    trajectory_topic = LaunchConfiguration('trajectory_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    candidate_clearance_m = LaunchConfiguration('candidate_clearance_m')
    max_view_range_m = LaunchConfiguration('max_view_range_m')
    min_cluster_size = LaunchConfiguration('min_cluster_size')
    min_goal_distance_m = LaunchConfiguration('min_goal_distance_m')
    revisit_block_radius_m = LaunchConfiguration('revisit_block_radius_m')
    goal_reached_tolerance_m = LaunchConfiguration('goal_reached_tolerance_m')
    path_clearance_m = LaunchConfiguration('path_clearance_m')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    goal_tolerance = LaunchConfiguration('goal_tolerance')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('goal_topic', default_value='/next_exploration_goal'),
        DeclareLaunchArgument('path_topic', default_value='/next_exploration_path'),
        DeclareLaunchArgument(
            'trajectory_topic', default_value='/exploration_trajectory'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('candidate_clearance_m', default_value='0.10'),
        DeclareLaunchArgument('max_view_range_m', default_value='3.5'),
        DeclareLaunchArgument('min_cluster_size', default_value='10'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='0.35'),
        DeclareLaunchArgument('revisit_block_radius_m', default_value='0.45'),
        DeclareLaunchArgument('goal_reached_tolerance_m', default_value='0.25'),
        DeclareLaunchArgument('path_clearance_m', default_value='0.15'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.22'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.20'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.35'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.20'),
        Node(
            package='fuel_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'goal_topic': goal_topic,
                'path_topic': path_topic,
                'trajectory_topic': trajectory_topic,
                'candidate_clearance_m': candidate_clearance_m,
                'max_view_range_m': max_view_range_m,
                'min_cluster_size': min_cluster_size,
                'min_goal_distance_m': min_goal_distance_m,
                'revisit_block_radius_m': revisit_block_radius_m,
                'goal_reached_tolerance_m': goal_reached_tolerance_m,
                'path_clearance_m': path_clearance_m,
            }],
        ),
        Node(
            package='fuel_exploration',
            executable='exploration_controller',
            name='exploration_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'path_topic': path_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'lookahead_distance': lookahead_distance,
                'goal_tolerance': goal_tolerance,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
            }],
        ),
    ])
