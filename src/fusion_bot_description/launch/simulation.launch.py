import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('fusion_bot_description')
    gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # World file (reuse TB3 house world)
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    default_world = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_house.world')

    # Model paths
    model_sdf = os.path.join(pkg_dir, 'models', 'fusion_bot', 'model.sdf')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'fusion_bot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()

    # Let Gazebo find our models + TB3 world models
    gz_model_path = os.path.join(pkg_dir, 'models')
    tb3_model_path = os.path.join(tb3_gazebo_dir, 'models')

    return LaunchDescription([
        # ---------- Arguments ----------
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='-0.5'),
        DeclareLaunchArgument('z', default_value='0.05'),

        # ---------- Environment ----------
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            ':'.join([gz_model_path, tb3_model_path]),
        ),

        # ---------- Gazebo Sim ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_dir, 'launch', 'gz_sim.launch.py'),
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', LaunchConfiguration('world')],
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # ---------- Spawn robot ----------
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', model_sdf,
                '-name', 'fusion_bot',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
            ],
            output='screen',
        ),

        # ---------- Robot State Publisher (TF from URDF) ----------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
            }],
            output='screen',
        ),

        # ---------- Clock Bridge (Gz 命名空间 → ROS /clock) ----------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[('/world/default/clock', '/clock')],
            output='screen',
        ),

        # ---------- Gz <-> ROS2 Bridge ----------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Drive
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # Joints (TF由robot_state_publisher+EKF接管，不从Gazebo桥接)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # 3D LiDAR (point cloud)
                '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # IMU
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # ---------- EKF: 融合 odom + IMU ----------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')],
            output='screen',
        ),
    ])
