import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    default_rtabmap_args = ' '.join([
        '--delete_db_on_start',
        '--Reg/Strategy', '1',
        '--Reg/Force3DoF', 'true',
        '--Mem/NotLinkedNodesKept', 'false',
        '--RGBD/CreateOccupancyGrid', 'true',
        '--RGBD/NeighborLinkRefining', 'true',
        '--Grid/3D', 'false',
        '--Grid/Sensor', '0',
        '--Grid/RayTracing', 'true',
        '--Grid/NormalsSegmentation', 'false',
        '--Grid/CellSize', '0.05',
        '--Grid/RangeMin', '0.25',
        '--Grid/RangeMax', '8.0',
        '--Grid/MinGroundHeight', '-0.05',
        '--Grid/MaxGroundHeight', '0.05',
        '--Grid/MaxObstacleHeight', '1.50',
        '--Grid/FootprintLength', '0.34',
        '--Grid/FootprintWidth', '0.24',
        '--Grid/FootprintHeight', '0.15',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_args', default_value=default_rtabmap_args),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py'),
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'namespace': 'rtabmap',
                'frame_id': 'base_link',
                'map_topic': LaunchConfiguration('map_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'imu_topic': '/imu',
                'rgb_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'depth': 'false',
                'subscribe_rgb': 'true',
                'subscribe_scan_cloud': 'true',
                'scan_cloud_topic': '/lidar/points',
                'visual_odometry': 'false',
                'icp_odometry': 'false',
                'approx_sync': 'true',
                'rtabmap_args': LaunchConfiguration('rtabmap_args'),
            }.items(),
        ),
    ])
