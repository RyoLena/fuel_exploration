from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rtabmap_slam",
            executable='rtabmap',
            parameters=[{
                'use_sim_time':True,
                'subscribe_depth': False,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'frame_id': 'base_link',
            }],
            remappings=[
                ('scan_cloud', '/lidar/points'),
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('imu', '/imu'),
            ],
        )
    ])
