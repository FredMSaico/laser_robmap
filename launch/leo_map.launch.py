import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('laser_robmap'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='laser_robmap',
            executable='laserscan_projector',
            name='laserscan_projector',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='laser_robmap',
            executable='map_generator',
            name='map_generator',
            output='screen',
            parameters=[config]
        )
    ])
