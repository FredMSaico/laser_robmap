from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo del filtro EKF de robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
            remappings=[
                ('/odom', '/wheel_odom_with_covariance'),
                ('/imu', '/vectornav/imu')
            ]
        ),
        # Publicador de transformaciones estáticas entre 'map' y 'odom'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_world_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),
        # Publicador de transformaciones estáticas entre 'base_link' y 'vectornav'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_vectornav',
            output='screen',
            arguments=['0.06', '0', '0', '0.707', '0.707', '0', '0', 'base_link', 'vectornav']
        ),
        # Publicador de transformaciones estáticas entre 'base_link' y 'base_scan'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_laser',
            output='screen',
            arguments=['-0.25', '0', '0', '0', '-0.7071', '0', '0.7071', 'base_link', 'base_scan']
        ),
    ])
