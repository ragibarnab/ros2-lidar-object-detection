from os import path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.join(path.dirname(launch_path), '..')

    lidar_object_detector_node = Node(
        package='lidar_object_detection',
        executable='lidar_object_detector_node',
        name='lidar_object_detector_node',
    )

    lidar_publisher_node = Node(
        package='lidar_object_detection',
        executable='lidar_publisher_node',
        name='lidar_publisher_node',
    )

    object3d_visualizer_node = Node(
        package='object_visualization',
        executable='object3d_visualizer_node',
        name='object3d_visualizer_node',
    )

    return LaunchDescription([
        lidar_object_detector_node,
        lidar_publisher_node,
        object3d_visualizer_node
    ])