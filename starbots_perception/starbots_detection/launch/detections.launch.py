import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    pointcloud_filter_node = Node(
        package = 'pointcloud_filter',
        executable = 'pointcloud_filter_node',
        name = 'pointcloud_filter',
        output = 'screen',
    )

    cup_detection_node = Node(
        package = 'starbots_detection',
        executable = 'cup_detection',
        name = 'cup_detection',
        output = 'screen',
        parameters = [{'use_sim_time': True}],
    )

    cup_holder_detection_node = Node(
        package = 'starbots_detection',
        executable = 'cup_holder_detection',
        name = 'cup_holder_detection',
        output = 'screen',
        parameters = [{'use_sim_time': True}],
    )

    return LaunchDescription([
        pointcloud_filter_node,
        cup_detection_node,
        cup_holder_detection_node,
    ])