import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    rviz_config = os.path.join(get_package_share_directory('object_detection'),'rviz','hole_detection.rviz')

    hole_detection_node = Node(
        package = 'object_detection',
        executable = 'hole_detection',
        name = 'hole_detection',
        output = 'screen',
        parameters = [{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        hole_detection_node,
        rviz_node
    ])