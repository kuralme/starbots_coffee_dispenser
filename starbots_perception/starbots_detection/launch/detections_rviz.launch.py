import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    rviz_config = os.path.join(get_package_share_directory('starbots_detection'),'rviz','detections.rviz')

    pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pointcloud_filter'),
                'launch',
                'depth_to_points.launch.py'
            )
        )
    )
    cup_holder_detection_node = Node(
        package = 'starbots_detection',
        executable = 'cup_holder_detection',
        name = 'cup_holder_detection',
        output = 'screen',
        parameters = [{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        pointcloud_launch,
        cup_holder_detection_node,
        rviz_node
    ])