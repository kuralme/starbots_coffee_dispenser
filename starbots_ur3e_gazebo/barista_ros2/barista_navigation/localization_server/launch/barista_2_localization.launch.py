import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    barista_2_config = os.path.join(get_package_share_directory(
        'localization_server'), 'config', 'barista_2_amcl_config.yaml')

    return LaunchDescription([

        Node(
            namespace='barista_2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[barista_2_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['barista_2/amcl']}]
        )
    ])
