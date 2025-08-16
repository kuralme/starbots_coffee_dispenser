import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    barista_1_config = os.path.join(get_package_share_directory(
        'localization_server'), 'config', 'barista_1_amcl_config.yaml')
    barista_2_config = os.path.join(get_package_share_directory(
        'localization_server'), 'config', 'barista_2_amcl_config.yaml')
    turtleE_1_config = os.path.join(get_package_share_directory(
        'localization_server'), 'config', 'turtleE_1_amcl_config.yaml')

    map_file = os.path.join(get_package_share_directory(
        'cartographer_slam'), 'config', 'starbots_with_warehouse_edited.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name': "map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_file}]
        ),

        Node(
            namespace='barista_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[barista_1_config]
        ),

        Node(
             namespace='barista_2',
             package='nav2_amcl',
             executable='amcl',
             name='amcl',
             output='screen',
             parameters=[barista_2_config]
        ),

        Node(
             namespace='turtleE_1',
             package='nav2_amcl',
             executable='amcl',
             name='amcl',
             output='screen',
             parameters=[turtleE_1_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'barista_1/amcl', 'barista_2/amcl','turtleE_1/amcl']}]
                        # {'node_names': ['map_server', 'barista_1/amcl']}]
        )
    ])
