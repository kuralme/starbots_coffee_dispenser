import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller_yaml_barista_2 = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'controller_barista_2.yaml')
    bt_navigator_yaml_barista_2 = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'bt_navigator_barista_2.yaml')
    planner_yaml_barista_2 = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'planner_server_barista_2.yaml')
    recovery_yaml_barista_2 = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'recovery_barista_2.yaml')

    return LaunchDescription([

        # Nodes for barista_2

        Node(
            namespace='barista_2',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_barista_2]),

        Node(
            namespace='barista_2',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_barista_2]),

        Node(
            namespace='barista_2',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml_barista_2],
            output='screen'),

        Node(
            namespace='barista_2',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_barista_2]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='barista_2_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': [
                            'barista_2/planner_server',
                            'barista_2/controller_server',
                            'barista_2/behavior_server',
                            'barista_2/bt_navigator'
                        ]}])

    ])
