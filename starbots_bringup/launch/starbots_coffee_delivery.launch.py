import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory('starbots_bringup'),'rviz','starbots_dispenser.rviz')
    moveit_config = (
        MoveItConfigsBuilder("ur_manipulator", package_name="ur3e_moveit_config")
        .robot_description_semantic(file_path="config/ur3e.srdf")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": False},
        ],
    )
    manipulation_node = Node(
        name="starbots_delivery_server_node",
        package="starbots_manipulation",
        executable="starbots_delivery_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )
    delayed_manipulation_node = TimerAction(
        period=6.0, # seconds
        actions=[manipulation_node]
    )
    pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pointcloud_filter'),
                'launch',
                'depth_to_points.launch.py'
            )
        )
    )
    cup_holder_detection_pipeline_node = Node(
        package = 'starbots_detection',
        executable = 'cup_holder_pipeline',
        name = 'cup_holder_detection_pipeline',
        namespace='starbots_detection',
        output = 'screen',
        parameters = [{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'error'],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'error'],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(move_group_node)
    ld.add_action(pointcloud_launch)
    ld.add_action(cup_holder_detection_pipeline_node)
    ld.add_action(delayed_manipulation_node)
    ld.add_action(rviz_node)
    return ld