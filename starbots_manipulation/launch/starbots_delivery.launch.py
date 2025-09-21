import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur_manipulator", package_name="ur3e_sim_moveit_config")
        .robot_description_semantic(file_path="config/name.srdf")
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
            {"use_sim_time": True},
        ],
    )
    manipulation_node = Node(
        name="starbots_delivery_server_node",
        package="ur3e_manipulation",
        executable="starbots_delivery_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )
    # Delay action before launching the starbots delivery node
    delayed_manipulation_node = TimerAction(
        period=6.0,  # seconds
        actions=[manipulation_node]
    )

    # Detection nodes to be launched
    detections_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('starbots_detection'),
                'launch',
                'detections.launch.py'
            )
        )
    )

    ld = LaunchDescription()
    ld.add_action(move_group_node)
    ld.add_action(detections_launch)
    ld.add_action(delayed_manipulation_node)
    return ld