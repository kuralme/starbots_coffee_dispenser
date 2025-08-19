from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name", package_name="ur3e_sim_moveit_config").to_moveit_configs()
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )
    manipulation_node = Node(
        name="pick_and_place_perception_node",
        package="ur3e_manipulation",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    # Delay action before launching the pick_and_place node
    delayed_manipulation_node = TimerAction(
        period=6.0,  # seconds
        actions=[manipulation_node]
    )

    # The object detection node to be launched first
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection.launch.py'
            )
        )
    )

    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(move_group_node)
    ld.add_action(object_detection_launch)
    ld.add_action(delayed_manipulation_node)
    return ld