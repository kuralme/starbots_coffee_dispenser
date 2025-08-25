from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name", package_name="ur3e_sim_moveit_config").sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("ur3e_sim_moveit_config"),
                "config/sensors_3d.yaml",
            )
        ).to_moveit_configs()
        
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
    return LaunchDescription(
        [move_group_node]
    )
