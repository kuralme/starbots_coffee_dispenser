from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur3e", package_name="ur3e_moveit_config")
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
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", "info"],
        # prefix=["xterm -e gdb -ex run --args"],
    )
    return LaunchDescription(
        [move_group_node]
    )
