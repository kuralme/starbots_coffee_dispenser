import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory('ur3e_manipulation'),'rviz','starbots_ur3e.rviz')
    moveit_config = (
        MoveItConfigsBuilder("ur_manipulator", package_name="ur3e_moveit_config")
        .robot_description_semantic(file_path="config/ur3e.srdf")
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
        package="ur3e_manipulation",
        executable="starbots_delivery_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
        # prefix=["xterm -e gdb -ex run --args"],
    )
    # Delay action before launching the starbots delivery node
    delayed_manipulation_node = TimerAction(
        period=6.0,  # seconds
        actions=[manipulation_node]
    )
    detections_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('starbots_detection'),
                'launch',
                'detections.launch.py'
            )
        )
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(move_group_node)
    # ld.add_action(detections_launch)
    ld.add_action(delayed_manipulation_node)
    ld.add_action(rviz_node)
    return ld