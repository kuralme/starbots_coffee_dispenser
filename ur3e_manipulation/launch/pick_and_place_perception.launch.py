from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory('ur3e_manipulation'),'rviz','starbots_ur3e.rviz')
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
            {"ompl.planning_pipeline": "ompl"},
            {"planner_configs": ["RRTstarkConfigDefault"]},
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
            moveit_config.sensors_3d,
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
    )
    # Delay action before launching the pick_and_place node
    delayed_manipulation_node = TimerAction(
        period=6.0,  # seconds
        actions=[manipulation_node]
    )

    # Both detection nodes to be launched first
    detections_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
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
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    ld.add_action(move_group_node)
    ld.add_action(detections_launch)
    ld.add_action(delayed_manipulation_node)
    ld.add_action(rviz_node)
    return ld