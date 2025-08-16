#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def gen_robot_info():

    # pose_1 = [13.64, -18.51, 1.57]
    # pose_1 = [13.58, -18.51, 1.57]
    pose_1 = [13.50, -18.51, 1.57]

    robot_name = "barista_1"
    x_pos = pose_1[0]
    y_pos = pose_1[1]
    yaw_pos = pose_1[2]
    robot = {'name': robot_name, 'x_pose': x_pos,
                    'y_pose': y_pos, 'z_pose': 0.1, 'Y_pose': yaw_pos}

    #print("############### ROBOTS MULTI ARRAY="+str(robot))

    return robot


def launch_setup(context, *args, **kwargs):

    launch_file_dir = os.path.join(get_package_share_directory(
        'barista_description'), 'launch')

    # Names and poses of the robots
    robot = gen_robot_info()

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_file_dir, 'spawn.launch.py')),
            launch_arguments={
                'x_spawn': TextSubstitution(text=str(robot['x_pose'])),
                'y_spawn': TextSubstitution(text=str(robot['y_pose'])),
                'yaw_spawn': TextSubstitution(text=str(robot['Y_pose'])),

                'entity_name': robot['name']
            }.items()))

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom_1',
        output='screen',
        emulate_tty=True,
        arguments=['-0.26', '0.05', '-0.54', '0', '0', '1.57', 'world', 'barista_1/odom']
    )

    return [ld,static_tf_pub]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
