#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('box_bot_gazebo')

    # Where the dummy barista description models of the robots are
    description_package_name = "box_bot_description"
    install_dir = get_package_prefix(description_package_name)

    # Where the dummy barsita description models of the robots are
    barista_description_package_name = "barista_description"
    barista_install_dir = get_package_prefix(barista_description_package_name)

    

    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')

    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(
        gazebo_plugins_name)

    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(
        gazebo_plugins_name)

    # The slot car plugin is inside here
    plugin_pkg = "rmf_robot_sim_gz_classic_plugins"
    plugin_dir = get_package_prefix(plugin_pkg)

    plugin_building_pkg = "rmf_building_sim_gz_classic_plugins"
    plugin_building_dir = get_package_prefix(plugin_building_pkg)

    # Actor plugin
    animated_actors_package = "animated_actors"
    animated_actors_lib_path = get_package_prefix(
        animated_actors_package)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + \
            '/share' + ':' + gazebo_models_path + ':' + barista_install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path + ':' + barista_install_dir + '/share'

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib' + ':' + \
            gazebo_plugins_name_path_install_dir + '/lib' + ':' + \
            plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + ':' + \
            plugin_building_dir + '/lib' + '/rmf_building_sim_gz_classic_plugins' + \
            ':' + animated_actors_lib_path + '/lib/' + animated_actors_package
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + \
            '/lib' + ':' + plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + \
            ':' + plugin_building_dir + '/lib' + '/rmf_building_sim_gz_classic_plugins' + \
            ':' + animated_actors_lib_path + '/lib/' + animated_actors_package

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_box_bot_gazebo, 'worlds', 'box_bot_box_restaurant_routes.world'), ''],
            description='SDF world file'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to the terminal.'
        ),
        gazebo
    ])
