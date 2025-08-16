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
    my_barista_rmf_gazebo_path = get_package_share_directory(
        'barista_gazebo')

    # Where the dummy barsita description models of the robots are
    description_package_name = "dummy_barista_description"
    install_dir = get_package_prefix(description_package_name)

    # Add to path the models for Ingestors and Digestors
    extra_models_package = "barista_extra_models"
    extra_models_package_path = get_package_share_directory(
        extra_models_package)
    extra_models_path = os.path.join(extra_models_package_path, 'models')

    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(
        gazebo_plugins_name)

    # The slotcar plugin is inside here
    plugin_pkg = "rmf_robot_sim_gz_classic_plugins"
    plugin_dir = get_package_prefix(plugin_pkg)

    # Actor plugin
    animated_actors_package = "animated_actors"
    animated_actors_lib_path = get_package_prefix(
        animated_actors_package)

    #############################################
    # Panda Plugins and Models
    panda_description_package_name = "panda_robot_bringup"
    panda_install_dir = get_package_prefix(
        panda_description_package_name)

    # Plugins
    gazebo_ros2_control_name = "gazebo_ros2_control"
    gazebo_ros2_control_path_install_dir = get_package_prefix(
        gazebo_ros2_control_name)

    ###########################

    ##############
    # Barista
    barista_description_package_name = "barista_description"
    barista_description_install_dir = get_package_prefix(
        barista_description_package_name)
    ##############

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + \
            '/share' + ':' + extra_models_path + ':' + panda_install_dir + \
            "/share" + ':' + barista_description_install_dir + "/share"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + extra_models_path + ':' + panda_install_dir + \
            "/share" + ':' + barista_description_install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib' + ':' + \
            gazebo_plugins_name_path_install_dir + '/lib' + ':' + \
            plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + \
            ':' + animated_actors_lib_path + '/lib/' + animated_actors_package + ':' + \
            gazebo_ros2_control_path_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + \
            '/lib' + ':' + plugin_dir + '/lib' + '/rmf_robot_sim_gz_classic_plugins' + \
            ':' + animated_actors_lib_path + '/lib/' + animated_actors_package + ':' + \
            gazebo_ros2_control_path_install_dir + '/lib'

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
                my_barista_rmf_gazebo_path, 'worlds', 'panda_arm_animated_ingestor.world'), ''],
            description='SDF world file'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),
        gazebo
    ])
