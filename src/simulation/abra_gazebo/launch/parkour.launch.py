#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_abra_gazebo = get_package_share_directory('abra_gazebo')
    pkg_abra_description = get_package_share_directory('abra_description')

    # Set the path to the WORLD model files.
    gazebo_models_path = os.path.join(pkg_abra_gazebo, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    ) 
    abra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_abra_description, 'launch', 'abra.launch.py'),
        )
    )

    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_abra_gazebo, 'launch', 'start_rviz.launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_abra_gazebo, 'worlds', 'parkour.world'), ''],
          description='SDF world file'),
        gazebo,
        abra,
        start_rviz
    ])
