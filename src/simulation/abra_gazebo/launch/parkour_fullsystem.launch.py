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
    pkg_lidar_localization_ros2 = get_package_share_directory("pcl_localization_ros2")
    pkg_stanley_controller = get_package_share_directory("abra_stanley_controller")
    pkg_abra_fsm_planner = get_package_share_directory("abra_fsm_planner")

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
            os.path.join(pkg_abra_gazebo, 'launch', 'start_fullsystem_rviz.launch.py'),
        )
    )

    lidar_localization_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar_localization_ros2, 'launch', 'pcl_localization.launch.py'),
        )
    )

    abra_localization = Node(
        package = "abra_localizer",
        executable = "abra_localizer_node",
        name = "abra_localizer",
        output="screen"
    )

    stanley_controller = Node(
        package = "abra_stanley_controller",
        executable = "abra_stanley_controller_node",
        name = "stanley_controller",
        output="screen"
    )

    abra_fsm_planner = Node(
        package = "abra_fsm_planner",
        executable = "abra_fsm_planner_node",
        name = "abra_fsm_planner_node",
        output="screen"
    )

    abra_spawner = Node(
            package="abra_gazebo",
            executable="spawn_traffic_lights.py",
            name="spawn_traffic_lights",
            output="screen"
        )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_abra_gazebo, 'worlds', 'parkour.world'), ''],
          description='SDF world file'),
        gazebo,
        abra,
        lidar_localization_ros2,
        start_rviz,
        abra_localization,
        stanley_controller,
        abra_fsm_planner,
        abra_spawner
    ])
