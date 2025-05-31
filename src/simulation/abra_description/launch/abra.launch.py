import os
import xacro
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_abra_description = get_package_share_directory('abra_description')
    xacro_file = os.path.join(pkg_abra_description, 'urdf', 'abra.xacro')

    robot_description_xacro = xacro.process_file(xacro_file)
    robot_description = robot_description_xacro.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='abra_description',
            executable='spawn_robot.py',
            arguments=[robot_description],
            output='screen'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description,
                 "use_sim_time": use_sim_time}],
            output="screen"),
    ])
