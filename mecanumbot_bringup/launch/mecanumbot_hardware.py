import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the robot description
    urdf_path = os.path.join(get_package_share_directory('mecanumbot_description'), 'urdf', 'mecanumbot.urdf')
    urdf_doc = xacro.parse(open(urdf_path, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = urdf_doc.toxml()

    robot_controller_config = os.path.join(get_package_share_directory('mecanumbot_description'), 'config', 'robot_controller_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/mecanumbot_state_publisher.py']
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/mecanumbot_teleop.py']
            )
        ),

        Node(
            package='mecanumbot_control',
            executable='mecanumbot_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                robot_controller_config
            ]
        )
    ])