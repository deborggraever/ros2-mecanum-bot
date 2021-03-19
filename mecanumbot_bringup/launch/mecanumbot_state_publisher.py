import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_path = os.path.join(get_package_share_directory('mecanumbot_description'), 'urdf', 'mecanumbot.urdf')
    urdf_doc = xacro.parse(open(urdf_path, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = urdf_doc.toxml()
    #with open(urdf_path, 'r') as urdf_file:
    #    robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}
            ]
        )
        #Node(
        #    package='mecanumbot_test',
        #    executable='talker',
        #    output='screen'
        #),
    ])