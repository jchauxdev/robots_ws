"""Solo el nodo base (RoboClaw). Ej.: ros2 launch mr_base base.launch.py port:=/dev/ttyACM0"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('mr_base'), 'config', 'mr_base.yaml'])),
        DeclareLaunchArgument('port', default_value='/dev/roboclaw'),
        Node(package='mr_base', executable='base_node', name='mr_base', output='screen',
             parameters=[LaunchConfiguration('config'), {'port': LaunchConfiguration('port')}]),
    ])
