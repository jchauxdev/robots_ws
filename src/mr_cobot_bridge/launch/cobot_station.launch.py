"""En el equipo del cobot, con la celda del cobot (paquetes jc_* en robots_ws) en marcha:  ros2 launch mr_cobot_bridge cobot_station.launch.py"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('mr_cobot_bridge'), 'config', 'cobot_station.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='mr_cobot_bridge', executable='cobot_station', name='cobot_station', output='screen',
             parameters=[cfg, {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
