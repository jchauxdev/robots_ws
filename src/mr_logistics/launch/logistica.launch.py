"""Gestor de misiones + App web.  ros2 launch mr_logistics logistica.launch.py  (abrir http://<ip>:8080)"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    lc = LaunchConfiguration
    cfg = PathJoinSubstitution([FindPackageShare('mr_logistics'), 'config', 'logistica.yaml'])
    stations = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'config', 'estaciones.yaml'])
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('stations', default_value=stations),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='', description='espacio de nombres del robot (mr)'),
        Node(package='mr_logistics', executable='mission_manager', name='mission_manager', output='screen',
             parameters=[cfg, {'stations_file': lc('stations')}]),
        # la App se ubica en el espacio del robot: cmd_vel, map, amcl_pose y diagnostics son relativos
        GroupAction([PushRosNamespace(lc('namespace')), Node(
            package='mr_logistics', executable='web_app', name='logistics_web', output='screen',
            condition=IfCondition(lc('web')), remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[cfg, {'stations_file': lc('stations')}])]),
    ])
