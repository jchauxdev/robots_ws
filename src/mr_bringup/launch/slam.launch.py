"""Mapeo con slam_toolbox (online asincrono). Equivale a 'rosrun gmapping slam_gmapping' de ROS 1.

  ros2 launch mr_bringup slam.launch.py                      # robot real o virtual
  ros2 launch mr_bringup slam.launch.py use_sim_time:=true   # Gazebo
Guardar:  ros2 run nav2_map_server map_saver_cli -f ~/mapas/laboratorio
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    lc = LaunchConfiguration
    cfg = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'config', 'slam_toolbox.yaml'])
    rviz = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'rviz', 'navegacion.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('slam_params', default_value=cfg),
        GroupAction([
            PushRosNamespace(lc('namespace')),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
                 remappings=TF, parameters=[lc('slam_params'), {'use_sim_time': lc('use_sim_time')}]),
            Node(package='rviz2', executable='rviz2', arguments=['-d', rviz], condition=IfCondition(lc('rviz')),
                 remappings=TF, parameters=[{'use_sim_time': lc('use_sim_time')}]),
        ]),
    ])
