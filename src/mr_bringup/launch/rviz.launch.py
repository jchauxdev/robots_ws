"""RViz2 en el PC del instructor (el robot publica; el PC solo visualiza).

  ros2 launch mr_bringup rviz.launch.py                     # config:=navegacion | robot
  ros2 launch mr_bringup rviz.launch.py namespace:=mr       # robot en la celda integrada (DA-21.1)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

TOPICS = ('scan', 'map', 'robot_description', 'plan', 'local_plan', 'goal_pose', 'initialpose', 'clicked_point',
          'particle_cloud', 'local_costmap/costmap', 'global_costmap/costmap', 'odom')


def _setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    cfg = LaunchConfiguration('config').perform(context)
    share = FindPackageShare('mr_bringup').perform(context)
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')] + [(f'/{t}', t) for t in TOPICS] if ns else []
    return [GroupAction([PushRosNamespace(ns), Node(
        package='rviz2', executable='rviz2', output='screen', remappings=remaps,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', f'{share}/rviz/{cfg}.rviz'])])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='navegacion'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
