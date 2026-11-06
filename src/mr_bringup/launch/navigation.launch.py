"""Navegacion autonoma: Nav2 (AMCL + planificador + controlador) + gestor de estaciones con nombre.

  ros2 launch mr_bringup navigation.launch.py map:=$HOME/mapas/laboratorio.yaml
  ros2 launch mr_bringup navigation.launch.py namespace:=mr     # celda integrada con el cobot (DA-21.1)
Luego:  ros2 run mr_navigation ir_a estacion_A
"""
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
# RViz: los topicos de navegacion.rviz son absolutos; se reubican en el espacio de nombres del robot
RVIZ_REMAP = TF + [(f'/{t}', t) for t in ('scan', 'map', 'robot_description', 'plan', 'local_plan', 'goal_pose',
                                          'initialpose', 'clicked_point', 'particle_cloud', 'local_costmap/costmap',
                                          'global_costmap/costmap', 'mr/stations_markers')]


def _setup(context):
    lc = LaunchConfiguration
    ns = lc('namespace').perform(context).strip('/')
    params = lc('params_file').perform(context)
    if ns:   # los costmaps usan topicos absolutos (/scan): se reescriben para el espacio de nombres
        txt = open(params).read().replace('topic: /scan', f'topic: /{ns}/scan')
        tmp = tempfile.NamedTemporaryFile('w', suffix='_nav2_params.yaml', delete=False)
        tmp.write(txt)
        tmp.close()
        params = tmp.name
    share = FindPackageShare('mr_bringup').perform(context)
    nav2 = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2), launch_arguments={
            'namespace': ns, 'use_namespace': 'true' if ns else 'false',
            'map': lc('map'), 'params_file': params, 'use_sim_time': lc('use_sim_time'),
            'autostart': 'true', 'use_composition': 'False'}.items()),
        GroupAction([PushRosNamespace(ns), Node(
            package='mr_navigation', executable='station_manager', name='station_manager', output='screen',
            remappings=TF, parameters=[{'stations_file': lc('stations'), 'use_sim_time': lc('use_sim_time')}])]),
    ]
    if lc('rviz').perform(context).lower() == 'true':
        actions.append(GroupAction([PushRosNamespace(ns), Node(
            package='rviz2', executable='rviz2', remappings=RVIZ_REMAP,
            arguments=['-d', share + '/rviz/navegacion.rviz'],
            parameters=[{'use_sim_time': lc('use_sim_time')}])]))
    return actions


def generate_launch_description():
    share = FindPackageShare('mr_bringup')
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([share, 'maps', 'laboratorio.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('stations', default_value=PathJoinSubstitution([share, 'config', 'estaciones.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
