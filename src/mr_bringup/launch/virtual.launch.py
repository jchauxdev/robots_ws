"""Robot VIRTUAL sin hardware ni GPU: emulador del RoboClaw (puerto serie virtual) + lidar 2D simulado.

mr_base es EXACTAMENTE el mismo nodo del robot real; solo cambia el puerto (/tmp/roboclaw_emu).
  ros2 launch mr_bringup virtual.launch.py
  ros2 launch mr_bringup virtual.launch.py true_wheel_diameter:=0.090 true_wheel_separation:=0.2016  # sin error
  ros2 launch mr_bringup virtual.launch.py namespace:=mr     # celda integrada con el cobot (DA-21.1)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    lc = LaunchConfiguration
    desc = PathJoinSubstitution([FindPackageShare('mr_description'), 'launch', 'description.launch.py'])
    base_cfg = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'config', 'mr_base.yaml'])
    world = PathJoinSubstitution([FindPackageShare('mr_sim'), 'worlds', 'laboratorio.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='p. ej. mr en la celda con el cobot'),
        DeclareLaunchArgument('base_config', default_value=base_cfg),
        DeclareLaunchArgument('world_map', default_value=world),
        DeclareLaunchArgument('x0', default_value='1.0'),
        DeclareLaunchArgument('y0', default_value='1.0'),
        DeclareLaunchArgument('yaw0', default_value='0.0'),
        DeclareLaunchArgument('true_wheel_diameter', default_value='0.0875'),
        DeclareLaunchArgument('true_wheel_separation', default_value='0.2090'),
        DeclareLaunchArgument('slip_std', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(desc), launch_arguments={
            'use_realsense': 'false', 'namespace': lc('namespace')}.items()),
        GroupAction([
            PushRosNamespace(lc('namespace')),
            Node(package='mr_sim', executable='roboclaw_emulator', name='roboclaw_emulator', output='screen',
                 parameters=[{'link': '/tmp/roboclaw_emu', 'x0': lc('x0'), 'y0': lc('y0'), 'yaw0': lc('yaw0'),
                              'true_wheel_diameter': lc('true_wheel_diameter'),
                              'true_wheel_separation': lc('true_wheel_separation'), 'slip_std': lc('slip_std')}]),
            Node(package='mr_sim', executable='lidar_sim', name='lidar_sim', output='screen',
                 parameters=[{'world_map': lc('world_map')}]),
        ]),
        # TimerAction se ejecuta fuera del alcance del GroupAction: el espacio de nombres se repite adentro
        TimerAction(period=1.5, actions=[GroupAction([
            PushRosNamespace(lc('namespace')),
            Node(package='mr_base', executable='base_node', name='mr_base', output='screen', remappings=TF,
                 parameters=[lc('base_config'), {'port': '/tmp/roboclaw_emu'}])])]),
    ])
