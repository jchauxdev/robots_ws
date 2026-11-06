"""Convierte la profundidad de la D435i en un LaserScan virtual (/d435i/scan) para los costmaps de Nav2.

  ros2 launch mr_bringup robot.launch.py realsense:=true
  ros2 launch mr_bringup realsense_scan.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('range_min', default_value='0.3'),    # zona ideal de la D435i: 0,3-3 m
        DeclareLaunchArgument('range_max', default_value='3.0'),
        DeclareLaunchArgument('scan_height', default_value='10'),   # filas de la imagen usadas
        Node(package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
             name='d435i_to_scan', namespace=lc('namespace'), output='screen',
             remappings=[('depth', 'd435i/depth/image_rect_raw'),
                         ('depth_camera_info', 'd435i/depth/camera_info'),
                         ('scan', 'd435i/scan')],
             parameters=[{'range_min': lc('range_min'), 'range_max': lc('range_max'),
                          'scan_height': lc('scan_height'), 'output_frame': 'd435i_depth_frame'}]),
    ])
