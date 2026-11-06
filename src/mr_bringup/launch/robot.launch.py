"""Robot REAL: modelo + RoboClaw + RPLidar (+ camara RGB, + RealSense D435i opcionales).

Jetson Nano (dentro del contenedor Docker Humble) o Jetson AGX Orin (nativo):
  ros2 launch mr_bringup robot.launch.py
  ros2 launch mr_bringup robot.launch.py camera:=true realsense:=true
  ros2 launch mr_bringup robot.launch.py namespace:=mr        # celda integrada con el cobot (DA-21.1)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    lc = LaunchConfiguration
    desc = PathJoinSubstitution([FindPackageShare('mr_description'), 'launch', 'description.launch.py'])
    base_cfg = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'config', 'mr_base.yaml'])
    cam_cfg = PathJoinSubstitution([FindPackageShare('mr_bringup'), 'config', 'usb_cam.yaml'])
    rs_launch = PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('namespace', default_value='', description='p. ej. mr en la celda con el cobot'),
        DeclareLaunchArgument('base_config', default_value=base_cfg),
        DeclareLaunchArgument('roboclaw_port', default_value='/dev/roboclaw'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('lidar_baud', default_value='115200',
                              description='A1/A2M8: 115200; A2M12/A3/S1: 256000; C1: 460800'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159265', description='orientacion del lidar (DA-16.1)'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('camera_driver', default_value='opencv', description='opencv (Nano) | usb_cam'),
        DeclareLaunchArgument('video_device', default_value='0'),
        DeclareLaunchArgument('realsense', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(desc), launch_arguments={
            'use_realsense': lc('realsense'), 'lidar_yaw': lc('lidar_yaw'), 'namespace': lc('namespace')}.items()),
        GroupAction([
            PushRosNamespace(lc('namespace')),
            SetRemap(src='/tf', dst='tf'), SetRemap(src='/tf_static', dst='tf_static'),   # tambien la RealSense
            Node(package='mr_base', executable='base_node', name='mr_base', output='screen',
                 parameters=[lc('base_config'), {'port': lc('roboclaw_port')}]),
            Node(package='rplidar_ros', executable='rplidar_node', name='rplidar_node', output='screen',
                 condition=IfCondition(lc('lidar')),
                 parameters=[{'channel_type': 'serial', 'serial_port': lc('lidar_port'),
                              'serial_baudrate': lc('lidar_baud'), 'frame_id': 'laser',
                              'inverted': False, 'angle_compensate': True}]),
            Node(package='mr_perception', executable='camera_node', name='rgb_camera', output='screen',
                 condition=IfCondition(PythonExpression(["'", lc('camera'), "' == 'true' and '",
                                                         lc('camera_driver'), "' == 'opencv'"])),
                 parameters=[{'device': lc('video_device')}]),
            Node(package='usb_cam', executable='usb_cam_node_exe', name='rgb_camera', output='screen',
                 condition=IfCondition(PythonExpression(["'", lc('camera'), "' == 'true' and '",
                                                         lc('camera_driver'), "' == 'usb_cam'"])),
                 parameters=[cam_cfg],
                 remappings=[('image_raw', 'rgb_camera/image_raw'), ('camera_info', 'rgb_camera/camera_info'),
                             ('image_raw/compressed', 'rgb_camera/image_raw/compressed')]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rs_launch), condition=IfCondition(lc('realsense')),
                                     launch_arguments={
                                         'camera_namespace': '', 'camera_name': 'd435i',
                                         'enable_color': 'true', 'enable_depth': 'true',
                                         'rgb_camera.color_profile': '640,480,15',
                                         'depth_module.depth_profile': '640,480,15',
                                         'align_depth.enable': 'true', 'pointcloud.enable': 'false',
                                         'enable_gyro': 'true', 'enable_accel': 'true', 'unite_imu_method': '2',
                                         'publish_tf': 'true'}.items()),
        ]),
    ])
