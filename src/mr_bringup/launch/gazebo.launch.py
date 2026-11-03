"""Simulacion en Gazebo Fortress (requiere GPU para lidar/camara): mundo laboratorio + robot + puente ros_gz.

  ros2 launch mr_bringup gazebo.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    desc = PathJoinSubstitution([FindPackageShare('mr_description'), 'launch', 'description.launch.py'])
    world = PathJoinSubstitution([FindPackageShare('mr_sim'), 'worlds', 'laboratorio.sdf'])
    gz = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('x0', default_value='1.0'),
        DeclareLaunchArgument('y0', default_value='1.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz),
                                 launch_arguments={'gz_args': ['-r ', lc('world')]}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(desc), launch_arguments={
            'sim': 'true', 'use_sim_time': 'true', 'use_realsense': 'false'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'mr_robot', '-topic', 'robot_description',
                        '-x', lc('x0'), '-y', lc('y0'), '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'use_sim_time': True}],
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image']),
    ])
