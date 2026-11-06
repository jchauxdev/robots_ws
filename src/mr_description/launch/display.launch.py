"""Visualiza el modelo del robot en RViz2 (equivale a 'roslaunch robot display.launch' de ROS 1)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg = FindPackageShare('mr_description')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'mr_robot.urdf.xacro']),
        ' use_realsense:=', LaunchConfiguration('use_realsense'),
        ' lidar_yaw:=', LaunchConfiguration('lidar_yaw')]), value_type=str)
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('gui', default_value='true', description='joint_state_publisher_gui (mover ruedas)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_realsense', default_value='true'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159265'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher', condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'robot.rviz'])]),
    ])
