"""Solo robot_state_publisher (lo incluyen los launch del robot real y de simulacion)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg = FindPackageShare('mr_description')
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg, 'urdf', 'mr_robot.urdf.xacro']),
        ' sim:=', LaunchConfiguration('sim'),
        ' use_realsense:=', LaunchConfiguration('use_realsense'),
        ' lidar_yaw:=', LaunchConfiguration('lidar_yaw')]), value_type=str)
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_realsense', default_value='true'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159265'),
        DeclareLaunchArgument('namespace', default_value=''),
        GroupAction([PushRosNamespace(LaunchConfiguration('namespace')), Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}])]),
    ])
