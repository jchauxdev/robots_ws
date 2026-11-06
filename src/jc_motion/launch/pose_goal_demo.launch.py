"""Ejecuta la demo C++ de MoveIt 2 con los parametros del robot (Día 2, Parte A: DA-05).

    ros2 launch jc_motion pose_goal_demo.launch.py use_sim_time:=true   # con Gazebo
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("jc_moveit_config"), "launch"))
from common import build_moveit_config  # noqa: E402
from launch.actions import SetEnvironmentVariable  # noqa: E402


def _setup(context):
    cfg = build_moveit_config("mock")
    st = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    return [Node(package="jc_motion", executable="pose_goal_demo", output="screen",
                 parameters=[cfg.robot_description, cfg.robot_description_semantic,
                             cfg.robot_description_kinematics, cfg.joint_limits,
                             {"use_sim_time": st}])]


def generate_launch_description():
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup)])
