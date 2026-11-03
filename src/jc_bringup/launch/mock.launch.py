"""Celda en modo MOCK: ros2_control con hardware simulado (sin fisica) + MoveIt 2.

Util para aprender MoveIt sin GPU ni robot.  ros2 launch jc_bringup mock.launch.py
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("jc_moveit_config"), "launch"))
from common import build_moveit_config  # noqa: E402


def _setup(context):
    cfg = build_moveit_config("mock")
    ctrl = os.path.join(get_package_share_directory("jc_description"), "config", "ros2_controllers.yaml")
    rviz_cfg = os.path.join(get_package_share_directory("jc_moveit_config"), "config", "moveit.rviz")
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[cfg.robot_description], output="log"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[cfg.robot_description, ctrl], output="log"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller",
                        "--controller-manager", "/controller_manager"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[cfg.to_dict()]),
        Node(package="jc_motion", executable="motion_server", output="screen",
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.robot_description_kinematics, cfg.joint_limits]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.robot_description_kinematics, cfg.planning_pipelines, cfg.joint_limits],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
