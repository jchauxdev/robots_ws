"""Lanza move_group (MoveIt 2) y opcionalmente RViz con el plugin MotionPlanning.

    ros2 launch jc_moveit_config move_group.launch.py sim_mode:=mock|gazebo|real use_sim_time:=false
No lanza controladores: estos los proveen jc_bringup (mock/Gazebo) o jc_driver (real).
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from common import build_moveit_config  # noqa: E402
from launch.actions import SetEnvironmentVariable  # noqa: E402


def _setup(context):
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    cfg = build_moveit_config(sim_mode)
    rviz_cfg = os.path.join(get_package_share_directory("jc_moveit_config"), "config", "moveit.rviz")

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[cfg.to_dict(), {"use_sim_time": use_sim_time,
                                    "publish_robot_description_semantic": True}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_cfg],
        parameters=[cfg.robot_description, cfg.robot_description_semantic,
                    cfg.robot_description_kinematics, cfg.planning_pipelines,
                    cfg.joint_limits, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument("sim_mode", default_value="mock"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
