"""RViz2 con el plugin MotionPlanning SIN lanzar move_group (ejecucion distribuida).

Uso tipico: la Jetson corre la celda (real.launch.py rviz:=false) y el PC del instructor,
en la misma red y con el mismo ROS_DOMAIN_ID, solo abre la visualizacion:

    ros2 launch jc_moveit_config rviz.launch.py                 # robot real
    ros2 launch jc_moveit_config rviz.launch.py use_sim_time:=true   # celda simulada en otra maquina
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from common import build_moveit_config  # noqa: E402


def _setup(context):
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    cfg = build_moveit_config(sim_mode)
    rviz_cfg = os.path.join(get_package_share_directory("jc_moveit_config"), "config", "moveit.rviz")
    return [Node(
        package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_cfg],
        parameters=[cfg.robot_description, cfg.robot_description_semantic,
                    cfg.robot_description_kinematics, cfg.planning_pipelines,
                    {"use_sim_time": use_sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode", default_value="real",
                              description="Variante del URDF (real/mock/gazebo); solo afecta la visualizacion"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
