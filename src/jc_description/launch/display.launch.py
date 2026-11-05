"""Visualiza el modelo URDF/Xacro del JetCobot en RViz2 con sliders articulares.

    ros2 launch jc_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("jc_description")
    xacro_file = os.path.join(pkg, "urdf", "jetcobot.urdf.xacro")
    rviz_file = os.path.join(pkg, "rviz", "view.rviz")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    # Mallas de Yahboom instaladas (scripts/importar_modelo_yahboom.sh) -> modelo real; si no, primitivas
    if os.path.isfile(os.path.join(pkg, "meshes", "yahboom", "base_link.STL")):
        modelo = "yahboom"
    else:
        modelo = "simple"
        print("[jc_description] Sin mallas de Yahboom: se dibuja el modelo simple. Para ver el JetCobot real, "
              "ejecute ~/robots_ws/src/jc_description/scripts/importar_modelo_yahboom.sh (DA-04).")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " sim_mode:=mock modelo:=", modelo]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Abrir joint_state_publisher_gui"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file],
             condition=IfCondition(rviz), output="screen"),
    ])
