"""Aplicacion completa de la celda (Día 3 en simulación, DA-12; Día 4 con el robot real, DA-17): vision + pick & place.

Requiere la celda en marcha:  sim.launch.py (mode:=sim)  o  real.launch.py camera:=true (mode:=real)
    ros2 launch jc_apps celda.launch.py mode:=sim detector:=color auto_sort:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def _setup(context):
    mode = LC("mode").perform(context)
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("jc_vision"),
                                                   "launch", "vision.launch.py")),
        launch_arguments={"mode": mode, "detector": LC("detector").perform(context),
                          "camera_frame": LC("camera_frame").perform(context),
                          "model": LC("model").perform(context)}.items())
    cfg = os.path.join(get_package_share_directory("jc_bringup"), "config", "celda.yaml")
    pp = Node(package="jc_apps", executable="pick_place_node", output="screen",
              parameters=[cfg, {"use_sim_time": mode == "sim",
                                "auto_sort": LC("auto_sort").perform(context).lower() == "true",
                                "dry_run": LC("dry_run").perform(context).lower() == "true"}])
    return [vision, pp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("detector", default_value="color"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
        DeclareLaunchArgument("model", default_value="yolo11n.pt"),
        DeclareLaunchArgument("auto_sort", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
