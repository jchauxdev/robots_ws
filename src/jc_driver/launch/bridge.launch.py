"""Puente al robot real (o simulado con simulate:=true).

    ros2 launch jc_driver bridge.launch.py port:=/dev/ttyUSB0 simulate:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("jc_driver"), "config", "bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="1000000"),
        DeclareLaunchArgument("simulate", default_value="false"),
        Node(package="jc_driver", executable="jetcobot_bridge", name="jetcobot_bridge",
             output="screen",
             parameters=[cfg, {"port": LaunchConfiguration("port"),
                               "baud": LaunchConfiguration("baud"),
                               "simulate": LaunchConfiguration("simulate")}]),
    ])
