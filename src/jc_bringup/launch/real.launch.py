"""Celda con el JetCobot REAL: puente pymycobot + MoveIt 2 + servidor de movimiento.

    ros2 launch jc_bringup real.launch.py port:=/dev/ttyUSB0
    ros2 launch jc_bringup real.launch.py simulate:=true     # ensayo sin robot
Opcional: camera:=true lanza la camara USB, handeye:=true publica la TF calibrada,
compress:=true republica imagenes comprimidas para verlas desde el PC y rviz:=false
deja la visualizacion al PC (ros2 launch jc_moveit_config rviz.launch.py).
"""
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("jc_moveit_config"), "launch"))
from common import build_moveit_config  # noqa: E402
from launch.actions import SetEnvironmentVariable  # noqa: E402


def _setup(context):
    cfg = build_moveit_config("real")
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bridge_cfg = os.path.join(get_package_share_directory("jc_driver"), "config", "bridge.yaml")
    rviz_cfg = os.path.join(get_package_share_directory("jc_moveit_config"), "config", "moveit.rviz")
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[cfg.robot_description], output="log"),
        Node(package="jc_driver", executable="jetcobot_bridge", name="jetcobot_bridge", output="screen",
             parameters=[bridge_cfg, {"port": lc("port"), "baud": int(lc("baud")),
                                      "simulate": lc("simulate").lower() == "true"}]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[cfg.to_dict()]),
        Node(package="jc_motion", executable="motion_server", output="screen",
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.robot_description_kinematics, cfg.joint_limits,
                         {"default_velocity_scaling": 0.2}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.robot_description_kinematics, cfg.planning_pipelines],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    if lc("camera").lower() == "true":
        cam_info = os.path.join(get_package_share_directory("jc_vision"), "config", "camera_info.yaml")
        nodes.append(Node(package="jc_vision", executable="camera_node", output="screen",
                          parameters=[{"device": lc("video_device"), "camera_info_file": cam_info,
                                       "frame_id": "camera_optical_frame"}]))
    if lc("handeye").lower() == "true":
        he = os.path.join(get_package_share_directory("jc_vision"), "config", "handeye_result.yaml")
        with open(he, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        t, q = d["translation"], d["rotation_xyzw"]
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--x", str(t[0]), "--y", str(t[1]), "--z", str(t[2]),
                       "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]), "--qw", str(q[3]),
                       "--frame-id", d["parent_frame"], "--child-frame-id", d["child_frame"]]))
    if lc("compress").lower() == "true":
        # Imagenes comprimidas (JPEG) para verlas desde el PC por la red (image_transport_plugins)
        for src in ("/jc/debug_image", "/camera/image_raw"):
            nodes.append(Node(package="image_transport", executable="republish", output="log",
                              name="republish_" + src.strip("/").replace("/", "_"),
                              arguments=["raw", "compressed"],
                              remappings=[("in", src), ("out/compressed", src + "/compressed")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="1000000"),
        DeclareLaunchArgument("simulate", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("handeye", default_value="false"),
        DeclareLaunchArgument("compress", default_value="false",
                              description="true: publica /jc/debug_image/compressed y /camera/image_raw/compressed"),
        OpaqueFunction(function=_setup),
    ])
