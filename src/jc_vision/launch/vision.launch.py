"""Detectores de vision para la celda.

    ros2 launch jc_vision vision.launch.py mode:=sim detector:=color   # Gazebo
    ros2 launch jc_vision vision.launch.py mode:=real detector:=aruco  # camara USB
    detector: color | aruco | clasico (color + aruco) | yolo | all
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def _setup(context):
    from launch.substitutions import LaunchConfiguration as LC
    mode = LC("mode").perform(context)
    det = LC("detector").perform(context)
    share = get_package_share_directory("jc_vision")
    sim = mode == "sim"
    common = {
        "image_topic": "/wrist_camera/image_raw" if sim else "/camera/image_raw",
        "camera_info_topic": "/wrist_camera/camera_info" if sim else "/camera/camera_info",
        "base_frame": "world",
        "camera_frame": "camera_optical_frame" if sim else LC("camera_frame").perform(context),
        "use_sim_time": sim,
    }
    nodes = []
    if det in ("color", "clasico", "all"):
        nodes.append(Node(package="jc_vision", executable="color_shape_detector", output="screen",
                          parameters=[common, {"profiles_file": os.path.join(share, "config", "hsv_profiles.yaml"),
                                               "plane_z": float(LC("plane_z").perform(context))}]))
    if det in ("aruco", "clasico", "all"):
        nodes.append(Node(package="jc_vision", executable="aruco_detector", output="screen",
                          parameters=[common, {"marker_length": float(LC("marker_length").perform(context))}]))
    if det in ("yolo", "all"):
        nodes.append(Node(package="jc_vision", executable="yolo_detector", output="screen",
                          parameters=[common, {"model": LC("model").perform(context),
                                               "plane_z": float(LC("plane_z").perform(context))}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        DeclareLaunchArgument("mode", default_value="sim", description="sim | real"),
        DeclareLaunchArgument("detector", default_value="color"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame",
                              description="real con hand-eye: camera_calibrated_optical"),
        DeclareLaunchArgument("plane_z", default_value="0.025"),
        DeclareLaunchArgument("marker_length", default_value="0.030"),
        DeclareLaunchArgument("model", default_value="yolo11n.pt"),
        OpaqueFunction(function=_setup),
    ])
