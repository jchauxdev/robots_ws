"""Celda robotica colaborativa SIMULADA en Gazebo Fortress + ros2_control + MoveIt 2.

    ros2 launch jc_bringup sim.launch.py            # todo
    ros2 launch jc_bringup sim.launch.py moveit:=false rviz:=false
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("jc_moveit_config"), "launch"))
from common import build_moveit_config  # noqa: E402


def _setup(context):
    bringup = get_package_share_directory("jc_bringup")
    world = os.path.join(bringup, "worlds", LaunchConfiguration("world").perform(context))
    use_moveit = LaunchConfiguration("moveit").perform(context).lower() == "true"
    cfg = build_moveit_config("gazebo")
    sim_time = {"use_sim_time": True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": (f"-s -r --headless-rendering -v 3 {world}"
                                      if LaunchConfiguration("headless").perform(context).lower() == "true"
                                      else f"-r -v 3 {world}")}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[cfg.robot_description, sim_time], output="log")

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "jetcobot", "-topic", "robot_description", "-z", "0.0"])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
                  arguments=[
                      "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                      "/wrist_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                  ])

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    ctrls = Node(package="controller_manager", executable="spawner",
                 arguments=["arm_controller", "gripper_controller",
                            "--controller-manager", "/controller_manager"])

    actions = [
        gz, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ctrls])),
    ]

    if use_moveit:
        rviz_cfg = os.path.join(get_package_share_directory("jc_moveit_config"), "config", "moveit.rviz")
        actions += [
            Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                 parameters=[cfg.to_dict(), sim_time]),
            Node(package="jc_motion", executable="motion_server", output="screen",
                 parameters=[cfg.robot_description, cfg.robot_description_semantic,
                             cfg.robot_description_kinematics, cfg.joint_limits, sim_time]),
            Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
                 parameters=[cfg.robot_description, cfg.robot_description_semantic,
                             cfg.robot_description_kinematics, cfg.planning_pipelines,
                             sim_time],
                 condition=IfCondition(LaunchConfiguration("rviz"))),
        ]
    return actions


def generate_launch_description():
    bringup = get_package_share_directory("jc_bringup")
    models = os.path.join(bringup, "models")
    # carpeta share/ que contiene jc_description: Gazebo resuelve ahi las mallas package://jc_description/...
    models += ":" + os.path.dirname(get_package_share_directory("jc_description"))
    old = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    # Ruta del plugin ign_ros2_control (normalmente la agrega su env-hook; se fuerza por robustez)
    plug = os.path.join(get_package_prefix("ign_ros2_control"), "lib")
    old_plug = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    return LaunchDescription([
        # decimales con punto en los YAML de parametros aunque el sistema este en espanol (1,5 vs 1.5)
        SetEnvironmentVariable("LC_NUMERIC", "C"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", models + (":" + old if old else "")),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plug + (":" + old_plug if old_plug else "")),
        DeclareLaunchArgument("world", default_value="celda.sdf"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true: solo servidor de Gazebo, sin GUI (equipos sin pantalla)"),
        OpaqueFunction(function=_setup),
    ])
