"""Construccion comun de la configuracion de MoveIt 2 para el JetCobot."""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def modelo_jetcobot(desc: str) -> str:
    """'yahboom' si estan instaladas las mallas de Yahboom (importar_modelo_yahboom.sh); si no, 'simple'."""
    if os.path.isfile(os.path.join(desc, "meshes", "yahboom", "base_link.STL")):
        return "yahboom"
    print("[jc_description] Sin mallas de Yahboom: se dibuja el modelo simple. Para ver el JetCobot real, "
          "ejecute ~/robots_ws/src/jc_description/scripts/importar_modelo_yahboom.sh --descargar (DA-03, Paso 7).")
    return "simple"


def build_moveit_config(sim_mode: str = "mock"):
    desc = get_package_share_directory("jc_description")
    xacro_file = os.path.join(desc, "urdf", "jetcobot.urdf.xacro")
    return (
        MoveItConfigsBuilder("jetcobot", package_name="jc_moveit_config")
        .robot_description(file_path=xacro_file,
                           mappings={"sim_mode": sim_mode, "modelo": modelo_jetcobot(desc)})
        .robot_description_semantic(file_path="config/jetcobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
