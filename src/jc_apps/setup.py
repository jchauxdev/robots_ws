from glob import glob
from setuptools import setup

package_name = "jc_apps"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Julian Rene Chaux Cedeno",
    maintainer_email="julianrenechaux@gmail.com",
    description="Nodos didacticos y de aplicacion de la celda JetCobot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "talker = jc_apps.talker:main",
        "listener = jc_apps.listener:main",
        "gripper_service_demo = jc_apps.gripper_service_demo:main",
        "countdown_server = jc_apps.countdown_server:main",
        "countdown_client = jc_apps.countdown_client:main",
        "param_demo = jc_apps.param_demo:main",
        "fk_check = jc_apps.fk_check:main",
        "trajectory_demo = jc_apps.trajectory_demo:main",
        "square_demo = jc_apps.square_demo:main",
        "pick_place_node = jc_apps.pick_place_node:main",
    ]},
)
