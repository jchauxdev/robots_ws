from glob import glob
from setuptools import setup

package_name = "jc_driver"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/udev", glob("udev/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Julian Rene Chaux Cedeno",
    maintainer_email="julianrenechaux@gmail.com",
    description="Puente ROS 2 <-> JetCobot (pymycobot) con acciones FollowJointTrajectory.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "jetcobot_bridge = jc_driver.jetcobot_bridge:main",
        "joint_state_follower = jc_driver.joint_state_follower:main",
        "check_robot = jc_driver.check_robot:main",
    ]},
)
