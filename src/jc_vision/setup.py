from glob import glob
from setuptools import setup

package_name = "jc_vision"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Julian Rene Chaux Cedeno",
    maintainer_email="julianrenechaux@gmail.com",
    description="Vision artificial para la celda JetCobot: camara, color/forma, ArUco, YOLO y calibracion mano-ojo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "camera_node = jc_vision.camera_node:main",
        "color_shape_detector = jc_vision.color_shape_detector:main",
        "aruco_detector = jc_vision.aruco_detector:main",
        "yolo_detector = jc_vision.yolo_detector:main",
        "handeye_calibration = jc_vision.handeye_calibration:main",
    ]},
)
