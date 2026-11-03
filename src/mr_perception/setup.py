from setuptools import setup

package_name = 'mr_perception'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='Camara USB ligera con OpenCV para la Jetson Nano (ROS 2 Humble en contenedor).',
    license='Apache-2.0',
    entry_points={'console_scripts': ['camera_node = mr_perception.camera_node:main']},
)
