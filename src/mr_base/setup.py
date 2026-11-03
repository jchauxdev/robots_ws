from glob import glob
from setuptools import setup

package_name = 'mr_base'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/udev', glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='Driver ROS 2 del RoboClaw (odometria, diagnostico) y herramientas de calibracion.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'base_node = mr_base.base_node:main',
        'mover = mr_base.motion_tool:main',
        'check_roboclaw = mr_base.check_roboclaw:main',
    ]},
)
