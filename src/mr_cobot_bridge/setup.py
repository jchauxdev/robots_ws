from glob import glob
from setuptools import setup

package_name = 'mr_cobot_bridge'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='Estacion del JetCobot para la logistica: pedidos -> pick & place sobre la bandeja del robot movil.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['cobot_station = mr_cobot_bridge.cobot_station:main']},
)
