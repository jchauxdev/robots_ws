from setuptools import setup

package_name = 'mr_navigation'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='Estaciones con nombre sobre el mapa y ordenes "ir a X" con Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'station_manager = mr_navigation.station_manager:main',
        'ir_a = mr_navigation.cli:ir_a',
        'recorrido = mr_navigation.cli:recorrido',
        'guardar_punto = mr_navigation.cli:guardar_punto',
        'estaciones = mr_navigation.cli:estaciones',
    ]},
)
