from setuptools import find_packages, setup

package_name = 'rmf_demos_bridges'

setup(
    name=package_name,
    version='2.0.4',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boon Han',
    maintainer_email='cnboonhan@openrobotics.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_socketio_bridge=rmf_demos_bridges.fleet_socketio_bridge:main',
            'fleet_robotmanager_mqtt_bridge=rmf_demos_bridges.fleet_robotmanager_mqtt_bridge:main',
            'mqtt_bridge=rmf_demos_bridges.mqtt_bridge:main',
            'bridge_layer=rmf_demos_bridges.bridge_layer.main:main',
            'bridge_publish_mock_workorder=rmf_demos_bridges.bridge_layer.tools.publish_mock_workorder:publish_once',
        ],
    },
)
