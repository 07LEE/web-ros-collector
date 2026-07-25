from setuptools import setup
import os
from glob import glob

package_name = 'camera_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    description='Web-based smartphone camera bridge for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_bridge_node = camera_bridge.camera_bridge_node:main'
        ],
    },
)
