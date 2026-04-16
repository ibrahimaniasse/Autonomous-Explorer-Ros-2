import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'autonomous_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Include models (since models folder has subdirectories, we need a small helper, but for now glob the roots if simple, or we install it properly)
        (os.path.join('share', package_name, 'models', 'explorer_bot'), glob('models/explorer_bot/*')),
        # Include rviz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaiah',
    maintainer_email='isaiah@example.com',
    description='Autonomous Explorer robot for SLAM and navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = autonomous_explorer.obstacle_detector:main',
            'frontier_explorer = autonomous_explorer.frontier_explorer_node:main',
            'waypoint_manager = autonomous_explorer.waypoint_manager_node:main',
        ],
    },
)
