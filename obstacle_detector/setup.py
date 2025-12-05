from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ludmán Kevin',
    maintainer_email='ludmankevin@gmail.com',
    description='Turtlebot3 építése ros2 környezetben, valamint akadály kikerülő vezérlés létrehozása lidar segítségével.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_detector.obstacle_detector:main'
        ],
    },
)