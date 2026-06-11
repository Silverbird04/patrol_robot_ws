
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'patrol_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user04',
    maintainer_email='se0102s@ewha.ac.kr',
    description='Patrol manager package for waypoint check requests',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_patrol_manager = patrol_manager.simple_patrol_manager:main',
        ],
    },
)
