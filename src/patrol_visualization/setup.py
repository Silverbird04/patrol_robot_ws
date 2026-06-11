from setuptools import find_packages, setup

package_name = 'patrol_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user04',
    maintainer_email='se010s@ewha.ac.kr',
    description='RViz marker visualization package for patrol robot events',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hazard_marker_publisher = patrol_visualization.hazard_marker_publisher:main',
        ],
    },
)
