import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'slam_optimization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        # URDF
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.xacro'))),

        # World
        (os.path.join('share', package_name, 'world'),
         glob(os.path.join('world', '*.world'))),

        # Config
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),

        # Maps
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.pgm'))),

        # RViz
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
    ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilknur',
    maintainer_email='ilknurkoparir262@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_drive = slam_optimization_pkg.autonomous_node:main',
        ],
    },
)
