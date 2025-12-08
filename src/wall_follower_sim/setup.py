import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wall_follower_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include semua file launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include semua file URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Include semua file World
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jekiijekk',
    maintainer_email='jekiijekk@todo.todo',
    description='Simulasi Wall Follower Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = wall_follower_sim.wall_follower_node:main',
        ],
    },
)