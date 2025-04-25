from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'snc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='s4064801@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_explore = snc.trigger_explore:main',
            'hazard_detection = snc.node_hazard_detection:main',
            'node_start = snc.node_start:main',
            'explore_wall_follow = snc.explore_wall_follow:main',
            'track_home = snc.track_home:main',
            
        ],
    },
)
