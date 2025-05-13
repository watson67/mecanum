# filepath: /home/eswarm/turtlesim_ws/src/turtle_swarm/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'turtle_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eswarm',
    maintainer_email='eswarm@example.com',
    description='Swarm simulation with turtlesim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtles = turtle_swarm.spawn_turtles:main',
            'spawn_3turtles = turtle_swarm.spawn_3turtles:main',
            'pose_conversion = turtle_swarm.pose_conversion:main',
            'pose_conversion2 = turtle_swarm.pose_conversion2:main',
            'turtle_resetter = turtle_swarm.reset_swarm:main',
            'consensus = turtle_swarm.consensus:main',
        ],
    },
)