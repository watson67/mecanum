from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'swarm_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eswarm',
    maintainer_email='augustin.bonnel@insa-strasbourg.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_manager = swarm_manager.tf2_manager:main',
            'tf2_obstacle_manager = swarm_manager.tf2_obstacle_manager:main',
            'tf2_visu = swarm_manager.tf2_visu:main',
            'swarm_master = swarm_manager.swarm_master:main',
            'distributed_tf2_manager = swarm_manager.distributed_tf2_manager:main',
            'distributed_manager = swarm_manager.distributed_manager:main',
            'goal_point_sender = swarm_manager.goal_point_sender:main',
        ],
    },
)
