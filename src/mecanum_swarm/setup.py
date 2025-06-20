from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanum_swarm'

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
            'tf2_manager = mecanum_swarm.tf2_manager:main',
            'tf2_obstacle_manager = mecanum_swarm.tf2_obstacle_manager:main',
            'swarm = mecanum_swarm.swarm:main',
            'tf2_visu = mecanum_swarm.tf2_visu:main',
            'swarm_master = mecanum_swarm.swarm_master:main',
            'circle = mecanum_swarm.circle:main',
            'rectangle = mecanum_swarm.rectangle:main',
            'distributed_swarm = mecanum_swarm.distributed_swarm:main',
            'distributed_obstacle_swarm = mecanum_swarm.distributed_obstacle_swarm:main',
            'distributed_event_swarm = mecanum_swarm.distributed_event_swarm:main',
            'distributed_manager = mecanum_swarm.distributed_manager:main',
            'distributed_tf2_manager = mecanum_swarm.distributed_tf2_manager:main',
            'goal_point_sender = mecanum_swarm.goal_point_sender:main',
            'swarm_obstacle = mecanum_swarm.swarm_obstacle:main',
            'cmd_vel_rate_logger = mecanum_swarm.cmd_vel_rate_logger:main',
            'barycenter_logger = mecanum_swarm.barycenter_logger:main',
            'distances_logger = mecanum_swarm.distances_logger:main',
            'goal_point_logger = mecanum_swarm.goal_point_logger:main',
            'eight = mecanum_swarm.eight:main',
            'formation_init_del= mecanum_swarm.formation_initialize_delaunay:main',
            'formation_init_spat= mecanum_swarm.formation_initialize_spatial:main',

        ],
    },
)
