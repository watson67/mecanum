from setuptools import find_packages, setup

package_name = 'distributed_swarm'

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
    maintainer='eswarm',
    maintainer_email='augustin.bonnel@insa-strasbourg.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distributed_swarm = distributed_swarm.distributed_swarm:main',
            'distributed_obstacle_swarm = distributed_swarm.distributed_obstacle_swarm:main',
            'distributed_event_swarm = distributed_swarm.distributed_event_swarm:main',
            'distributed_predictive_swarm = distributed_swarm.distributed_predictive_swarm:main',

        ],
    },
)
