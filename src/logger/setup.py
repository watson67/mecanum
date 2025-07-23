from setuptools import find_packages, setup

package_name = 'logger'

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
    entry_points={
        'console_scripts': [
            'cmd_vel_rate_logger = logger.cmd_vel_rate_logger:main',
            'barycenter_logger = logger.barycenter_logger:main',
            'distances_logger = logger.distances_logger:main',
            'goal_point_logger = logger.goal_point_logger:main',
            'event_logger = logger.event:main',
            'predictive_logger = logger.predictive_logger:main',
            'cpu_ram_logger = logger.cpu_ram_logger:main',
        ],
    },
)
