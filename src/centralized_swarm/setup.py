from setuptools import find_packages, setup

package_name = 'centralized_swarm'

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
            'swarm = centralized_swarm.swarm:main',
            'swarm2 = centralized_swarm.swarm2:main',
            'predictive_swarm = centralized_swarm.predictive_swarm:main',
            'predictive_event_swarm = centralized_swarm.predictive_event_swarm:main',
            'event_swarm = centralized_swarm.event_swarm:main',
            'old_swarm = centralized_swarm.old_swarm:main',
        ],
    },
)
