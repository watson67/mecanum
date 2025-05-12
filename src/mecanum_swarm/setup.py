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
            'swarm = mecanum_swarm.swarm:main',
            'tf2_visu = mecanum_swarm.tf2_visu:main',
            'swarm_master = mecanum_swarm.swarm_master:main',
        ],
    },
)
