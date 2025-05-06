from setuptools import find_packages, setup
from glob import glob

package_name = 'consensus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'master = consensus.master:main',
            'consensus_v1 = consensus.consensus_v1:main',
            'consensus_v2 = consensus.consensus_v2:main',
            'consensus_v3 = consensus.consensus_v2:main',
        ],
    },
)
