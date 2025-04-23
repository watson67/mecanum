from setuptools import find_packages, setup

package_name = 'leader_follower'

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
            'leader_follower_v1 = leader_follower.leader_follower_v1:main',
            'leader_follower_v2 = leader_follower.leader_follower_v2:main',
            'leader_follower_v3 = leader_follower.leader_follower_v3:main',
        ],
    },
)
