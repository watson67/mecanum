from setuptools import find_packages, setup

package_name = 'teleop_mecanum'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'teleop_mecanum = teleop_mecanum.teleop_mecanum:main',
            'teleop_dualshock = teleop_mecanum.dualshock_mecanum:main',
            'teleop_mecanum_all = teleop_mecanum.teleop_mecanum_all:main',
            'follower_mecanum = teleop_mecanum.follower_mecanum:main',
        ],
    },
)
