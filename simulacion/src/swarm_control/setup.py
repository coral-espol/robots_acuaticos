from setuptools import find_packages, setup

package_name = 'swarm_control'

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
    maintainer='emmons6411',
    maintainer_email='emmanuel4590@gmail.com',
    description='swarm control simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'working_smooth_movement = swarm_control.working_smooth_movement:main',
            'swarm_flocking = swarm_control.swarm_flocking:main',
            'swarm_data_logger = swarm_control.swarm_data_logger:main',
        ],
    },
)