from setuptools import find_packages, setup

package_name = 'swarm_control_model3'

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
    maintainer_email='egrivas@espol.edu.ec',
    description='Control package for aquatic robot model 3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'differential_flocking = swarm_control_model3.differential_flocking:main',
            'swarm_data_logger = swarm_control_model3.swarm_data_logger:main',
            'force_torque_wander = swarm_control_model3.force_torque_wander:main',
            'force_torque_straight = swarm_control_model3.force_torque_straight:main',
            'cmd_vel_straight = swarm_control_model3.cmd_vel_straight:main',
            'cmd_vel_wander = swarm_control_model3.cmd_vel_wander:main',
            'cmd_vel_swarm = swarm_control_model3.cmd_vel_swarm:main',
            'waypoint_nav_swarm = swarm_control_model3.waypoint_nav_swarm:main'
        ],
    },
)
