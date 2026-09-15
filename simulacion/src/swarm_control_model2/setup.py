from setuptools import find_packages, setup

package_name = 'swarm_control_model2'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'differential_flocking = swarm_control_model2.differential_flocking:main',
            'swarm_data_logger = swarm_control_model2.swarm_data_logger:main'
        ],
    },
)
