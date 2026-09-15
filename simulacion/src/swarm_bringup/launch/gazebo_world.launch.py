from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Description: This launch file belongs to the final simulation process.
# It launches Gazebo with the specified world (flat_ocean.sdf) and sets up the necessary nodes for the simulation.
# To use it, simply launch the file with the desired configuration.
# This is the FIRST launch file you will need to run to execute the simulation.

def generate_launch_description():
    # Paths
    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'flat_ocean.sdf')
    
    desc_pkg = get_package_share_directory('swarm_robot_description')

    resource_paths = [
        desc_pkg, ':',
        os.path.dirname(desc_pkg), ':',
        desc_pkg + '/models', ':',
    ]

    # Try optional robot description packages
    try:
        desc_pkg_model2 = get_package_share_directory('swarm_robot_description_model2')
        resource_paths += [
            desc_pkg_model2, ':',
            os.path.dirname(desc_pkg_model2), ':',
            desc_pkg_model2 + '/models', ':',
        ]
    except Exception:
        pass

    try:
        desc_pkg_model3 = get_package_share_directory('swarm_robot_description_model3')
        resource_paths += [
            desc_pkg_model3, ':',
            os.path.dirname(desc_pkg_model3), ':',
            desc_pkg_model3 + '/models', ':',
        ]
    except Exception:
        pass

    resource_paths += [os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    
    return LaunchDescription([
        # Set environment variables for Gazebo (This gave some problems due to Gazebo's Harmonic 
        # new form to reference directory models)
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            resource_paths
        ),

        # Launch Gazebo with the world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path], 
            output='screen'
        ),

        # Clock bridge
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ])