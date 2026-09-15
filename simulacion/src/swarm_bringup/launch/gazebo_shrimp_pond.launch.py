from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'shrimp_pond_static.sdf')

    # Include common resource roots for meshes/models if later needed
    resource_paths = [
        worlds_pkg,
        ':',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            resource_paths,
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
    ])
