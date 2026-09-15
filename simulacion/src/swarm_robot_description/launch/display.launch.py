from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Display of the Robot on RViz2 from the urdf (which contains the original CAD design model)

def generate_launch_description():
    pkg = get_package_share_directory('swarm_robot_description')
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf')
    rviz_config_path = os.path.join(pkg, 'config', 'robot_display.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
        ),
        # Joint State Publisher GUI (to move joints interactively)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='jsp_gui'
        ),
        Node(package='rviz2', executable='rviz2')
    ])

