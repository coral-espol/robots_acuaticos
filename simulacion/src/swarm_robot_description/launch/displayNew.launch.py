from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

# Display of the Robot on RViz2 from the urdf.xacro (which contains the new design model used in simulations)


def generate_launch_description():
    # Paths
    desc_pkg = get_package_share_directory('swarm_robot_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'azimuth_bot.urdf.xacro')
    rviz_config_path = os.path.join(desc_pkg, 'config', 'robot_display.rviz')

    # Generate URDF from XACRO
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': False}
            ]
        ),

        # Joint State Publisher GUI (to move joints interactively)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
        )
    ])