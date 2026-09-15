from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Description: This launch file was made during the testing process. 
# Not recommended for production use, it's just evidence of testing.
# It spawns a robot in Gazebo and sets up the necessary nodes and parameters to make a single robot movement testing.
# To use it, simply launch the file with the desired configuration.

def generate_launch_description():
    # --- Launch arguments ---
    spawn_name_arg = DeclareLaunchArgument(
        'spawn_name', default_value='swarm_bot_1',
        description='Name of the robot in Gazebo'
    )

    # --- Paths ---
    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'flat_ocean.sdf')
    
    desc_pkg = get_package_share_directory('swarm_robot_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'azimuth_bot.urdf.xacro')
    
    # --- Variables ---
    spawn_name = LaunchConfiguration('spawn_name')
    
    # URDF.xacro exportation
    urdf_xml = Command([
        'xacro', ' ', xacro_path,
        ' spawn_name:=', spawn_name
    ])
    
    return LaunchDescription([
        # Arguments
        spawn_name_arg,

        # Set environment variables for Gazebo
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [desc_pkg, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),

        # 1) Open Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path], 
            output='screen'
        ),

        # 2) Clock bridge
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
        
        # 3) Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=spawn_name,
            output='screen',
            parameters=[
                {'robot_description': ParameterValue(urdf_xml, value_type=str)},
                {'use_sim_time': True}
            ]
        ),
        
        # 4) Robot spawner
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim', 
                    executable='create',
                    arguments=[
                        '-name', spawn_name,
                        '-string', urdf_xml,
                        '-x', '0', '-y', '0', '-z', '0.0',
                        '-allow_renaming', 'true'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # 5) Comunication bridge
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_bridge', 
                    executable='parameter_bridge',
                    name='cmd_vel_bridge',
                    arguments=['/model/swarm_bot_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                    output='screen',
                    remappings=[('/model/swarm_bot_1/cmd_vel', '/swarm_bot_1/cmd_vel')]
                ),
            ]
        ),
        
        # 6) Smooth controller aplicated for aquatic navigation
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='swarm_control',
                    executable='working_smooth_movement',
                    name='working_smooth_movement',
                    output='screen'
                ),
            ]
        ),

    ])