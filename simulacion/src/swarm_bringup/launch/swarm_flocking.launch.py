from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Description: This launch file was made during the testing process. 
# Not recommended for production use, it's just evidence of testing.
# It spawns multiple robots in Gazebo and sets up the necessary nodes and parameters for flocking behavior.
# Probably not working right now.

def generate_launch_description():
    # Configuration
    num_robots = 5 # Number of robots in the swarm (needs to be changed manually)

    # Paths
    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'flat_ocean.sdf')
    
    desc_pkg = get_package_share_directory('swarm_robot_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'azimuth_bot.urdf.xacro')
    
    actions = []
    
    # Variables de entorno
    actions.append(
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [desc_pkg, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        )
    )
    
    # 1) Gazebo
    actions.append(
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path], 
            output='screen'
        )
    )
    
    # 2) Clock bridge
    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_bridge', 
                    executable='parameter_bridge',
                    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                    output='screen'
                )
            ]
        )
    )
    # 3) Create robots and controllers
    for i in range(num_robots):
        robot_name = f'swarm_bot_{i+1}'

        # URDF for each robot
        urdf_xml = Command([
            'xacro', ' ', xacro_path,
            ' spawn_name:=', robot_name
        ])
        
        # Robot State Publisher
        actions.append(
            TimerAction(
                period=5.0 + i * 0.5,  # Staggered delay
                actions=[
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name=f'{robot_name}_state_publisher',
                        namespace=robot_name,
                        output='screen',
                        parameters=[
                            {'robot_description': ParameterValue(urdf_xml, value_type=str)},
                            {'use_sim_time': True}
                        ]
                    )
                ]
            )
        )
        
        # Spawn robot
        actions.append(
            TimerAction(
                period=8.0 + i * 1.0,  # Staggered delay
                actions=[
                    Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=[
                            '-name', robot_name,
                            '-string', urdf_xml,
                            '-x', '0', '-y', '0', '-z', '0.0',
                            '-allow_renaming', 'true'
                        ],
                        output='screen'
                    )
                ]
            )
        )

        # Flocking controller
        actions.append(
            TimerAction(
                period=15.0 + i * 0.2,  # Staggered delay
                actions=[
                    Node(
                        package='swarm_control',
                        executable='swarm_flocking',
                        name=f'{robot_name}_flocking',
                        output='screen',
                        parameters=[
                            {'robot_id': i},
                            {'num_robots': num_robots},
                            {'use_sim_time': True}
                        ]
                    )
                ]
            )
        )
    
    return LaunchDescription(actions)