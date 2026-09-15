from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
# Description: This launch file belongs to the final simulation process.
# It launches the spawner (5 to 7 robots its recommended for testing purposes), and sets up the necessary nodes for the simulation.
# To use it, simply launch the file with the desired configuration.
# This is the SECOND launch file you will need to run to execute the simulation.

def generate_launch_description():
    # Arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='5', # Number of robots in the swarm (needs to be changed manually)
        description='Number of robots to spawn'
    )

    # Paths
    desc_pkg = get_package_share_directory('swarm_robot_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'azimuth_bot.urdf.xacro')

    # Variables
    num_robots = LaunchConfiguration('num_robots')
    
    actions = [
        num_robots_arg,

        # Set environment variables for Gazebo (This gave some problems due to Gazebo's Harmonic 
        # new form to reference directory models)
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [desc_pkg, ':', desc_pkg + '/models', ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        )
    ]
    # Spawn robots (fixed to spawn 5 robots, can be made dynamic later in further development)
    for i in range(5): # range: number of robots (needs to be changed manually)
        robot_name = f'swarm_bot_{i+1}'

        # URDF for each robot
        urdf_xml = Command([
            'xacro', ' ', xacro_path,
            ' spawn_name:=', robot_name
        ])
        
        # Robot State Publisher
        actions.append(
            TimerAction(
                period=2.0 + i * 0.5,  # Staggered delay
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

        # Spawn robot in Gazebo
        actions.append(
            TimerAction(
                period=4.0 + i * 1.0,  # Longer delay for spawn
                actions=[
                    Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=[
                            '-name', robot_name,
                            '-string', urdf_xml,
                            '-x', str(i * 0.5), '-y', '0', '-z', '0.0',
                            '-allow_renaming', 'true'
                        ],
                        output='screen'
                    )
                ]
            )
        )
    
    return LaunchDescription(actions)