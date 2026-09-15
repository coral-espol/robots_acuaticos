from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
# Description: This launch file belongs to the final simulation process.
# It launches the Flocking controllers (5 to 7 robots its recommended for testing purposes), 
# and sets up the necessary nodes (controller only) for the simulation.
# To use it, simply launch the file with the desired configuration.
# This is the THIRD launch file you will need to run to execute the simulation.

def generate_launch_description():
    # Arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='5',
        description='Number of robots in swarm'
    )
    
    actions = [num_robots_arg]

    # Flocking controllers for each robot
    for i in range(5): # range: Number of robots (needs to be changed manually)
        robot_name = f'swarm_bot_{i+1}'
        
        actions.append(
            TimerAction(
                period=2.0 + i * 0.2,  # Staggered delay to activate node with each robot
                actions=[
                    Node( # Here is the node used for each robot's flocking behavior (swarm_flocking controller)
                        # You can choose between flocking controller with (laser based controller) or without collision (odometry only)
                        package='swarm_control',
                        executable='swarm_flocking', # Select between 'swarm_flocking' or 'swarm_flocking_without_collision'
                        name=f'{robot_name}_flocking',
                        output='screen',
                        parameters=[
                            {'robot_id': i},
                            {'num_robots': 5}, # Number of robots in swarm (needs to be changed manually)
                            {'use_sim_time': True}
                        ]
                    )
                ]
            )
        )
    
    return LaunchDescription(actions)