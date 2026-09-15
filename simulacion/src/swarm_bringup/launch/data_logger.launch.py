from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
# Description: This launch file belongs to the final simulation process.
# It launches the Data Logger node for the swarm simulation.
# This is the LAST launch file you will need to run if you want to get full information about the swarm's behavior.

def generate_launch_description():
    return LaunchDescription([
        # Data logger
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='swarm_control',
                    executable='swarm_data_logger',
                    name='swarm_data_logger',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True}
                    ]
                )
            ]
        )
    ])