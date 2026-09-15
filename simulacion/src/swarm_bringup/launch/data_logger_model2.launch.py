from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Data logger for model2
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='swarm_control_model2',
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
