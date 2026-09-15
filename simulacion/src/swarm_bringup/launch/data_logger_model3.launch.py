from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Data logger para el modelo 3. Sirve tanto para pruebas con cmd_vel_swarm
# (log_nav_status:=false, por defecto) como para pruebas con
# waypoint_nav_swarm (log_nav_status:=true), en cuyo caso agrega columnas de
# navegación (nav_state, wp_index, dist_to_wp, evasions) leídas de
# /robotN/nav_status.
#
# Ejemplos:
#   ros2 launch swarm_bringup data_logger_model3.launch.py num_robots:=20
#   ros2 launch swarm_bringup data_logger_model3.launch.py num_robots:=20 log_nav_status:=true


def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument('num_robots', default_value='20')
    log_nav_status_arg = DeclareLaunchArgument('log_nav_status', default_value='false')

    return LaunchDescription([
        num_robots_arg,
        log_nav_status_arg,
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='swarm_control_model3',
                    executable='swarm_data_logger',
                    name='swarm_data_logger',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': True,
                            'num_robots': ParameterValue(LaunchConfiguration('num_robots'), value_type=int),
                            'log_nav_status': ParameterValue(LaunchConfiguration('log_nav_status'), value_type=bool),
                        }
                    ],
                )
            ],
        )
    ])
