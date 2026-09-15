from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='5',
        description='Number of robots in swarm'
    )
    
    num_robots = 5  # Change manually for now
    actions = [num_robots_arg]

    # Package paths
    desc_pkg = FindPackageShare('swarm_robot_description_model2')
    xacro_path = PathJoinSubstitution([desc_pkg, 'urdf', 'aquatic_robot_model2.urdf.xacro'])

    # Create robots and controllers
    for i in range(num_robots):
        robot_name = f'robot{i+1}'

        # URDF for each robot
        urdf_xml = Command([
            'xacro', ' ', xacro_path,
            ' prefix:=', robot_name, '_'
        ])
        
        # Robot State Publisher
        actions.append(
            TimerAction(
                period=5.0 + i * 0.5,
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
        x_pos = 2.0 * (i - num_robots/2)
        actions.append(
            TimerAction(
                period=8.0 + i * 1.0,
                actions=[
                    Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=[
                            '-name', robot_name,
                            '-topic', f'{robot_name}/robot_description',
                            '-x', str(x_pos), '-y', '0', '-z', '0.5'
                        ],
                        output='screen'
                    )
                ]
            )
        )

        # Bridge cmd_vel
        actions.append(
            TimerAction(
                period=10.0 + i * 0.5,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            f'{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                        ],
                        output='screen'
                    )
                ]
            )
        )

        # Bridge odom
        actions.append(
            TimerAction(
                period=10.0 + i * 0.5,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            f'{robot_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                        ],
                        output='screen'
                    )
                ]
            )
        )

        # Differential flocking controller
        actions.append(
            TimerAction(
                period=15.0 + i * 0.2,
                actions=[
                    Node(
                        package='swarm_control_model2',
                        executable='differential_flocking',
                        name=f'{robot_name}_flocking',
                        namespace=robot_name,
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
