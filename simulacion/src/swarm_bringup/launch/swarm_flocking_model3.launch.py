from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Keep manual constant for now (same project style)
    num_robots = 10

    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'shrimp_pond_static.sdf')

    desc_pkg_path = get_package_share_directory('swarm_robot_description_model3')
    urdf_file_path = os.path.join(desc_pkg_path, 'urdf', 'CarcasaURDF.urdf')

    with open(urdf_file_path, 'r', encoding='utf-8') as f:
        urdf_xml_text = f.read()

    gazebo_urdf_xml_text = urdf_xml_text.replace(
        'package://swarm_robot_description_model3/',
        f'file://{desc_pkg_path}/',
    )

    resource_paths = [
        worlds_pkg,
        ':',
        desc_pkg_path,
        ':',
        os.path.dirname(desc_pkg_path),
        ':',
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),

        # 1) Gazebo + shrimp pond
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path],
            output='screen',
        ),

        # 2) Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),

        # 3) Global dynamic pose bridge for all controllers
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/world/shrimp_pond_static/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    ],
                    remappings=[
                        ('/world/shrimp_pond_static/dynamic_pose/info', '/swarm/dynamic_tf'),
                    ],
                    output='screen',
                )
            ],
        ),
    ]

    # 4) Spawn + bridges + per-robot controller
    spawn_points = [
        (-6.1, 5.4),
        (-3.2, 6.7),
        (1.5, 5.9),
        (5.3, 6.1),
        (-5.8, 2.3),
        (-1.4, 3.1),
        (2.8, 2.5),
        (6.6, 3.0),
        (-6.9, -0.5),
        (-2.7, -0.8),
        (1.9, -1.5),
        (5.7, -0.9),
        (-4.5, -3.8),
        (-0.9, -4.2),
        (3.4,-4.5),
        (6.2, -5.1),
        (-6.4, -6.2),
        (-2.1, -6.9),
        (1.1, -6.4),
        (4.8, -6.8),
    ]

    for i in range(num_robots):
        robot_name = f'robot{i+1}'
        x_pos, y_pos = spawn_points[i % len(spawn_points)]

        actions.append(
            TimerAction(
                period=2.0 + i * 0.2,
                actions=[
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name=f'{robot_name}_state_publisher',
                        namespace=robot_name,
                        output='screen',
                        parameters=[
                            {'robot_description': urdf_xml_text},
                            {'use_sim_time': True},
                        ],
                    )
                ],
            )
        )

        actions.append(
            TimerAction(
                period=5.0 + i * 0.6,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-name', robot_name,
                            '-string', gazebo_urdf_xml_text,
                            '-x', str(round(x_pos, 3)),
                            '-y', str(round(y_pos, 3)),
                            '-z', '0.10',
                        ],
                        output='screen',
                    )
                ],
            )
        )

        actions.append(
            TimerAction(
                period=6.5 + i * 0.2,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        ],
                        output='screen',
                    )
                ],
            )
        )

        actions.append(
            TimerAction(
                period=6.5 + i * 0.2,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        ],
                        remappings=[
                            (f'/model/{robot_name}/odometry', f'/{robot_name}/odometry'),
                        ],
                        output='screen',
                    )
                ],
            )
        )

        actions.append(
            TimerAction(
                period=8.0 + i * 0.2,
                actions=[
                    Node(
                        package='swarm_control_model3',
                        executable='cmd_vel_swarm',
                        name='cmd_vel_swarm',
                        namespace=robot_name,
                        output='screen',
                        parameters=[
                            {'robot_id': i},
                            {'num_robots': num_robots},
                            {'cmd_topic': f'/model/{robot_name}/cmd_vel'},
                            {'odom_topic': f'/{robot_name}/odometry'},
                            {'dynamic_tf_topic': '/swarm/dynamic_tf'},
                            {'initial_x': float(x_pos)},
                            {'initial_y': float(y_pos)},
                            {'use_tf_proximity_fallback': True},
                            {'base_linear_x': 0.10},
                            {'min_linear_x': 0.05},
                            {'max_linear_x': 0.14},
                            {'max_angular_z': 0.18},
                            {'max_linear_accel': 0.14},
                            {'max_angular_accel': 0.25},
                            {'neighbor_distance': 2.5},
                            {'separation_distance': 0.8},
                            {'separation_gain': 1.3},
                            {'alignment_gain': 0.7},
                            {'cohesion_gain': 0.45},
                            {'forward_bias_gain': 0.45},
                            {'heading_gain': 1.25},
                            {'turn_slowdown': 0.5},
                            {'state_timeout_s': 1.2},
                            {'require_state': True},
                            {'geofence_enabled': True},
                            {'geofence_center_x': 0.0},
                            {'geofence_center_y': 0.0},
                            {'geofence_radius_m': 6.0},
                            {'duration_s': 0.0},
                            {'publish_rate_hz': 20.0},
                            {'log_period_s': 10.0},
                            {'use_sim_time': True},
                        ],
                    )
                ],
            )
        )

    return LaunchDescription(actions)
