from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    worlds_pkg = get_package_share_directory('swarm_worlds')
    world_path = os.path.join(worlds_pkg, 'worlds', 'shrimp_pond_static.sdf')

    desc_pkg_path = get_package_share_directory('swarm_robot_description_model3')
    urdf_file_path = os.path.join(desc_pkg_path, 'urdf', 'CarcasaURDF.urdf')

    with open(urdf_file_path, 'r', encoding='utf-8') as f:
        urdf_xml_text = f.read()

    # Use absolute file:// URIs for Gazebo mesh loading robustness
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

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            resource_paths,
        ),

        # 1) Start Gazebo with the shrimp pond world
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

        # 3) Robot state publisher
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot1_state_publisher',
                    namespace='robot1',
                    output='screen',
                    parameters=[
                        {'robot_description': urdf_xml_text},
                        {'use_sim_time': True},
                    ],
                )
            ],
        ),

        # 4) Spawn a single robot
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'robot1',
                        '-string', gazebo_urdf_xml_text,
                        '-x', '0.0', '-y', '0.0', '-z', '0.10',
                    ],
                    output='screen',
                )
            ],
        ),

        # 5) Bridge cmd_vel ROS -> Gazebo (VelocityControl)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/model/robot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    ],
                    output='screen',
                )
            ],
        ),

        # 5b) Bridge odometry Gazebo -> ROS
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/model/robot1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    ],
                    remappings=[
                        ('/model/robot1/odometry', '/robot1/odometry'),
                    ],
                    output='screen',
                )
            ],
        ),

        # 5c) Bridge dynamic pose Gazebo -> ROS (TF fallback estimator)
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
                        ('/world/shrimp_pond_static/dynamic_pose/info', '/robot1/dynamic_tf'),
                    ],
                    output='screen',
                )
            ],
        ),

        # 6) Cmd_vel wander controller (single robot, bounded + safe)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='swarm_control_model3',
                    executable='cmd_vel_wander',
                    name='cmd_vel_wander',
                    namespace='robot1',
                    output='screen',
                    parameters=[
                        {'cmd_topic': '/model/robot1/cmd_vel'},
                        {'odom_topic': '/robot1/odometry'},
                        {'dynamic_tf_topic': '/robot1/dynamic_tf'},
                        {'dynamic_tf_child_contains': 'robot1'},
                        {'base_linear_x': 0.10},
                        {'min_linear_x': 0.05},
                        {'max_linear_x': 0.12},
                        {'turn_amplitude': 0.08},
                        {'max_angular_z': 0.16},
                        {'turn_update_period_s': 3.0},
                        {'max_linear_accel': 0.14},
                        {'max_angular_accel': 0.22},
                        {'state_timeout_s': 1.2},
                        {'require_state': True},
                        {'deadman_source': 'dynamic_tf'},
                        {'geofence_enabled': True},
                        {'geofence_center_x': 0.0},
                        {'geofence_center_y': 0.0},
                        {'geofence_radius_m': 5.0},
                        {'spin_threshold_rad_s': 0.35},
                        {'log_period_s': 10.0},
                        {'duration_s': 0.0},
                        {'publish_rate_hz': 20.0},
                        {'use_sim_time': True},
                    ],
                )
            ],
        ),
    ])
