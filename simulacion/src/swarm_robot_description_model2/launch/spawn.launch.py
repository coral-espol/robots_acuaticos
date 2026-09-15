from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='aquatic_robot',
            description='Name of the robot'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial x position'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial y position'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'z',
            default_value='0.5',
            description='Initial z position'
        )
    )

    # Get parameters
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('swarm_robot_description_model2'),
            'urdf',
            'aquatic_robot_model2.urdf.xacro'
        ]),
        ' prefix:=', robot_name, '_'
    ])

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True}
        ]
    )

    # Spawn entity in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', [robot_name, '/robot_description'],
            '-x', x,
            '-y', y,
            '-z', z
        ],
        output='screen'
    )

    # Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [robot_name, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ],
        output='screen'
    )

    # Bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            [robot_name, '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry']
        ],
        output='screen'
    )

    nodes = [
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_cmd_vel,
        bridge_odom,
    ]

    return LaunchDescription(declared_arguments + nodes)
