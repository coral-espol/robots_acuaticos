from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Simulación del algoritmo REAL de navegación por waypoints + evasión
# reactiva (ver navegacion.py de la Raspberry Pi), corriendo con el nodo
# waypoint_nav_swarm en lugar del flocking (cmd_vel_swarm) de
# swarm_flocking_model3.launch.py.
#
# Para configurar una prueba: edita las constantes de abajo (cantidad de
# robots, posiciones de spawn, ruta de waypoints por robot y parámetros de
# navegación/evasión). Todo es manual a propósito, mismo estilo que
# swarm_flocking_model3.launch.py.


def generate_launch_description():
    num_robots = 3

    # Puntos de spawn (mismos que swarm_flocking_model3.launch.py, dentro
    # del radio navegable de la piscina).
    spawn_points = [
        (-6.1, 5.4), (-3.2, 6.7), (1.5, 5.9), (5.3, 6.1),
        (-5.8, 2.3), (-1.4, 3.1), (2.8, 2.5), (6.6, 3.0),
        (-6.9, -0.5), (-2.7, -0.8), (1.9, -1.5), (5.7, -0.9),
        (-4.5, -3.8), (-0.9, -4.2), (3.4, -4.5), (6.2, -5.1),
        (-6.4, -6.2), (-2.1, -6.9), (1.1, -6.4), (4.8, -6.8),
    ]

    # Prueba: ida en línea recta/diagonal hacia un punto lejano + regreso a
    # base (HOME). Un punto ABSOLUTO (x, y) por robot, en el mismo plano que
    # spawn_points; se arma la ruta como [target, home] (ida y vuelta).
    # robot1 (abajo-izquierda): diagonal hacia el centro del world, pero
    #   pasándose un poco por debajo de este.
    # robot2 (medio): línea recta hacia el centro exacto del world.
    # robot3 (arriba): diagonal hacia el centro en X, quedándose unos
    #   metros por ENCIMA de su propia altura de spawn (diagonal corta).
    robot_targets = [
        (-8.0, 0.0),
        (-3.4, 0.0),
        (3.6, 0.0),
    ]

    # Parámetros de navegación / evasión (equivalentes a navegacion.py)
    nav_params = {
        'radio_llegada': 1.0,        # RADIO_LLEGADA
        'umbral_giro_deg': 25.0,     # UMBRAL_GIRO
        'heading_kp': 1.0,           # KP (adaptado a rad/s en vez de PWM)
        'vel_base': 0.258,           # VEL_BASE (velocidad promedio medida en el robot real)
        'turn_angular_z': 0.18,      # velocidad angular fija al girar en sitio
        'dist_seguridad': 1.0,       # DIST_SEGURIDAD
        't_evasion_recto': 3.0,      # T_EVASION_RECTO
        'state_timeout_s': 1.2,      # equivalente a T_GPS (deadman)
        'require_state': True,
        'publish_rate_hz': 20.0,
        'log_period_s': 10.0,
    }

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

        # 3) Global dynamic pose bridge para respaldo de todos los controladores
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

    for i in range(num_robots):
        robot_name = f'robot{i+1}'
        x_pos, y_pos = spawn_points[i % len(spawn_points)]
        target_x, target_y = robot_targets[i % len(robot_targets)]

        # Ruta: ida al punto lejano, luego regreso a la posición de spawn (HOME).
        robot_waypoints_x = [round(target_x, 3), round(x_pos, 3)]
        robot_waypoints_y = [round(target_y, 3), round(y_pos, 3)]

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
                        executable='waypoint_nav_swarm',
                        name='waypoint_nav_swarm',
                        namespace=robot_name,
                        output='screen',
                        parameters=[
                            {
                                'robot_id': i,
                                'num_robots': num_robots,
                                'cmd_topic': f'/model/{robot_name}/cmd_vel',
                                'odom_topic': f'/{robot_name}/odometry',
                                'dynamic_tf_topic': '/swarm/dynamic_tf',
                                'initial_x': float(x_pos),
                                'initial_y': float(y_pos),
                                'use_tf_proximity_fallback': True,
                                'waypoints_x': robot_waypoints_x,
                                'waypoints_y': robot_waypoints_y,
                                'use_sim_time': True,
                                **nav_params,
                            }
                        ],
                    )
                ],
            )
        )

    return LaunchDescription(actions)
