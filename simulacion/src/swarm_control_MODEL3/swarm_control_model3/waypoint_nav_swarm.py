#!/usr/bin/env python3

"""
waypoint_nav_swarm.py — Versión simulada del algoritmo real de navegación
(ver navegacion.py de la Raspberry Pi): navegación autónoma por waypoints
con umbral de giro (bang-bang) + avance con corrección proporcional, y
evasión reactiva simple de colisiones cuando otro robot del enjambre está
demasiado cerca.

No es una réplica fiel de la Raspberry (no hay UART/GPS/IMU reales): la
posición y el rumbo se toman de la odometría del propio robot (equivalente
al GPS propio) y de la de los demás robots (equivalente al enjambre
recibido por radio). El "estado" y su lógica de frescura/deadman se toman
del mismo patrón usado en cmd_vel_swarm.py.
"""

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class WaypointNavSwarm(Node):
    """Controlador de navegación por waypoints + evasión reactiva (por robot)."""

    def __init__(self):
        super().__init__('waypoint_nav_swarm')

        # Identidad del robot
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 5)

        # Topics
        self.declare_parameter('cmd_topic', '/model/robot1/cmd_vel')
        self.declare_parameter('odom_topic', '/robot1/odometry')
        self.declare_parameter('dynamic_tf_topic', '/swarm/dynamic_tf')
        self.declare_parameter('nav_status_topic', 'nav_status')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('use_tf_proximity_fallback', True)

        # Ruta de waypoints de este robot, en el plano local x/y (metros),
        # análogo a la lista WAYPOINTS (lat, lon) de navegacion.py.
        self.declare_parameter('waypoints_x', [0.0])
        self.declare_parameter('waypoints_y', [0.0])

        # Navegación (equivalentes a navegacion.py)
        self.declare_parameter('radio_llegada', 2.5)       # RADIO_LLEGADA
        self.declare_parameter('umbral_giro_deg', 25.0)    # UMBRAL_GIRO
        self.declare_parameter('heading_kp', 1.0)          # KP (adaptado a rad/s)
        self.declare_parameter('vel_base', 0.10)           # VEL_BASE
        self.declare_parameter('turn_angular_z', 0.18)     # velocidad angular al girar en sitio

        # Evasión de colisiones (equivalentes a navegacion.py)
        self.declare_parameter('dist_seguridad', 1.2)      # DIST_SEGURIDAD
        self.declare_parameter('t_evasion_recto', 3.0)     # T_EVASION_RECTO

        # Seguridad / estado (equivalente al deadman por GPS viejo: T_GPS)
        self.declare_parameter('state_timeout_s', 1.2)
        self.declare_parameter('require_state', True)

        # Runtime
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('log_period_s', 10.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.robot_name = f'robot{self.robot_id + 1}'

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.dynamic_tf_topic = str(self.get_parameter('dynamic_tf_topic').value)
        self.nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.use_tf_proximity_fallback = bool(self.get_parameter('use_tf_proximity_fallback').value)

        wx = [float(v) for v in self.get_parameter('waypoints_x').value]
        wy = [float(v) for v in self.get_parameter('waypoints_y').value]
        self.waypoints: List[Tuple[float, float]] = list(zip(wx, wy))

        self.radio_llegada = max(0.05, float(self.get_parameter('radio_llegada').value))
        self.umbral_giro_rad = math.radians(float(self.get_parameter('umbral_giro_deg').value))
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.vel_base = float(self.get_parameter('vel_base').value)
        self.turn_angular_z = abs(float(self.get_parameter('turn_angular_z').value))

        self.dist_seguridad = max(0.05, float(self.get_parameter('dist_seguridad').value))
        self.t_evasion_recto = max(0.1, float(self.get_parameter('t_evasion_recto').value))

        self.state_timeout_s = max(0.1, float(self.get_parameter('state_timeout_s').value))
        self.require_state = bool(self.get_parameter('require_state').value)

        self.publish_rate_hz = max(5.0, float(self.get_parameter('publish_rate_hz').value))
        self.log_period_s = max(2.0, float(self.get_parameter('log_period_s').value))

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.nav_status_topic, 10)

        # Own odometry (equivalente al GPS propio)
        self.create_subscription(Odometry, self.odom_topic, self._self_odom_cb, 10)
        # Odometría de todos los robots (equivalente a las posiciones de enjambre por radio)
        for i in range(self.num_robots):
            self.create_subscription(
                Odometry,
                f'/robot{i+1}/odometry',
                lambda msg, rid=i: self._neighbor_odom_cb(msg, rid),
                10,
            )
        # Dynamic TF opcional como respaldo
        self.create_subscription(TFMessage, self.dynamic_tf_topic, self._dynamic_tf_cb, 10)

        # Estado por robot: {id: {'x','y','yaw','stamp','source'}}
        self.states: Dict[int, Dict[str, object]] = {}

        # Estado de navegación (máquina de estados no bloqueante,
        # equivalente al lazo bloqueante de navegar()/maniobra_evasion())
        self.wp_index = 0
        self.nav_state = 'NORMAL'  # NORMAL | EVADE_TURN | EVADE_STRAIGHT | DONE
        self.evade_target_heading = 0.0
        self.evade_straight_start_ros = None
        self.evasions_count = 0

        self.start_ros = None
        self.last_log_ros = None

        dt = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'waypoint_nav_swarm started: robot={self.robot_name}, num_robots={self.num_robots}, '
            f'waypoints={len(self.waypoints)}, radio_llegada={self.radio_llegada:.2f}m, '
            f'umbral_giro={math.degrees(self.umbral_giro_rad):.1f}deg, '
            f'vel_base={self.vel_base:.3f}, dist_seguridad={self.dist_seguridad:.2f}m, '
            f'require_state={self.require_state}'
        )

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _norm_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _parse_robot_id(self, child_frame_id: str) -> Optional[int]:
        for i in range(self.num_robots):
            token = f'robot{i+1}'
            if token in child_frame_id:
                return i
        return None

    def _update_state(self, rid: int, x: float, y: float, yaw: float, stamp, source: str):
        prev = self.states.get(rid)
        if prev is not None and prev.get('source') == 'odom' and source == 'tf':
            if (stamp - prev['stamp']) <= Duration(seconds=self.state_timeout_s):
                return

        self.states[rid] = {'x': x, 'y': y, 'yaw': yaw, 'stamp': stamp, 'source': source}

    def _neighbor_odom_cb(self, msg: Odometry, rid: int):
        now_ros = self.get_clock().now()
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self._update_state(rid, float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw, now_ros, 'odom')

    def _self_odom_cb(self, msg: Odometry):
        now_ros = self.get_clock().now()
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self._update_state(self.robot_id, float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw, now_ros, 'odom')

    def _dynamic_tf_cb(self, msg: TFMessage):
        now_ros = self.get_clock().now()
        self_seen = False
        unlabeled_candidates: List[Tuple[float, float, float]] = []

        for t in msg.transforms:
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            yaw = self._yaw_from_quat(
                t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w,
            )
            rid = self._parse_robot_id(t.child_frame_id)
            if rid is None:
                rid = self._parse_robot_id(t.header.frame_id)
            if rid is None:
                unlabeled_candidates.append((x, y, yaw))
                continue
            self._update_state(rid, x, y, yaw, now_ros, 'tf')
            if rid == self.robot_id:
                self_seen = True

        if (not self_seen) and self.use_tf_proximity_fallback and unlabeled_candidates:
            me_prev = self.states.get(self.robot_id)
            if me_prev is not None:
                ref_x, ref_y = float(me_prev['x']), float(me_prev['y'])
            else:
                ref_x, ref_y = self.initial_x, self.initial_y

            best = min(unlabeled_candidates, key=lambda c: (c[0] - ref_x) ** 2 + (c[1] - ref_y) ** 2)
            self._update_state(self.robot_id, best[0], best[1], best[2], now_ros, 'tf_fallback')

    def _fresh_state(self, rid: int, now_ros) -> Optional[Dict[str, object]]:
        st = self.states.get(rid)
        if st is None:
            return None
        if (now_ros - st['stamp']) > Duration(seconds=self.state_timeout_s):
            return None
        return st

    def _nearest_threat(self, me: Dict[str, object], now_ros) -> Optional[Tuple[int, float, float]]:
        """Vecino vigente más cercano dentro de dist_seguridad -> (rid, dist, bearing)."""
        mx, my = float(me['x']), float(me['y'])
        best = None
        for rid in range(self.num_robots):
            if rid == self.robot_id:
                continue
            st = self._fresh_state(rid, now_ros)
            if st is None:
                continue
            dx = float(st['x']) - mx
            dy = float(st['y']) - my
            d = math.hypot(dx, dy)
            if d < self.dist_seguridad and (best is None or d < best[1]):
                best = (rid, d, math.atan2(dy, dx))
        return best

    def _publish_stop(self):
        self.pub.publish(Twist())

    def _publish_status(self, label: str, dist_to_wp: Optional[float], heading_err_deg: Optional[float]):
        payload = {
            'state': label,
            'wp_index': self.wp_index,
            'num_waypoints': len(self.waypoints),
            'dist_to_wp': round(dist_to_wp, 3) if dist_to_wp is not None else None,
            'heading_error_deg': round(heading_err_deg, 2) if heading_err_deg is not None else None,
            'evasions': self.evasions_count,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _tick(self):
        now_ros = self.get_clock().now()
        if self.start_ros is None:
            self.start_ros = now_ros
            self.last_log_ros = now_ros

        if not self.waypoints:
            self._publish_stop()
            self._publish_status('SIN_WAYPOINTS', None, None)
            return

        me = self._fresh_state(self.robot_id, now_ros)
        if self.require_state and me is None:
            self._publish_stop()
            self._publish_status('DEADMAN', None, None)
            if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
                self.last_log_ros = now_ros
                self.get_logger().warn('deadman active: no fresh self state')
            return

        if me is None:
            self._publish_stop()
            self._publish_status('SIN_ESTADO', None, None)
            return

        linear = 0.0
        angular = 0.0
        label = 'NORMAL'
        dist_to_wp = None
        heading_err_deg = None

        # DONE: ya se recorrieron todos los waypoints
        if self.nav_state == 'DONE' or self.wp_index >= len(self.waypoints):
            self.nav_state = 'DONE'
            label = 'DONE'

        elif self.nav_state == 'EVADE_TURN':
            err = self._norm_angle(self.evade_target_heading - float(me['yaw']))
            heading_err_deg = math.degrees(err)
            if abs(err) > self.umbral_giro_rad:
                angular = math.copysign(self.turn_angular_z, err)
                label = 'EVADIENDO_GIRO'
            else:
                self.nav_state = 'EVADE_STRAIGHT'
                self.evade_straight_start_ros = now_ros
                linear = self.vel_base
                label = 'EVADIENDO_RECTO'

        elif self.nav_state == 'EVADE_STRAIGHT':
            elapsed_s = (now_ros - self.evade_straight_start_ros).nanoseconds / 1e9
            err = self._norm_angle(self.evade_target_heading - float(me['yaw']))
            heading_err_deg = math.degrees(err)
            if elapsed_s >= self.t_evasion_recto:
                self.nav_state = 'NORMAL'
                label = 'NORMAL'
            else:
                linear = self.vel_base
                angular = self._clamp(self.heading_kp * err, -self.turn_angular_z, self.turn_angular_z)
                label = 'EVADIENDO_RECTO'

        # NORMAL (o se acaba de salir de evasión en este mismo tick)
        if self.nav_state == 'NORMAL' and label != 'DONE':
            wp_x, wp_y = self.waypoints[self.wp_index]
            dx = wp_x - float(me['x'])
            dy = wp_y - float(me['y'])
            dist_to_wp = math.hypot(dx, dy)

            if dist_to_wp < self.radio_llegada:
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints):
                    self.nav_state = 'DONE'
                    label = 'DONE'
                    dist_to_wp = None
                else:
                    wp_x, wp_y = self.waypoints[self.wp_index]
                    dx = wp_x - float(me['x'])
                    dy = wp_y - float(me['y'])
                    dist_to_wp = math.hypot(dx, dy)

            if label != 'DONE':
                threat = self._nearest_threat(me, now_ros)
                if threat is not None:
                    _, _, bearing_to_threat = threat
                    rel = self._norm_angle(bearing_to_threat - float(me['yaw']))
                    # Girar hacia el lado contrario al robot detectado
                    escape = float(me['yaw']) - math.pi / 2.0 if rel > 0 else float(me['yaw']) + math.pi / 2.0
                    self.evade_target_heading = self._norm_angle(escape)
                    self.nav_state = 'EVADE_TURN'
                    self.evasions_count += 1
                    label = 'EVADIENDO_GIRO'
                    err = self._norm_angle(self.evade_target_heading - float(me['yaw']))
                    heading_err_deg = math.degrees(err)
                    angular = math.copysign(self.turn_angular_z, err)
                    linear = 0.0
                else:
                    desired_heading = math.atan2(dy, dx)
                    err = self._norm_angle(desired_heading - float(me['yaw']))
                    heading_err_deg = math.degrees(err)
                    if abs(err) > self.umbral_giro_rad:
                        linear = 0.0
                        angular = math.copysign(self.turn_angular_z, err)
                        label = 'GIRANDO'
                    else:
                        linear = self.vel_base
                        angular = self._clamp(self.heading_kp * err, -self.turn_angular_z, self.turn_angular_z)
                        label = 'AVANZANDO'

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)
        self._publish_status(label, dist_to_wp, heading_err_deg)

        if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
            self.last_log_ros = now_ros
            self.get_logger().info(
                f'nav: state={label}, wp={self.wp_index}/{len(self.waypoints)}, '
                f'dist={dist_to_wp if dist_to_wp is not None else -1:.2f}, '
                f'v={linear:.3f}, w={angular:.3f}, evasions={self.evasions_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavSwarm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
