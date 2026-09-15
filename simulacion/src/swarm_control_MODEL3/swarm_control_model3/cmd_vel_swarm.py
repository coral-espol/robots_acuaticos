#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class CmdVelSwarm(Node):
    """Controlador de enjambre suave por cmd_vel (boids 2D) para un robot."""

    def __init__(self):
        super().__init__('cmd_vel_swarm')

        # Identidad del robot
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 5)

        # Topics
        self.declare_parameter('cmd_topic', '/model/robot1/cmd_vel')
        self.declare_parameter('odom_topic', '/robot1/odometry')
        self.declare_parameter('dynamic_tf_topic', '/swarm/dynamic_tf')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('use_tf_proximity_fallback', True)

        # Velocidades y suavizado
        self.declare_parameter('base_linear_x', 0.10)
        self.declare_parameter('min_linear_x', 0.05)
        self.declare_parameter('max_linear_x', 0.14)
        self.declare_parameter('max_angular_z', 0.18)
        self.declare_parameter('max_linear_accel', 0.14)
        self.declare_parameter('max_angular_accel', 0.25)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Reglas de enjambre
        self.declare_parameter('neighbor_distance', 2.5)
        self.declare_parameter('separation_distance', 0.8)
        self.declare_parameter('separation_gain', 1.3)
        self.declare_parameter('alignment_gain', 0.7)
        self.declare_parameter('cohesion_gain', 0.45)
        self.declare_parameter('forward_bias_gain', 0.45)
        self.declare_parameter('heading_gain', 1.25)
        self.declare_parameter('turn_slowdown', 0.5)

        # Seguridad / estado
        self.declare_parameter('state_timeout_s', 1.2)
        self.declare_parameter('require_state', True)
        self.declare_parameter('geofence_enabled', True)
        self.declare_parameter('geofence_center_x', 0.0)
        self.declare_parameter('geofence_center_y', 0.0)
        self.declare_parameter('geofence_radius_m', 6.0)

        # Runtime
        self.declare_parameter('duration_s', 0.0)
        self.declare_parameter('log_period_s', 10.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.robot_name = f'robot{self.robot_id + 1}'

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.dynamic_tf_topic = str(self.get_parameter('dynamic_tf_topic').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.use_tf_proximity_fallback = bool(self.get_parameter('use_tf_proximity_fallback').value)

        self.base_linear_x = float(self.get_parameter('base_linear_x').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.max_linear_accel = max(0.01, float(self.get_parameter('max_linear_accel').value))
        self.max_angular_accel = max(0.01, float(self.get_parameter('max_angular_accel').value))
        self.publish_rate_hz = max(5.0, float(self.get_parameter('publish_rate_hz').value))

        self.neighbor_distance = max(0.1, float(self.get_parameter('neighbor_distance').value))
        self.separation_distance = max(0.1, float(self.get_parameter('separation_distance').value))
        self.separation_gain = float(self.get_parameter('separation_gain').value)
        self.alignment_gain = float(self.get_parameter('alignment_gain').value)
        self.cohesion_gain = float(self.get_parameter('cohesion_gain').value)
        self.forward_bias_gain = float(self.get_parameter('forward_bias_gain').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.turn_slowdown = self._clamp(float(self.get_parameter('turn_slowdown').value), 0.0, 1.0)

        self.state_timeout_s = max(0.1, float(self.get_parameter('state_timeout_s').value))
        self.require_state = bool(self.get_parameter('require_state').value)
        self.geofence_enabled = bool(self.get_parameter('geofence_enabled').value)
        self.geofence_center_x = float(self.get_parameter('geofence_center_x').value)
        self.geofence_center_y = float(self.get_parameter('geofence_center_y').value)
        self.geofence_radius_m = max(0.5, float(self.get_parameter('geofence_radius_m').value))

        self.duration_s = float(self.get_parameter('duration_s').value)
        self.log_period_s = max(2.0, float(self.get_parameter('log_period_s').value))

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        # Own odometry (self-state robusta)
        self.create_subscription(Odometry, self.odom_topic, self._self_odom_cb, 10)
        # Odometría de todos los robots para interacción de enjambre
        for i in range(self.num_robots):
            self.create_subscription(
                Odometry,
                f'/robot{i+1}/odometry',
                lambda msg, rid=i: self._neighbor_odom_cb(msg, rid),
                10,
            )
        # Dynamic TF opcional como respaldo
        self.create_subscription(TFMessage, self.dynamic_tf_topic, self._dynamic_tf_cb, 10)

        # Estado por robot: {id: {'x','y','yaw','stamp'}}
        self.states: Dict[int, Dict[str, object]] = {}
        self.last_state_msg_ros = None

        # Estado controlador
        self.start_ros = None
        self.last_tick_ros = None
        self.last_log_ros = None
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        dt = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(dt, self._tick)

        duration_txt = 'infinite' if self.duration_s <= 0.0 else f'{self.duration_s:.1f}s'
        self.get_logger().info(
            f'cmd_vel_swarm started: robot={self.robot_name}, num_robots={self.num_robots}, '
            f'cmd_topic={self.cmd_topic}, tf_topic={self.dynamic_tf_topic}, '
            f'odom_topic={self.odom_topic}, '
            f'v_base={self.base_linear_x:.3f}, max_w={self.max_angular_z:.3f}, '
            f'require_state={self.require_state}, geofence_r={self.geofence_radius_m:.1f}m, '
            f'duration={duration_txt}'
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
        # Si ya hay odom fresca, no sobrescribirla con TF
        if prev is not None and prev.get('source') == 'odom' and source == 'tf':
            if (stamp - prev['stamp']) <= Duration(seconds=self.state_timeout_s):
                return

        self.states[rid] = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'stamp': stamp,
            'source': source,
        }

    def _neighbor_odom_cb(self, msg: Odometry, rid: int):
        now_ros = self.get_clock().now()
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self._update_state(
            rid,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
            now_ros,
            'odom',
        )

    def _self_odom_cb(self, msg: Odometry):
        now_ros = self.get_clock().now()
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self._update_state(
            self.robot_id,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
            now_ros,
            'odom',
        )

    def _dynamic_tf_cb(self, msg: TFMessage):
        now_ros = self.get_clock().now()
        self.last_state_msg_ros = now_ros
        self_seen = False
        unlabeled_candidates: List[Tuple[float, float, float]] = []

        for t in msg.transforms:
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            yaw = self._yaw_from_quat(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )

            rid = self._parse_robot_id(t.child_frame_id)
            if rid is None:
                rid = self._parse_robot_id(t.header.frame_id)
            if rid is None:
                unlabeled_candidates.append((x, y, yaw))
                continue
            self._update_state(
                rid,
                x,
                y,
                yaw,
                now_ros,
                'tf',
            )
            if rid == self.robot_id:
                self_seen = True

        # Fallback: si TF no trae nombres de robot, estimar self por cercanía
        if (not self_seen) and self.use_tf_proximity_fallback and unlabeled_candidates:
            me_prev = self.states.get(self.robot_id)
            if me_prev is not None:
                ref_x = float(me_prev['x'])
                ref_y = float(me_prev['y'])
            else:
                ref_x = self.initial_x
                ref_y = self.initial_y

            best = min(
                unlabeled_candidates,
                key=lambda c: (c[0] - ref_x) ** 2 + (c[1] - ref_y) ** 2,
            )
            self._update_state(self.robot_id, best[0], best[1], best[2], now_ros, 'tf_fallback')

    def _fresh_state(self, rid: int, now_ros) -> Optional[Dict[str, object]]:
        st = self.states.get(rid)
        if st is None:
            return None
        if (now_ros - st['stamp']) > Duration(seconds=self.state_timeout_s):
            return None
        return st

    def _compute_swarm_vector(self, me: Dict[str, object], neighbors: List[Tuple[int, Dict[str, object]]]) -> Tuple[float, float]:
        mx = float(me['x'])
        my = float(me['y'])
        myaw = float(me['yaw'])

        sep_x = 0.0
        sep_y = 0.0
        coh_x = 0.0
        coh_y = 0.0
        ali_x = 0.0
        ali_y = 0.0

        close_count = 0
        neigh_count = 0

        for _, st in neighbors:
            nx = float(st['x'])
            ny = float(st['y'])
            nyaw = float(st['yaw'])

            dx = nx - mx
            dy = ny - my
            d = math.hypot(dx, dy)
            if d < 1e-6 or d > self.neighbor_distance:
                continue

            neigh_count += 1
            coh_x += nx
            coh_y += ny
            ali_x += math.cos(nyaw)
            ali_y += math.sin(nyaw)

            if d < self.separation_distance:
                close_count += 1
                # repulsión fuerte a corta distancia
                scale = (self.separation_distance - d) / self.separation_distance
                sep_x -= (dx / d) * scale
                sep_y -= (dy / d) * scale

        vx = 0.0
        vy = 0.0

        if close_count > 0:
            vx += self.separation_gain * sep_x
            vy += self.separation_gain * sep_y

        if neigh_count > 0:
            cx = coh_x / neigh_count
            cy = coh_y / neigh_count
            vx += self.cohesion_gain * (cx - mx)
            vy += self.cohesion_gain * (cy - my)

            ax = ali_x / neigh_count
            ay = ali_y / neigh_count
            vx += self.alignment_gain * ax
            vy += self.alignment_gain * ay

        # Sesgo de avance para evitar paradas / oscilaciones
        vx += self.forward_bias_gain * math.cos(myaw)
        vy += self.forward_bias_gain * math.sin(myaw)

        # Geocerca: tracción al centro si sale del radio
        if self.geofence_enabled:
            cdx = self.geofence_center_x - mx
            cdy = self.geofence_center_y - my
            cdist = math.hypot(cdx, cdy)
            if cdist > self.geofence_radius_m:
                pull = min(1.0, (cdist - self.geofence_radius_m) / max(0.1, self.geofence_radius_m))
                vx += 1.2 * pull * (cdx / cdist)
                vy += 1.2 * pull * (cdy / cdist)

        return vx, vy

    def _publish_stop(self):
        self.pub.publish(Twist())

    def _tick(self):
        now_ros = self.get_clock().now()
        if self.start_ros is None:
            self.start_ros = now_ros
            self.last_tick_ros = now_ros
            self.last_log_ros = now_ros

        dt = max(1e-3, (now_ros - self.last_tick_ros).nanoseconds / 1e9)
        self.last_tick_ros = now_ros

        elapsed_s = (now_ros - self.start_ros).nanoseconds / 1e9
        if self.duration_s > 0.0 and elapsed_s > self.duration_s:
            self._publish_stop()
            return

        me = self._fresh_state(self.robot_id, now_ros)
        if self.require_state and me is None:
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self._publish_stop()
            if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
                self.last_log_ros = now_ros
                self.get_logger().warn('deadman active: no fresh self state')
            return

        if me is None:
            # Sin estado propio fresco y deadman deshabilitado: comando mínimo seguro
            msg = Twist()
            msg.linear.x = self.min_linear_x
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return

        neighbors: List[Tuple[int, Dict[str, object]]] = []
        for rid in range(self.num_robots):
            if rid == self.robot_id:
                continue
            st = self._fresh_state(rid, now_ros)
            if st is not None:
                neighbors.append((rid, st))

        svx, svy = self._compute_swarm_vector(me, neighbors)
        desired_heading = math.atan2(svy, svx)
        heading_err = self._norm_angle(desired_heading - float(me['yaw']))

        target_angular = self._clamp(self.heading_gain * heading_err, -self.max_angular_z, self.max_angular_z)

        # Menos velocidad lineal cuando está girando mucho
        turn_ratio = abs(target_angular) / max(1e-6, self.max_angular_z)
        target_linear = self.base_linear_x * (1.0 - self.turn_slowdown * turn_ratio)
        target_linear = self._clamp(target_linear, self.min_linear_x, self.max_linear_x)

        # Slew-rate limiting
        lin_step = self.max_linear_accel * dt
        ang_step = self.max_angular_accel * dt

        if self.cmd_linear_x < target_linear:
            self.cmd_linear_x = min(self.cmd_linear_x + lin_step, target_linear)
        else:
            self.cmd_linear_x = max(self.cmd_linear_x - lin_step, target_linear)

        if self.cmd_angular_z < target_angular:
            self.cmd_angular_z = min(self.cmd_angular_z + ang_step, target_angular)
        else:
            self.cmd_angular_z = max(self.cmd_angular_z - ang_step, target_angular)

        self.cmd_linear_x = self._clamp(self.cmd_linear_x, self.min_linear_x, self.max_linear_x)
        self.cmd_angular_z = self._clamp(self.cmd_angular_z, -self.max_angular_z, self.max_angular_z)

        msg = Twist()
        msg.linear.x = self.cmd_linear_x
        msg.angular.z = self.cmd_angular_z
        self.pub.publish(msg)

        if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
            self.last_log_ros = now_ros
            self.get_logger().info(
                f'swarm: neighbors={len(neighbors)}, v={self.cmd_linear_x:.3f}, '
                f'w={self.cmd_angular_z:.3f}, heading_err={heading_err:.3f}'
            )



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSwarm()
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
