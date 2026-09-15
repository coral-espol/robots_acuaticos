#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class CmdVelWander(Node):
    """Wander suave para un solo robot con límites y seguridades básicas."""

    def __init__(self):
        super().__init__('cmd_vel_wander')

        # Topics
        self.declare_parameter('cmd_topic', '/model/robot1/cmd_vel')
        self.declare_parameter('odom_topic', '/robot1/odometry')
        self.declare_parameter('dynamic_tf_topic', '/robot1/dynamic_tf')
        self.declare_parameter('dynamic_tf_child_contains', 'robot1')

        # Velocidades objetivo
        self.declare_parameter('base_linear_x', 0.07)
        self.declare_parameter('max_linear_x', 0.10)
        self.declare_parameter('min_linear_x', 0.03)
        self.declare_parameter('turn_amplitude', 0.10)
        self.declare_parameter('max_angular_z', 0.15)
        self.declare_parameter('turn_update_period_s', 3.5)

        # Suavizado / dinámica de comando
        self.declare_parameter('max_linear_accel', 0.12)
        self.declare_parameter('max_angular_accel', 0.20)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Seguridad
        self.declare_parameter('state_timeout_s', 1.2)
        self.declare_parameter('require_state', True)
        self.declare_parameter('deadman_source', 'dynamic_tf')
        self.declare_parameter('geofence_enabled', True)
        self.declare_parameter('geofence_center_x', 0.0)
        self.declare_parameter('geofence_center_y', 0.0)
        self.declare_parameter('geofence_radius_m', 5.0)

        # Métricas básicas
        self.declare_parameter('spin_threshold_rad_s', 0.35)
        self.declare_parameter('log_period_s', 10.0)

        # Runtime (0=infinito)
        self.declare_parameter('duration_s', 0.0)

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.dynamic_tf_topic = str(self.get_parameter('dynamic_tf_topic').value)
        self.dynamic_tf_child_contains = str(self.get_parameter('dynamic_tf_child_contains').value)

        self.base_linear_x = float(self.get_parameter('base_linear_x').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.turn_amplitude = abs(float(self.get_parameter('turn_amplitude').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.turn_update_period_s = max(0.5, float(self.get_parameter('turn_update_period_s').value))

        self.max_linear_accel = max(0.01, float(self.get_parameter('max_linear_accel').value))
        self.max_angular_accel = max(0.01, float(self.get_parameter('max_angular_accel').value))
        self.publish_rate_hz = max(5.0, float(self.get_parameter('publish_rate_hz').value))

        self.state_timeout_s = max(0.1, float(self.get_parameter('state_timeout_s').value))
        self.require_state = bool(self.get_parameter('require_state').value)
        self.deadman_source = str(self.get_parameter('deadman_source').value).strip().lower()
        if self.deadman_source not in ('dynamic_tf', 'odom', 'either'):
            self.deadman_source = 'dynamic_tf'
        self.geofence_enabled = bool(self.get_parameter('geofence_enabled').value)
        self.geofence_center_x = float(self.get_parameter('geofence_center_x').value)
        self.geofence_center_y = float(self.get_parameter('geofence_center_y').value)
        self.geofence_radius_m = max(0.5, float(self.get_parameter('geofence_radius_m').value))

        self.spin_threshold_rad_s = abs(float(self.get_parameter('spin_threshold_rad_s').value))
        self.log_period_s = max(2.0, float(self.get_parameter('log_period_s').value))

        self.duration_s = float(self.get_parameter('duration_s').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(TFMessage, self.dynamic_tf_topic, self._dynamic_tf_cb, 10)

        # Estado odometría
        self.last_odom_ros = None
        self.last_dynamic_tf_ros = None
        self.last_pos_x = None
        self.last_pos_y = None
        self.cur_pos_x = None
        self.cur_pos_y = None
        self.cur_yaw = 0.0
        self.cur_omega = 0.0

        # Estado controlador
        self.start_ros = None
        self.last_tick_ros = None
        self.last_turn_change_ros = None
        self.last_log_ros = None

        self.target_linear_x = self._clamp(self.base_linear_x, self.min_linear_x, self.max_linear_x)
        self.target_angular_z = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Métricas
        self.total_distance = 0.0
        self.samples = 0
        self.spin_samples = 0
        self.max_abs_omega = 0.0

        dt = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(dt, self._tick)

        duration_txt = 'infinite' if self.duration_s <= 0.0 else f'{self.duration_s:.1f}s'
        self.get_logger().info(
            'cmd_vel wander started: '
            f'cmd_topic={self.cmd_topic}, odom_topic={self.odom_topic}, dynamic_tf_topic={self.dynamic_tf_topic}, '
            f'v_base={self.base_linear_x:.3f}, turn_amp={self.turn_amplitude:.3f}, '
            f'max_w={self.max_angular_z:.3f}, geofence_r={self.geofence_radius_m:.1f}m, '
            f'require_state={self.require_state}, deadman_source={self.deadman_source}, '
            f'duration={duration_txt}'
        )

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _publish_stop(self):
        msg = Twist()
        self.pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        now_ros = self.get_clock().now()
        self.last_odom_ros = now_ros

        self.cur_pos_x = float(msg.pose.pose.position.x)
        self.cur_pos_y = float(msg.pose.pose.position.y)
        self.cur_omega = float(msg.twist.twist.angular.z)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cur_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_pos_x is not None and self.last_pos_y is not None:
            dx = self.cur_pos_x - self.last_pos_x
            dy = self.cur_pos_y - self.last_pos_y
            self.total_distance += math.hypot(dx, dy)

        self.last_pos_x = self.cur_pos_x
        self.last_pos_y = self.cur_pos_y

        self.samples += 1
        abs_omega = abs(self.cur_omega)
        self.max_abs_omega = max(self.max_abs_omega, abs_omega)
        if abs_omega > self.spin_threshold_rad_s:
            self.spin_samples += 1

    def _dynamic_tf_cb(self, msg: TFMessage):
        now_ros = self.get_clock().now()
        self.last_dynamic_tf_ros = now_ros

        selected = None
        token = self.dynamic_tf_child_contains
        for t in msg.transforms:
            if token and token in t.child_frame_id:
                selected = t
                break
        if selected is None and msg.transforms:
            selected = msg.transforms[0]
        if selected is None:
            return

        self.cur_pos_x = float(selected.transform.translation.x)
        self.cur_pos_y = float(selected.transform.translation.y)

        q = selected.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cur_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_pos_x is not None and self.last_pos_y is not None:
            dx = self.cur_pos_x - self.last_pos_x
            dy = self.cur_pos_y - self.last_pos_y
            self.total_distance += math.hypot(dx, dy)

        self.last_pos_x = self.cur_pos_x
        self.last_pos_y = self.cur_pos_y

    def _new_turn_target(self):
        self.target_angular_z = random.uniform(-self.turn_amplitude, self.turn_amplitude)
        self.target_angular_z = self._clamp(self.target_angular_z, -self.max_angular_z, self.max_angular_z)

    def _geofence_override(self):
        if not self.geofence_enabled:
            return False
        if self.cur_pos_x is None or self.cur_pos_y is None:
            return False

        dx = self.geofence_center_x - self.cur_pos_x
        dy = self.geofence_center_y - self.cur_pos_y
        dist = math.hypot(dx, dy)
        if dist <= self.geofence_radius_m:
            return False

        heading_to_center = math.atan2(dy, dx)
        err = math.atan2(math.sin(heading_to_center - self.cur_yaw), math.cos(heading_to_center - self.cur_yaw))

        self.target_linear_x = self._clamp(self.base_linear_x * 0.75, self.min_linear_x, self.max_linear_x)
        self.target_angular_z = self._clamp(1.5 * err, -self.max_angular_z, self.max_angular_z)
        return True

    def _tick(self):
        now_ros = self.get_clock().now()
        if self.start_ros is None:
            self.start_ros = now_ros
            self.last_tick_ros = now_ros
            self.last_turn_change_ros = now_ros
            self.last_log_ros = now_ros
            self._new_turn_target()

        dt = (now_ros - self.last_tick_ros).nanoseconds / 1e9
        dt = max(1e-3, dt)
        self.last_tick_ros = now_ros

        elapsed_s = (now_ros - self.start_ros).nanoseconds / 1e9
        keep_moving = (self.duration_s <= 0.0) or (elapsed_s <= self.duration_s)
        if not keep_moving:
            self._publish_stop()
            return

        # Deadman por estado: odom, dynamic_tf o cualquiera de ambos
        odom_ok = self.last_odom_ros is not None and (now_ros - self.last_odom_ros) <= Duration(seconds=self.state_timeout_s)
        tf_ok = self.last_dynamic_tf_ros is not None and (now_ros - self.last_dynamic_tf_ros) <= Duration(seconds=self.state_timeout_s)

        if self.deadman_source == 'odom':
            state_ok = odom_ok
        elif self.deadman_source == 'either':
            state_ok = odom_ok or tf_ok
        else:
            state_ok = tf_ok

        if self.require_state and not state_ok:
            self.cmd_linear_x = 0.0
            self.cmd_angular_z = 0.0
            self._publish_stop()
            if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
                self.last_log_ros = now_ros
                self.get_logger().warn('deadman active: no recent state, publishing stop')
            return

        # Actualizar target de giro cada cierto tiempo
        if ((now_ros - self.last_turn_change_ros).nanoseconds / 1e9) >= self.turn_update_period_s:
            self.last_turn_change_ros = now_ros
            self.target_linear_x = self._clamp(self.base_linear_x, self.min_linear_x, self.max_linear_x)
            self._new_turn_target()

        # Geocerca manda prioridad
        self._geofence_override()

        # Slew-rate limiting para evitar comandos bruscos
        lin_step = self.max_linear_accel * dt
        ang_step = self.max_angular_accel * dt

        if self.cmd_linear_x < self.target_linear_x:
            self.cmd_linear_x = min(self.cmd_linear_x + lin_step, self.target_linear_x)
        else:
            self.cmd_linear_x = max(self.cmd_linear_x - lin_step, self.target_linear_x)

        if self.cmd_angular_z < self.target_angular_z:
            self.cmd_angular_z = min(self.cmd_angular_z + ang_step, self.target_angular_z)
        else:
            self.cmd_angular_z = max(self.cmd_angular_z - ang_step, self.target_angular_z)

        self.cmd_linear_x = self._clamp(self.cmd_linear_x, self.min_linear_x, self.max_linear_x)
        self.cmd_angular_z = self._clamp(self.cmd_angular_z, -self.max_angular_z, self.max_angular_z)

        msg = Twist()
        msg.linear.x = self.cmd_linear_x
        msg.angular.z = self.cmd_angular_z
        self.pub.publish(msg)

        if ((now_ros - self.last_log_ros).nanoseconds / 1e9) >= self.log_period_s:
            self.last_log_ros = now_ros
            spin_pct = (100.0 * self.spin_samples / self.samples) if self.samples > 0 else 0.0
            self.get_logger().info(
                f'wander: v={self.cmd_linear_x:.3f}, w={self.cmd_angular_z:.3f}, '
                f'dist={self.total_distance:.2f}m, spin_pct={spin_pct:.1f}%, max|w_odom|={self.max_abs_omega:.3f}'
            )

    def print_summary(self):
        spin_pct = (100.0 * self.spin_samples / self.samples) if self.samples > 0 else 0.0
        self.get_logger().info(
            f'summary: distance={self.total_distance:.2f}m, samples={self.samples}, '
            f'spin_pct={spin_pct:.1f}%, max|w_odom|={self.max_abs_omega:.3f}'
        )



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.print_summary()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
