#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench


class ForceTorqueWander(Node):
    """Minimal force/torque controller with a very smooth simple wander.

    - Single robot: gentle forward speed hold + tiny sinusoidal yaw torque.
    - Multi-robot fallback: lightweight flocking-like bias toward neighbor center.
    - Commands Gazebo via /world/<world>/wrench using EntityWrench.
    """

    def __init__(self):
        super().__init__('force_torque_wander')

        self.declare_parameter('world_name', 'shrimp_pond_static')
        self.declare_parameter('model_name', 'robot1')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 1)

        # Defaults are fallback values; launch file can override them.
        self.declare_parameter('base_force', 0.15)
        self.declare_parameter('max_force', 0.40)
        self.declare_parameter('max_torque', 0.03)
        self.declare_parameter('speed_kp', 1.0)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('lin_drag_k', 3.2)
        self.declare_parameter('ang_drag_k', 1.8)
        self.declare_parameter('heading_kp', 0.12)
        self.declare_parameter('heading_kd', 0.20)
        self.declare_parameter('force_slew_rate', 0.04)
        self.declare_parameter('torque_slew_rate', 0.012)
        self.declare_parameter('desired_speed', 0.03)
        self.declare_parameter('turn_period', 40.0)
        self.declare_parameter('turn_amplitude', 0.0025)
        self.declare_parameter('lateral_drag_k', 1.5)
        self.declare_parameter('pure_damping_mode', True)
        self.declare_parameter('arena_half_x', 25.0)
        self.declare_parameter('arena_half_y', 12.0)
        self.declare_parameter('boundary_k', 0.03)
        self.declare_parameter('crash_speed', 0.12)
        self.declare_parameter('crash_yaw_rate', 0.30)
        self.declare_parameter('target_is_model', False)
        self.declare_parameter('target_link_name', 'base_link')

        self.world_name = self.get_parameter('world_name').value
        self.model_name = self.get_parameter('model_name').value
        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value

        self.base_force = float(self.get_parameter('base_force').value)
        self.max_force = float(self.get_parameter('max_force').value)
        self.max_torque = float(self.get_parameter('max_torque').value)
        self.speed_kp = float(self.get_parameter('speed_kp').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.lin_drag_k = float(self.get_parameter('lin_drag_k').value)
        self.ang_drag_k = float(self.get_parameter('ang_drag_k').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.force_slew_rate = float(self.get_parameter('force_slew_rate').value)
        self.torque_slew_rate = float(self.get_parameter('torque_slew_rate').value)
        self.desired_speed = float(self.get_parameter('desired_speed').value)
        self.turn_period = float(self.get_parameter('turn_period').value)
        self.turn_amplitude = float(self.get_parameter('turn_amplitude').value)
        self.lateral_drag_k = float(self.get_parameter('lateral_drag_k').value)
        self.pure_damping_mode = bool(self.get_parameter('pure_damping_mode').value)
        self.arena_half_x = float(self.get_parameter('arena_half_x').value)
        self.arena_half_y = float(self.get_parameter('arena_half_y').value)
        self.boundary_k = float(self.get_parameter('boundary_k').value)
        self.crash_speed = float(self.get_parameter('crash_speed').value)
        self.crash_yaw_rate = float(self.get_parameter('crash_yaw_rate').value)
        self.target_is_model = bool(self.get_parameter('target_is_model').value)
        self.target_link_name = str(self.get_parameter('target_link_name').value)

        self.wrench_pub = self.create_publisher(
            EntityWrench,
            f'/world/{self.world_name}/wrench',
            10,
        )

        self.neighbors = {}
        self.have_odom = False
        self.yaw = 0.0
        self.current_yaw_rate = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_speed = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.create_subscription(
            Odometry,
            f'/{self.model_name}/odometry',
            self._self_odom_cb,
            10,
        )

        for i in range(self.num_robots):
            if i == self.robot_id:
                continue
            self.create_subscription(
                Odometry,
                f'/robot{i + 1}/odometry',
                lambda msg, rid=i: self._neighbor_cb(msg, rid),
                10,
            )

        self.force_cmd = self.base_force
        self.torque_cmd = 0.0
        self.target_speed = self.desired_speed
        self.target_heading = 0.0
        self.heading_initialized = False
        self.filtered_fx = 0.0
        self.filtered_fy = 0.0
        self.filtered_tz = 0.0
        self.last_log_time = time.time()

        self.update_timer = self.create_timer(0.1, self.update_control)

        self.get_logger().info(
            f'Force/Torque wander started for {self.model_name} in world {self.world_name} '
            f'(target={"MODEL" if self.target_is_model else "LINK"})'
        )
        self.get_logger().info(
            'Loaded params: '
            f'base_force={self.base_force:.6f}, max_force={self.max_force:.6f}, '
            f'max_torque={self.max_torque:.6f}, desired_speed={self.desired_speed:.6f}, '
            f'max_speed={self.max_speed:.6f}, lin_drag_k={self.lin_drag_k:.3f}, '
            f'ang_drag_k={self.ang_drag_k:.3f}, pure_damping_mode={self.pure_damping_mode}'
        )

    def _neighbor_cb(self, msg, rid):
        self.neighbors[rid] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'last_seen': time.time(),
        }

    def _self_odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw_rate = msg.twist.twist.angular.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_vx = vx
        self.current_vy = vy
        self.current_speed = math.sqrt(vx * vx + vy * vy)
        if not self.heading_initialized:
            self.target_heading = self.yaw
            self.heading_initialized = True
        self.have_odom = True

    def _publish_wrench(self, fx, fy, tz):
        # Model target is robust to URDF root-link hierarchy changes.
        ew = EntityWrench()
        ew.header.frame_id = 'world'
        if self.target_is_model:
            ew.entity.name = self.model_name
            ew.entity.type = Entity.MODEL
        else:
            ew.entity.name = f'{self.model_name}::{self.target_link_name}'
            ew.entity.type = Entity.LINK

        ew.wrench = Wrench()
        ew.wrench.force.x = float(fx)
        ew.wrench.force.y = float(fy)
        ew.wrench.force.z = 0.0
        ew.wrench.torque.x = 0.0
        ew.wrench.torque.y = 0.0
        ew.wrench.torque.z = float(tz)

        self.wrench_pub.publish(ew)

    def _random_motion(self, now):
        # Very simple smooth controller:
        # - tiny forward speed hold in body frame
        # - continuous sinusoidal yaw torque (no random jumps)
        # - strong damping to suppress abrupt motion and spin

        # Body-frame velocities
        v_forward = self.current_vx * math.cos(self.yaw) + self.current_vy * math.sin(self.yaw)
        v_lateral = -self.current_vx * math.sin(self.yaw) + self.current_vy * math.cos(self.yaw)

        # Forward speed hold (gentle)
        speed_error = self.desired_speed - v_forward
        self.force_cmd = self.base_force + self.speed_kp * speed_error
        self.force_cmd = max(0.0, min(self.max_force, self.force_cmd))

        thrust_fx = self.force_cmd * math.cos(self.yaw)
        thrust_fy = self.force_cmd * math.sin(self.yaw)

        # Lateral damping projected to world (reduces side-slip)
        lat_damp = -self.lateral_drag_k * v_lateral
        lat_fx = lat_damp * (-math.sin(self.yaw))
        lat_fy = lat_damp * (math.cos(self.yaw))

        # World-frame linear drag
        drag_fx = -self.lin_drag_k * self.current_vx
        drag_fy = -self.lin_drag_k * self.current_vy

        raw_fx = thrust_fx + lat_fx + drag_fx
        raw_fy = thrust_fy + lat_fy + drag_fy

        # Soft geofence toward center to avoid hitting borders.
        over_x = max(0.0, abs(self.current_x) - self.arena_half_x)
        over_y = max(0.0, abs(self.current_y) - self.arena_half_y)
        if over_x > 0.0:
            raw_fx += -self.boundary_k * over_x * math.copysign(1.0, self.current_x)
        if over_y > 0.0:
            raw_fy += -self.boundary_k * over_y * math.copysign(1.0, self.current_y)

        # Continuous tiny turning profile + angular damping
        smooth_turn = self.turn_amplitude * math.sin((2.0 * math.pi * now) / max(1.0, self.turn_period))
        if self.pure_damping_mode:
            raw_tz = -self.ang_drag_k * self.current_yaw_rate
        else:
            raw_tz = smooth_turn - self.heading_kd * self.current_yaw_rate - self.ang_drag_k * self.current_yaw_rate

        if self.max_torque <= 0.0:
            raw_tz = 0.0
        raw_tz = max(-self.max_torque, min(self.max_torque, raw_tz))

        # Safety clamps
        if self.current_speed > (1.1 * self.max_speed):
            raw_fx *= 0.30
            raw_fy *= 0.30
            raw_tz *= 0.40

        if abs(self.current_yaw_rate) > 0.15:
            raw_fx *= 0.25
            raw_fy *= 0.25
            raw_tz = -0.40 * self.max_torque * math.copysign(1.0, self.current_yaw_rate)

        # Hard safety brake against numeric blowups / runaway.
        if (
            (not math.isfinite(raw_fx))
            or (not math.isfinite(raw_fy))
            or (not math.isfinite(raw_tz))
            or self.current_speed > self.crash_speed
            or abs(self.current_yaw_rate) > self.crash_yaw_rate
        ):
            raw_fx = -0.8 * self.max_force * math.tanh(2.0 * self.current_vx)
            raw_fy = -0.8 * self.max_force * math.tanh(2.0 * self.current_vy)
            raw_tz = -0.8 * self.max_torque * math.tanh(3.0 * self.current_yaw_rate)

        raw_fx = max(-self.max_force, min(self.max_force, raw_fx))
        raw_fy = max(-self.max_force, min(self.max_force, raw_fy))
        raw_tz = max(-self.max_torque, min(self.max_torque, raw_tz))

        # Low-pass filtering for smooth commands
        alpha = 0.08
        target_fx = (1.0 - alpha) * self.filtered_fx + alpha * raw_fx
        target_fy = (1.0 - alpha) * self.filtered_fy + alpha * raw_fy
        target_tz = (1.0 - alpha) * self.filtered_tz + alpha * raw_tz

        # Slew-rate limit to avoid abrupt command jumps
        self.filtered_fx += max(-self.force_slew_rate, min(self.force_slew_rate, target_fx - self.filtered_fx))
        self.filtered_fy += max(-self.force_slew_rate, min(self.force_slew_rate, target_fy - self.filtered_fy))
        self.filtered_tz += max(-self.torque_slew_rate, min(self.torque_slew_rate, target_tz - self.filtered_tz))

        return self.filtered_fx, self.filtered_fy, self.filtered_tz

    def _flocking_like_motion(self):
        # Minimal placeholder: when neighbors appear, bias toward group center
        valid_neighbors = {
            k: v
            for k, v in self.neighbors.items()
            if time.time() - v['last_seen'] < 1.0
        }

        if not valid_neighbors:
            return self._random_motion(time.time())

        center_x = sum(n['x'] for n in valid_neighbors.values()) / len(valid_neighbors)
        center_y = sum(n['y'] for n in valid_neighbors.values()) / len(valid_neighbors)

        desired_angle = math.atan2(center_y, center_x)
        fx = self.base_force * math.cos(desired_angle)
        fy = self.base_force * math.sin(desired_angle)

        # Gentle angular correction
        tz = max(-self.max_torque * 0.6, min(self.max_torque * 0.6, desired_angle * 0.4))

        return fx, fy, tz

    def update_control(self):
        if not self.have_odom:
            return

        fx, fy, tz = self._flocking_like_motion()

        fx = max(-self.max_force, min(self.max_force, fx))
        fy = max(-self.max_force, min(self.max_force, fy))
        tz = max(-self.max_torque, min(self.max_torque, tz))

        self._publish_wrench(fx, fy, tz)

        now = time.time()
        if now - self.last_log_time > 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f'cmd wrench -> Fx:{fx:.2f} Fy:{fy:.2f} Tz:{tz:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
