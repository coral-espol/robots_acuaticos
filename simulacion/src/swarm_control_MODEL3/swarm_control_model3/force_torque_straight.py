#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_msgs.msg import TFMessage


class ForceTorqueStraight(Node):
    """Very simple baseline controller: smooth straight motion with depth hold.

    Goals:
    - Validate stable physical behavior first.
    - Keep movement gentle and mostly linear.
    - Avoid spin with pure angular damping torques.
    - Keep depth near a target z with vertical force control.
    """

    def __init__(self):
        super().__init__('force_torque_straight')

        self.declare_parameter('world_name', 'shrimp_pond_static')
        self.declare_parameter('model_name', 'robot1')

        self.declare_parameter('desired_speed', 0.01)
        self.declare_parameter('max_speed', 0.03)
        self.declare_parameter('speed_kp', 0.3)
        self.declare_parameter('base_force', 0.003)
        self.declare_parameter('max_force', 0.02)

        self.declare_parameter('lin_drag_k', 6.0)
        self.declare_parameter('lateral_drag_k', 3.0)
        self.declare_parameter('force_slew_rate', 0.002)
        self.declare_parameter('depth_target_z', -0.02)
        self.declare_parameter('depth_kp', 18.0)
        self.declare_parameter('depth_kd', 12.0)
        self.declare_parameter('max_force_z', 15.0)

        self.declare_parameter('ang_damp_xy', 6.0)
        self.declare_parameter('ang_damp_z', 4.0)
        self.declare_parameter('max_torque', 0.8)

        self.declare_parameter('arena_half_x', 20.0)
        self.declare_parameter('arena_half_y', 10.0)
        self.declare_parameter('boundary_k', 0.25)

        self.declare_parameter('target_is_model', True)
        self.declare_parameter('target_link_name', 'base_link')
        self.declare_parameter('open_loop_mode', True)
        self.declare_parameter('open_loop_fx', 0.00025)
        self.declare_parameter('open_loop_duration', 8.0)
        self.declare_parameter('force_zero_mode', False)

        self.world_name = str(self.get_parameter('world_name').value)
        self.model_name = str(self.get_parameter('model_name').value)

        self.desired_speed = float(self.get_parameter('desired_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.speed_kp = float(self.get_parameter('speed_kp').value)
        self.base_force = float(self.get_parameter('base_force').value)
        self.max_force = float(self.get_parameter('max_force').value)

        self.lin_drag_k = float(self.get_parameter('lin_drag_k').value)
        self.lateral_drag_k = float(self.get_parameter('lateral_drag_k').value)
        self.force_slew_rate = float(self.get_parameter('force_slew_rate').value)
        self.depth_target_z = float(self.get_parameter('depth_target_z').value)
        self.depth_kp = float(self.get_parameter('depth_kp').value)
        self.depth_kd = float(self.get_parameter('depth_kd').value)
        self.max_force_z = float(self.get_parameter('max_force_z').value)
        self.ang_damp_xy = float(self.get_parameter('ang_damp_xy').value)
        self.ang_damp_z = float(self.get_parameter('ang_damp_z').value)
        self.max_torque = float(self.get_parameter('max_torque').value)

        self.arena_half_x = float(self.get_parameter('arena_half_x').value)
        self.arena_half_y = float(self.get_parameter('arena_half_y').value)
        self.boundary_k = float(self.get_parameter('boundary_k').value)

        self.target_is_model = bool(self.get_parameter('target_is_model').value)
        self.target_link_name = str(self.get_parameter('target_link_name').value)
        self.open_loop_mode = bool(self.get_parameter('open_loop_mode').value)
        self.open_loop_fx = float(self.get_parameter('open_loop_fx').value)
        self.open_loop_duration = float(self.get_parameter('open_loop_duration').value)
        self.force_zero_mode = bool(self.get_parameter('force_zero_mode').value)

        self.wrench_pub = self.create_publisher(
            EntityWrench,
            f'/world/{self.world_name}/wrench',
            10,
        )

        self.have_odom = False
        self._odom_announced = False
        self._tf_pose_announced = False
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.filtered_fx = 0.0
        self.filtered_fy = 0.0
        self.filtered_fz = 0.0
        self.filtered_tx = 0.0
        self.filtered_ty = 0.0
        self.filtered_tz = 0.0
        self.last_log_time = time.time()
        self.start_time = time.time()
        self._last_tf_time = None
        self._last_tf_x = 0.0
        self._last_tf_y = 0.0
        self._last_tf_z = 0.0
        self._last_tf_yaw = 0.0

        self.create_subscription(
            Odometry,
            f'/{self.model_name}/odometry',
            self._odom_cb,
            10,
        )
        # Raw Gazebo odometry topic fallback (in case remapping is not active)
        self.create_subscription(
            Odometry,
            f'/model/{self.model_name}/odometry',
            self._odom_cb,
            10,
        )
        self.create_subscription(
            TFMessage,
            '/robot1/dynamic_tf',
            self._dynamic_tf_cb,
            10,
        )
        self.create_subscription(
            TFMessage,
            f'/world/{self.world_name}/dynamic_pose/info',
            self._dynamic_tf_cb,
            10,
        )

        self.create_timer(0.1, self.update_control)

        self.get_logger().info(
            f'Straight controller started for {self.model_name} in {self.world_name} '
            f'(target={"MODEL" if self.target_is_model else "LINK"})'
        )
        self.get_logger().info(
            f'open_loop_mode={self.open_loop_mode}, open_loop_fx={self.open_loop_fx:.6f}, '
            f'open_loop_duration={self.open_loop_duration:.1f}s'
        )
        self.get_logger().info(f'force_zero_mode={self.force_zero_mode}')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z
        self.have_odom = True
        if not self._odom_announced:
            self._odom_announced = True
            self.get_logger().info('Odometry received; controller active')

    def _publish_wrench(self, fx: float, fy: float, fz: float, tx: float, ty: float, tz: float):
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
        ew.wrench.force.z = float(fz)
        ew.wrench.torque.x = float(tx)
        ew.wrench.torque.y = float(ty)
        ew.wrench.torque.z = float(tz)

        self.wrench_pub.publish(ew)

    def _dynamic_tf_cb(self, msg: TFMessage):
        for t in msg.transforms:
            child = t.child_frame_id
            parent = t.header.frame_id
            if (self.model_name not in child) and (self.model_name not in parent):
                continue
            if ('base_link' not in child) and (self.model_name not in child) and (self.model_name not in parent):
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            q = t.transform.rotation

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            now = time.time()
            if self._last_tf_time is not None:
                dt = max(1e-3, now - self._last_tf_time)
                self.vx = (x - self._last_tf_x) / dt
                self.vy = (y - self._last_tf_y) / dt
                self.vz = (z - self._last_tf_z) / dt
                dyaw = math.atan2(math.sin(yaw - self._last_tf_yaw), math.cos(yaw - self._last_tf_yaw))
                self.wz = dyaw / dt
                self.wx = 0.0
                self.wy = 0.0

            self.x = x
            self.y = y
            self.z = z
            self.yaw = yaw

            self._last_tf_x = x
            self._last_tf_y = y
            self._last_tf_z = z
            self._last_tf_yaw = yaw
            self._last_tf_time = now
            self.have_odom = True

            if not self._tf_pose_announced:
                self._tf_pose_announced = True
                self.get_logger().info('Dynamic TF pose received; fallback state estimator active')
            break

    def update_control(self):
        if self.force_zero_mode:
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.last_log_time = now
                self.get_logger().warn('force_zero_mode active: not publishing wrench')
            return

        if not self.have_odom:
            # Baseline fallback: open-loop tiny straight push for a short time.
            elapsed = time.time() - self.start_time
            if self.open_loop_mode and elapsed < self.open_loop_duration:
                fx = max(-self.max_force, min(self.max_force, self.open_loop_fx))
                self._publish_wrench(fx, 0.0, 0.0, 0.0, 0.0, 0.0)
            else:
                self._publish_wrench(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.last_log_time = now
                self.get_logger().warn('No odometry yet; running open-loop fallback')
            return

        # Body-frame velocities
        v_forward = self.vx * math.cos(self.yaw) + self.vy * math.sin(self.yaw)
        v_lateral = -self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)

        # Forward-only speed regulation
        speed_error = self.desired_speed - v_forward
        cmd_force = self.base_force + self.speed_kp * speed_error
        cmd_force = max(0.0, min(self.max_force, cmd_force))

        thrust_fx = cmd_force * math.cos(self.yaw)
        thrust_fy = cmd_force * math.sin(self.yaw)

        # World linear drag
        drag_fx = -self.lin_drag_k * self.vx
        drag_fy = -self.lin_drag_k * self.vy

        # Lateral damping projected to world
        lat_damp = -self.lateral_drag_k * v_lateral
        lat_fx = lat_damp * (-math.sin(self.yaw))
        lat_fy = lat_damp * (math.cos(self.yaw))

        raw_fx = thrust_fx + drag_fx + lat_fx
        raw_fy = thrust_fy + drag_fy + lat_fy

        # Depth hold (z axis): keep close to surface plane without sinking.
        depth_error = self.depth_target_z - self.z
        raw_fz = self.depth_kp * depth_error - self.depth_kd * self.vz
        raw_fz = max(-self.max_force_z, min(self.max_force_z, raw_fz))

        # Soft geofence brake toward center
        over_x = max(0.0, abs(self.x) - self.arena_half_x)
        over_y = max(0.0, abs(self.y) - self.arena_half_y)
        if over_x > 0.0:
            raw_fx += -self.boundary_k * over_x * math.copysign(1.0, self.x)
        if over_y > 0.0:
            raw_fy += -self.boundary_k * over_y * math.copysign(1.0, self.y)

        # Safety if speed spikes
        speed = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        if speed > self.max_speed:
            raw_fx = -0.7 * self.max_force * math.tanh(2.0 * self.vx)
            raw_fy = -0.7 * self.max_force * math.tanh(2.0 * self.vy)
            raw_fz = max(-self.max_force_z, min(self.max_force_z, -0.6 * self.vz))

        # Keep torques ultra-restrictive: only tiny yaw damping.
        raw_tx = 0.0
        raw_ty = 0.0
        raw_tz = max(-self.max_torque, min(self.max_torque, -self.ang_damp_z * self.wz))

        raw_fx = max(-self.max_force, min(self.max_force, raw_fx))
        raw_fy = max(-self.max_force, min(self.max_force, raw_fy))
        raw_fz = max(-self.max_force_z, min(self.max_force_z, raw_fz))

        # Smooth via slew-rate
        self.filtered_fx += max(-self.force_slew_rate, min(self.force_slew_rate, raw_fx - self.filtered_fx))
        self.filtered_fy += max(-self.force_slew_rate, min(self.force_slew_rate, raw_fy - self.filtered_fy))
        self.filtered_fz += max(-0.08, min(0.08, raw_fz - self.filtered_fz))
        self.filtered_tx += max(-0.01, min(0.01, raw_tx - self.filtered_tx))
        self.filtered_ty += max(-0.01, min(0.01, raw_ty - self.filtered_ty))
        self.filtered_tz += max(-0.01, min(0.01, raw_tz - self.filtered_tz))

        self._publish_wrench(
            self.filtered_fx,
            self.filtered_fy,
            self.filtered_fz,
            self.filtered_tx,
            self.filtered_ty,
            self.filtered_tz,
        )

        now = time.time()
        if now - self.last_log_time > 2.0:
            self.last_log_time = now
            self.get_logger().info(
                f'cmd -> Fx:{self.filtered_fx:.4f} Fy:{self.filtered_fy:.4f} Fz:{self.filtered_fz:.3f} '
                f'Tx:{self.filtered_tx:.3f} Ty:{self.filtered_ty:.3f} Tz:{self.filtered_tz:.3f} z:{self.z:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueStraight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
