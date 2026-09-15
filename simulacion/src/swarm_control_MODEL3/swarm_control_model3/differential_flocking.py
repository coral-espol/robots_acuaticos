#!/usr/bin/env python3
import math
import subprocess
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DifferentialFlocking(Node):
    def __init__(self):
        super().__init__('differential_flocking')

        # Configuración del swarm
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 5)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_name = f'robot{self.robot_id + 1}'

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.z = 0.15  # Ajuste para modelo 3 (base inferior + espuma)
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # Estado de vecinos
        self.neighbors = {}

        # Parámetros de flocking
        self.separation_distance = 1.5
        self.neighbor_distance = 3.0
        self.separation_gain = 1.5
        self.alignment_gain = 0.8
        self.cohesion_gain = 0.5

        # Parámetros del robot diferencial (modelo 3)
        self.wheel_base = 0.316  # Distancia aprox entre motores izquierdo y derecho
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0
        self.max_wheel_vel = 2.0

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.robot_name}/motor_controller/commands',
            10,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{self.robot_name}/odometry',
            10,
        )

        # Suscripciones a vecinos
        for i in range(self.num_robots):
            if i != self.robot_id:
                self.create_subscription(
                    Odometry,
                    f'/robot{i+1}/odometry',
                    lambda msg, rid=i: self.neighbor_callback(msg, rid),
                    10,
                )

        # Timers
        self.update_timer = self.create_timer(0.1, self.update_flocking)
        self.gazebo_timer = self.create_timer(0.05, self.update_gazebo)
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)

        # Setup de posiciones iniciales
        initial_positions = [
            (3.0, 0.5, 0.0),
            (-2.0, 3.0, math.pi / 2),
            (1.0, -2.5, -math.pi / 4),
            (-1.5, -1.0, math.pi),
            (0.0, 2.0, math.pi / 3),
        ]

        if self.robot_id < len(initial_positions):
            self.x, self.y, self.theta = initial_positions[self.robot_id]
        else:
            angle = (self.robot_id * 2 * math.pi) / self.num_robots
            self.x = 2.0 * math.cos(angle)
            self.y = 2.0 * math.sin(angle)
            self.theta = angle + math.pi / 2

        self.updating_gazebo = False
        self.start_time = time.time()

        self.get_logger().info(
            f'Robot {self.robot_name} (MODEL3) inicializado en ({self.x:.2f}, {self.y:.2f})'
        )

    def neighbor_callback(self, msg, robot_id):
        self.neighbors[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'last_seen': time.time(),
        }

    def separation(self):
        sep_x, sep_y = 0.0, 0.0

        for neighbor in self.neighbors.values():
            dx = self.x - neighbor['x']
            dy = self.y - neighbor['y']
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.separation_distance and distance > 0.01:
                force = (self.separation_distance - distance) / self.separation_distance
                sep_x += (dx / distance) * force
                sep_y += (dy / distance) * force

        return sep_x * self.separation_gain, sep_y * self.separation_gain

    def alignment(self):
        if not self.neighbors:
            return 0.0, 0.0

        avg_vx = sum(n['vx'] for n in self.neighbors.values()) / len(self.neighbors)
        avg_vy = sum(n['vy'] for n in self.neighbors.values()) / len(self.neighbors)

        ali_x = (avg_vx - self.vx) * self.alignment_gain
        ali_y = (avg_vy - self.vy) * self.alignment_gain

        return ali_x, ali_y

    def cohesion(self):
        if not self.neighbors:
            return 0.0, 0.0

        center_x = sum(n['x'] for n in self.neighbors.values()) / len(self.neighbors)
        center_y = sum(n['y'] for n in self.neighbors.values()) / len(self.neighbors)

        coh_x = (center_x - self.x) * self.cohesion_gain
        coh_y = (center_y - self.y) * self.cohesion_gain

        return coh_x, coh_y

    def differential_kinematics(self, vx_global, vy_global, omega):
        vx_local = vx_global * math.cos(self.theta) + vy_global * math.sin(self.theta)
        vy_local = -vx_global * math.sin(self.theta) + vy_global * math.cos(self.theta)

        if abs(vy_local) > 0.01:
            omega += vy_local * 2.0

        vx_local = np.clip(vx_local, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)

        v_left = vx_local - (omega * self.wheel_base / 2.0)
        v_right = vx_local + (omega * self.wheel_base / 2.0)

        v_left = np.clip(v_left, -self.max_wheel_vel, self.max_wheel_vel)
        v_right = np.clip(v_right, -self.max_wheel_vel, self.max_wheel_vel)

        return v_left, v_right

    def update_flocking(self):
        current_time = time.time()

        self.neighbors = {
            k: v for k, v in self.neighbors.items()
            if current_time - v['last_seen'] < 1.0
        }

        if not self.neighbors:
            t = current_time - self.start_time
            target_vx = 0.1 + 0.05 * math.sin(t * 0.2 + self.robot_id)
            target_vy = 0.05 * math.cos(t * 0.3 + self.robot_id)
            target_omega = 0.1 * math.sin(t * 0.1)
        else:
            sep_x, sep_y = self.separation()
            ali_x, ali_y = self.alignment()
            coh_x, coh_y = self.cohesion()

            target_vx = sep_x + ali_x + coh_x
            target_vy = sep_y + ali_y + coh_y

            desired_theta = math.atan2(target_vy, target_vx)
            theta_error = math.atan2(
                math.sin(desired_theta - self.theta),
                math.cos(desired_theta - self.theta),
            )
            target_omega = theta_error * 1.5

        alpha = 0.3
        self.vx = alpha * target_vx + (1 - alpha) * self.vx
        self.vy = alpha * target_vy + (1 - alpha) * self.vy
        self.omega = alpha * target_omega + (1 - alpha) * self.omega

        v_left, v_right = self.differential_kinematics(self.vx, self.vy, self.omega)

        cmd = Float64MultiArray()
        cmd.data = [v_left, v_right]
        self.joint_cmd_pub.publish(cmd)

        dt = 0.1
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def update_gazebo(self):
        if self.updating_gazebo:
            return

        threading.Thread(target=self._update_gazebo_async, daemon=True).start()

    def _update_gazebo_async(self):
        self.updating_gazebo = True
        try:
            cmd = [
                'gz', 'service', '-s', '/world/flat_ocean/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '100',
                '--req',
                f'name: "{self.robot_name}", '
                f'position: {{x: {self.x}, y: {self.y}, z: {self.z}}}, '
                f'orientation: {{z: {math.sin(self.theta/2)}, w: {math.cos(self.theta/2)}}}',
            ]
            subprocess.run(cmd, capture_output=True, timeout=0.5)
        except Exception:
            pass
        finally:
            self.updating_gazebo = False

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = f'{self.robot_name}/base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialFlocking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
