#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelStraight(Node):
    """Publica cmd_vel suave y estable para prueba base en línea recta."""

    def __init__(self):
        super().__init__('cmd_vel_straight')

        self.declare_parameter('cmd_topic', '/model/robot1/cmd_vel')
        self.declare_parameter('linear_x', 0.03)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('zigzag_angular_z', 0.0)
        self.declare_parameter('zigzag_period_s', 12.0)
        self.declare_parameter('duration_s', 0.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.angular_z = float(self.get_parameter('angular_z').value)
        self.zigzag_angular_z = float(self.get_parameter('zigzag_angular_z').value)
        self.zigzag_period_s = max(0.1, float(self.get_parameter('zigzag_period_s').value))
        self.duration_s = float(self.get_parameter('duration_s').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.start_time_ros = None
        self.last_log_ros = None

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self._tick)

        duration_txt = 'infinite' if self.duration_s <= 0.0 else f'{self.duration_s:.1f}s'
        self.get_logger().info(
            f'cmd_vel straight started: topic={self.cmd_topic}, vx={self.linear_x:.3f}, '
            f'wz={self.angular_z:.3f}, zigzag_wz={self.zigzag_angular_z:.3f}, '
            f'zigzag_period={self.zigzag_period_s:.1f}s, duration={duration_txt}'
        )

    def _tick(self):
        now_ros = self.get_clock().now()
        if self.start_time_ros is None:
            self.start_time_ros = now_ros
            self.last_log_ros = now_ros

        elapsed_s = (now_ros - self.start_time_ros).nanoseconds / 1e9

        msg = Twist()
        keep_moving = (self.duration_s <= 0.0) or (elapsed_s <= self.duration_s)
        if keep_moving:
            msg.linear.x = self.linear_x
            zigzag = self.zigzag_angular_z * math.sin((2.0 * math.pi / self.zigzag_period_s) * elapsed_s)
            msg.angular.z = self.angular_z + zigzag
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)

        if ((now_ros - self.last_log_ros).nanoseconds / 1e9) > 2.0:
            self.last_log_ros = now_ros
            self.get_logger().info(
                f'publishing cmd_vel: vx={msg.linear.x:.3f}, wz={msg.angular.z:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStraight()
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
