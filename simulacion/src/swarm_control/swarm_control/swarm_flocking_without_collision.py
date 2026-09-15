#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import subprocess
import time
import math
import numpy as np
import threading

# Main functional controller for swarm behavior
# Description: This class implements a simple flocking behavior for a swarm of robots.
# It uses basic steering behaviors to achieve separation, alignment, and cohesion.
# The update_flocking method is called at a regular interval to update the robot's velocity and position.
# It also handles communication with other robots in the swarm, allowing for coordinated movement
# (intended to emulate in a basic way the ESP-NOW communication for the real robots).

# Swarm behavior
class SwarmFlocking(Node):
    def __init__(self):
        super().__init__('swarm_flocking')

        # Swarm configuration
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 5)  # Number of robots in the swarm (needs to be changed manually)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_name = f'swarm_bot_{self.robot_id + 1}'

        # State of the robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # State of other robots
        self.neighbors = {}  # {robot_id: {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'last_seen': time}}

        # Flocking parameters
        self.separation_distance = 1.5  # Minimum distance between robots
        self.alignment_distance = 3.0   # Distance for alignment
        self.cohesion_distance = 4.0    # Distance for cohesion

        self.max_speed = 0.3
        self.max_turn_rate = 0.4

        # Publishers and subscribers
        self.odom_pub = self.create_publisher(
            Odometry, 
            f'/swarm/{self.robot_name}/odom', 
            10
        )

        # Subscribe to odometry of other robots
        for i in range(self.num_robots):
            if i != self.robot_id:
                other_robot = f'swarm_bot_{i + 1}'
                self.create_subscription(
                    Odometry,
                    f'/swarm/{other_robot}/odom',
                    lambda msg, robot_id=i: self.neighbor_callback(msg, robot_id),
                    10
                )
        
        # Timers
        self.update_timer = self.create_timer(0.1, self.update_flocking)  # 10Hz for calculations
        self.gazebo_timer = self.create_timer(0.2, self.update_gazebo)    # 5Hz for Gazebo
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)   # 10Hz for odometry

        # Initial position (circular formation)
        angle = (self.robot_id * 2 * math.pi) / self.num_robots
        self.x = 2.0 * math.cos(angle)
        self.y = 2.0 * math.sin(angle)
        self.theta = angle + math.pi/2  # Tangential orientation

        self.updating_gazebo = False
        self.start_time = time.time()
        
        self.get_logger().info(f'Robot {self.robot_name} initialized at ({self.x:.2f}, {self.y:.2f})')
    
    def neighbor_callback(self, msg, robot_id):
        """Receive information from neighboring robots"""
        self.neighbors[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'last_seen': time.time()
        }
    
    def update_flocking(self):
        """Update flocking behavior"""
        current_time = time.time()

        # Clean up old neighbors
        self.neighbors = {k: v for k, v in self.neighbors.items()
                         if current_time - v['last_seen'] < 1.0}
        
        if not self.neighbors:
            # No neighbors, exploratory movement
            t = current_time - self.start_time
            target_vx = 0.1 + 0.05 * math.sin(t * 0.2 + self.robot_id)
            target_vy = 0.05 * math.cos(t * 0.3 + self.robot_id)
        else:
            # Apply flocking rules
            sep_x, sep_y = self.separation()
            ali_x, ali_y = self.alignment()
            coh_x, coh_y = self.cohesion()

            # Combine forces with weights
            target_vx = 0.5 * sep_x + 0.3 * ali_x + 0.2 * coh_x
            target_vy = 0.5 * sep_y + 0.3 * ali_y + 0.2 * coh_y

        # Limit speed
        speed = math.sqrt(target_vx**2 + target_vy**2)
        if speed > self.max_speed:
            target_vx = target_vx * self.max_speed / speed
            target_vy = target_vy * self.max_speed / speed

        # Smooth velocities
        alpha = 0.2
        self.vx += alpha * (target_vx - self.vx)
        self.vy += alpha * (target_vy - self.vy)

        # Update position and orientation
        dt = 0.1
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        if abs(self.vx) > 0.01 or abs(self.vy) > 0.01:
            self.theta = math.atan2(self.vy, self.vx)
    
    def separation(self):
        """Rule 1: Separation - avoid collisions"""
        avoid_x, avoid_y = 0.0, 0.0
        count = 0
        
        for neighbor in self.neighbors.values():
            dx = self.x - neighbor['x']
            dy = self.y - neighbor['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.separation_distance and dist > 0:
                # Force inversely proportional to distance
                force = (self.separation_distance - dist) / self.separation_distance
                avoid_x += (dx / dist) * force
                avoid_y += (dy / dist) * force
                count += 1
        
        if count > 0:
            avoid_x /= count
            avoid_y /= count
        
        return avoid_x, avoid_y
    
    def alignment(self):
        """Rule 2: Alignment - move like neighbors"""
        avg_vx, avg_vy = 0.0, 0.0
        count = 0
        
        for neighbor in self.neighbors.values():
            dx = self.x - neighbor['x']
            dy = self.y - neighbor['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.alignment_distance:
                avg_vx += neighbor['vx']
                avg_vy += neighbor['vy']
                count += 1
        
        if count > 0:
            avg_vx /= count
            avg_vy /= count
            return avg_vx, avg_vy
        
        return 0.0, 0.0
    
    def cohesion(self):
        """Rule 3: Cohesion - stay close to the group"""
        center_x, center_y = 0.0, 0.0
        count = 0
        
        for neighbor in self.neighbors.values():
            dx = self.x - neighbor['x']
            dy = self.y - neighbor['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.cohesion_distance:
                center_x += neighbor['x']
                center_y += neighbor['y']
                count += 1
        
        if count > 0:
            center_x /= count
            center_y /= count

            # Move towards the center of the group
            dx = center_x - self.x
            dy = center_y - self.y
            return dx * 0.1, dy * 0.1  # Small factor for smooth movement

        return 0.0, 0.0
    
    def update_gazebo(self):
        """Update position in Gazebo (low frequency)"""
        if self.updating_gazebo:
            return
        
        self.updating_gazebo = True
        thread = threading.Thread(target=self._update_gazebo_async)
        thread.daemon = True
        thread.start()
    
    def _update_gazebo_async(self):
        """Update Gazebo asynchronously"""
        try:
            qz = math.sin(self.theta / 2.0)
            qw = math.cos(self.theta / 2.0)
            
            cmd = [
                'timeout', '0.3',
                'gz', 'service', '-s', '/world/flat_ocean/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '150',
                '--req',
                f'name: "{self.robot_name}" '
                f'position {{x: {self.x:.4f} y: {self.y:.4f} z: 0.0}} '
                f'orientation {{z: {qz:.4f} w: {qw:.4f}}}'
            ]
            
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.3)
            
        except:
            pass
        finally:
            self.updating_gazebo = False
    
    def publish_odometry(self):
        """Publish odometry for other robots"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = f'{self.robot_name}/base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmFlocking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()