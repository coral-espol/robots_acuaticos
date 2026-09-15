#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import subprocess
import time
import math
import numpy as np
import threading

# Main functional controller for swarm behavior (with obstacle avoidance)
# Description: This class implements a simple flocking behavior for a swarm of robots.
# Laser based Controller version for swarm flocking (simple behavior among robots and obstacle avoidance)
# It's used for simulating and controlling a swarm of robots in a 3D aquatic environment on Gazebo.
# The controller uses laser scans for obstacle detection and avoidance.
# It implements a simple flocking behavior based on three rules: separation, alignment, and cohesion.
# Separation: Avoid crowding neighbors (steer away from nearby robots)
# Alignment: Move towards the average heading of neighbors
# Cohesion: Move towards the average position of neighbors
# This behavior is implemented in the update_flocking method, which is called at a regular interval from the swarm_behavior launch file.

# (NOTE: This is a simplified simulation to test Gazebo Harmonic's performance and may not fully represent real-world behavior.
# The actual implementation may require more complex algorithms and communication protocols for real robots.
# It has some problems with the laser scan plugin from Gazebo due to libraries issues)

# Swarm behavior
class SwarmFlocking(Node):
    def __init__(self):
        super().__init__('swarm_flocking')
        
        # Swarm configuration
        self.declare_parameter('robot_id', 0) 
        self.declare_parameter('num_robots', 5) # Number of robots in the swarm (needs to be changed manually)

        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        self.robot_name = f'swarm_bot_{self.robot_id + 1}' # Unique ID for each robot

        # State of this robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # State of other robots
        self.neighbors = {}  # {robot_id: {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'last_seen': time}}

        # Flocking parameters
        self.separation_distance = 1.5  # Minimum distance between robots
        self.alignment_distance = 3.0   # Alignment distance
        self.cohesion_distance = 4.0    # Cohesion distance

        self.max_speed = 0.3
        self.max_turn_rate = 0.4

        # State for obstacle avoidance
        self.obstacle_ranges = []
        self.obstacle_angles = []
        self.min_obstacle_distance = float('inf')

        # Evasion parameters
        self.obstacle_avoidance_distance = 1.0  # Distance to trigger avoidance
        self.obstacle_force_gain = 0.8           # Strength of the avoidance force

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

        # Subscribe to laser scan
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/swarm/laser_scan',
            self.laser_callback,
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

    # Laser callback
    def laser_callback(self, msg):
        """Process laser sensor data"""
        self.obstacle_ranges = list(msg.ranges)
        self.obstacle_angles = [msg.angle_min + i * msg.angle_increment 
                               for i in range(len(msg.ranges))]

        # Find closest obstacle
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        self.min_obstacle_distance = min(valid_ranges) if valid_ranges else float('inf')

    # Obstacle avoidance function
    def obstacle_avoidance(self):
        """Rule 4: Obstacle avoidance"""
        avoid_x, avoid_y = 0.0, 0.0
        
        if self.min_obstacle_distance < self.obstacle_avoidance_distance:
            for i, (distance, angle) in enumerate(zip(self.obstacle_ranges, self.obstacle_angles)):
                if not math.isinf(distance) and not math.isnan(distance):
                    if distance < self.obstacle_avoidance_distance:
                        # Inversely proportional repulsion force
                        force = (self.obstacle_avoidance_distance - distance) / self.obstacle_avoidance_distance

                        # Global angle of the obstacle
                        global_angle = self.theta + angle

                        # Evasion force (opposite to the obstacle)
                        avoid_x -= math.cos(global_angle) * force * self.obstacle_force_gain
                        avoid_y -= math.sin(global_angle) * force * self.obstacle_force_gain
        
        return avoid_x, avoid_y
    
    def update_flocking(self):
        """Actualizar comportamiento de flocking CON evasión"""
        current_time = time.time()

        # Clear old neighbors
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
            target_vx = 0.4 * sep_x + 0.25 * ali_x + 0.15 * coh_x
            target_vy = 0.4 * sep_y + 0.25 * ali_y + 0.15 * coh_y

        # Obstacle avoidance (high priority)
        obs_x, obs_y = self.obstacle_avoidance()
        target_vx += 0.6 * obs_x  # High weight for avoidance
        target_vy += 0.6 * obs_y

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

    # Rule 1: Separation
    def separation(self):
        """Rule 1: Separation - avoid collisions"""
        avoid_x, avoid_y = 0.0, 0.0
        count = 0
        
        for neighbor in self.neighbors.values():
            dx = self.x - neighbor['x']
            dy = self.y - neighbor['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.separation_distance and dist > 0:
                # Inversely proportional force to distance
                force = (self.separation_distance - dist) / self.separation_distance
                avoid_x += (dx / dist) * force
                avoid_y += (dy / dist) * force
                count += 1
        
        if count > 0:
            avoid_x /= count
            avoid_y /= count
        
        return avoid_x, avoid_y

    # Rule 2: Alignment
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

    # Rule 3: Cohesion
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