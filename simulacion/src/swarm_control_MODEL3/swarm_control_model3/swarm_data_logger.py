#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from datetime import datetime
from typing import Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class SwarmDataLogger(Node):
    def __init__(self):
        super().__init__('swarm_data_logger')

        self.declare_parameter('num_robots', 5)
        self.declare_parameter('log_nav_status', False)

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.log_nav_status = bool(self.get_parameter('log_nav_status').value)

        self.data_dir = os.path.expanduser('~/swarm_project/data')
        os.makedirs(self.data_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(
            self.data_dir,
            f'swarm_data_model3_{timestamp}.csv',
        )

        self.fieldnames = [
            'timestamp', 'robot_id', 'robot_name',
            'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'speed',
        ]
        if self.log_nav_status:
            self.fieldnames += ['nav_state', 'wp_index', 'dist_to_wp', 'evasions']

        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()

        # Último estado de navegación conocido por robot (opcional, viene de
        # waypoint_nav_swarm por /robotN/nav_status como JSON en String).
        self.nav_status: Dict[int, dict] = {}

        for i in range(self.num_robots):
            robot_name = f'robot{i+1}'
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odometry',
                lambda msg, robot_id=i: self.odom_callback(msg, robot_id),
                10,
            )
            if self.log_nav_status:
                self.create_subscription(
                    String,
                    f'/{robot_name}/nav_status',
                    lambda msg, robot_id=i: self.nav_status_callback(msg, robot_id),
                    10,
                )

        self.start_time = time.time()
        self.data_count = 0

        self.stats_timer = self.create_timer(10.0, self.print_stats)

        self.get_logger().info('Swarm Data Logger initialized for Model 3')
        self.get_logger().info(f'Data will be saved to: {self.csv_filename}')
        self.get_logger().info(f'Listening to {self.num_robots} robots (log_nav_status={self.log_nav_status})')

    def nav_status_callback(self, msg, robot_id):
        try:
            self.nav_status[robot_id] = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def odom_callback(self, msg, robot_id):
        current_time = time.time() - self.start_time

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        data_row = {
            'timestamp': round(current_time, 3),
            'robot_id': robot_id,
            'robot_name': f'robot{robot_id+1}',
            'pos_x': round(x, 4),
            'pos_y': round(y, 4),
            'pos_z': round(z, 4),
            'vel_x': round(vx, 4),
            'vel_y': round(vy, 4),
            'speed': round(speed, 4),
        }

        if self.log_nav_status:
            status = self.nav_status.get(robot_id, {})
            data_row['nav_state'] = status.get('state', '')
            data_row['wp_index'] = status.get('wp_index', '')
            data_row['dist_to_wp'] = status.get('dist_to_wp', '')
            data_row['evasions'] = status.get('evasions', '')

        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writerow(data_row)

            self.data_count += 1
        except Exception as e:
            self.get_logger().error(f'Error writing to CSV: {e}')

    def print_stats(self):
        current_time = time.time() - self.start_time
        rate = self.data_count / current_time if current_time > 0 else 0
        self.get_logger().info(
            f'Time: {current_time:.1f}s | Data points: {self.data_count} | Rate: {rate:.1f} Hz'
        )

    def generate_summary(self):
        try:
            summary_filename = self.csv_filename.replace('.csv', '_summary.txt')

            robot_data = {}
            total_points = 0

            with open(self.csv_filename, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    robot_id = int(row['robot_id'])
                    if robot_id not in robot_data:
                        robot_data[robot_id] = {
                            'speeds': [],
                            'positions': [],
                            'velocities_x': [],
                            'velocities_y': [],
                        }

                    robot_data[robot_id]['speeds'].append(float(row['speed']))
                    robot_data[robot_id]['positions'].append((float(row['pos_x']), float(row['pos_y'])))
                    robot_data[robot_id]['velocities_x'].append(float(row['vel_x']))
                    robot_data[robot_id]['velocities_y'].append(float(row['vel_y']))
                    total_points += 1

            with open(summary_filename, 'w') as f:
                f.write('SWARM BEHAVIOR DATA SUMMARY - MODEL 3\n')
                f.write('=' * 50 + '\n\n')
                f.write(f'CSV File: {os.path.basename(self.csv_filename)}\n')
                f.write(f'Total data points: {total_points}\n')
                f.write(f'Number of robots: {len(robot_data)}\n')
                f.write(f'Data points per robot: ~{total_points // len(robot_data) if robot_data else 0}\n\n')

                f.write('ROBOT STATISTICS:\n')
                f.write('-' * 30 + '\n')

                for robot_id, data in sorted(robot_data.items()):
                    speeds = data['speeds']
                    if speeds:
                        f.write(f'Robot {robot_id + 1}:\n')
                        f.write(f'  Data points: {len(speeds)}\n')
                        f.write(f'  Average speed: {sum(speeds)/len(speeds):.4f} m/s\n')
                        f.write(f'  Max speed: {max(speeds):.4f} m/s\n')
                        f.write(f'  Min speed: {min(speeds):.4f} m/s\n')

                        total_distance = sum(speeds) * 0.1
                        f.write(f'  Estimated total distance: {total_distance:.2f} m\n')

                        positions = data['positions']
                        if positions:
                            x_coords = [pos[0] for pos in positions]
                            y_coords = [pos[1] for pos in positions]
                            f.write(f'  X range: {min(x_coords):.2f} to {max(x_coords):.2f} m\n')
                            f.write(f'  Y range: {min(y_coords):.2f} to {max(y_coords):.2f} m\n')

                        vx = data['velocities_x']
                        vy = data['velocities_y']
                        if vx and vy:
                            f.write(f'  Avg vel_x: {sum(vx)/len(vx):.4f} m/s\n')
                            f.write(f'  Avg vel_y: {sum(vy)/len(vy):.4f} m/s\n')
                        f.write('\n')

                f.write('DATA ANALYSIS INSTRUCTIONS:\n')
                f.write('-' * 30 + '\n')
                f.write('You can analyze the CSV data using:\n')
                f.write('1. Excel/LibreOffice Calc\n')
                f.write('2. Python with matplotlib/pandas\n')
                f.write('3. Any data analysis tool that supports CSV\n\n')

                f.write('CSV COLUMNS:\n')
                f.write('- timestamp: Time since start (seconds)\n')
                f.write(f'- robot_id: Robot identifier (0-{self.num_robots - 1})\n')
                f.write(f'- robot_name: Robot name (robot1 to robot{self.num_robots})\n')
                f.write('- pos_x, pos_y, pos_z: Position coordinates (meters)\n')
                f.write('- vel_x, vel_y: Velocity components (m/s)\n')
                f.write('- speed: Total speed magnitude (m/s)\n')
                if self.log_nav_status:
                    f.write('- nav_state: waypoint_nav_swarm state (NORMAL/GIRANDO/AVANZANDO/EVADIENDO_*/DONE/DEADMAN)\n')
                    f.write('- wp_index: index of the waypoint currently being pursued\n')
                    f.write('- dist_to_wp: distance to the current waypoint (meters)\n')
                    f.write('- evasions: cumulative collision-avoidance maneuvers triggered\n')

            self.get_logger().info(f'Summary saved to: {summary_filename}')

        except Exception as e:
            self.get_logger().error(f'Error generating summary: {e}')


def main(args=None):
    rclpy.init(args=args)
    logger = SwarmDataLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print('\nGenerating data summary...')
        logger.generate_summary()
        print(f'Data saved to: {logger.csv_filename}')
        print(f"Summary saved to: {logger.csv_filename.replace('.csv', '_summary.txt')}")
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
