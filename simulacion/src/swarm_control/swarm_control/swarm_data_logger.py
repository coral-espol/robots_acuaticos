#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime
import time
import math
# Data Logger Node
# Description: This node logs the odometry data of a swarm of robots.
# It subscribes to the odometry topics of each robot and saves the data to a CSV file.
# The data is logged in real-time and can be used for post-analysis of the swarm behavior.

class SwarmDataLogger(Node):
    def __init__(self):
        super().__init__('swarm_data_logger')
        
        # Configuration
        self.num_robots = 5 # Number of robots in the swarm (needs to be changed manually)

        # Create data directory
        self.data_dir = os.path.expanduser('~/swarm_project/data')
        os.makedirs(self.data_dir, exist_ok=True)

        # Filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.data_dir, f'swarm_data_{timestamp}.csv')

        # Create CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'robot_id', 'robot_name',
                'pos_x', 'pos_y', 'vel_x', 'vel_y', 'speed'
            ]
            self.csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            self.csv_writer.writeheader()

        # Subscribers to the odometry of all robots
        for i in range(self.num_robots):
            robot_name = f'swarm_bot_{i+1}'
            self.create_subscription(
                Odometry,
                f'/swarm/{robot_name}/odom',
                lambda msg, robot_id=i: self.odom_callback(msg, robot_id),
                10
            )
        
        self.start_time = time.time()
        self.data_count = 0

        # Timer to show statistics every 10 seconds
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        
        self.get_logger().info(f'Swarm Data Logger initialized')
        self.get_logger().info(f'Data will be saved to: {self.csv_filename}')
    
    def odom_callback(self, msg, robot_id):
        """Callback para recibir y guardar datos de odometría"""
        current_time = time.time() - self.start_time

        # Extract data
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Prepare data for CSV
        data_row = {
            'timestamp': round(current_time, 3),
            'robot_id': robot_id,
            'robot_name': f'swarm_bot_{robot_id+1}',
            'pos_x': round(x, 4),
            'pos_y': round(y, 4),
            'vel_x': round(vx, 4),
            'vel_y': round(vy, 4),
            'speed': round(speed, 4)
        }

        # Save to CSV immediately
        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'robot_id', 'robot_name',
                    'pos_x', 'pos_y', 'vel_x', 'vel_y', 'speed'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow(data_row)
            
            self.data_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error writing to CSV: {e}')
    
    def print_stats(self):
        """Show statistics every 10 seconds"""
        current_time = time.time() - self.start_time
        self.get_logger().info(f'Time: {current_time:.1f}s | Data points saved: {self.data_count}')
    
    def generate_summary(self):
        """Generate summary file upon completion"""
        try:
            summary_filename = self.csv_filename.replace('.csv', '_summary.txt')

            # Read the CSV file to generate statistics
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
                            'velocities_y': []
                        }
                    
                    robot_data[robot_id]['speeds'].append(float(row['speed']))
                    robot_data[robot_id]['positions'].append((float(row['pos_x']), float(row['pos_y'])))
                    robot_data[robot_id]['velocities_x'].append(float(row['vel_x']))
                    robot_data[robot_id]['velocities_y'].append(float(row['vel_y']))
                    total_points += 1

            # Write summary
            with open(summary_filename, 'w') as f:
                f.write("SWARM BEHAVIOR DATA SUMMARY\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"CSV File: {os.path.basename(self.csv_filename)}\n")
                f.write(f"Total data points: {total_points}\n")
                f.write(f"Number of robots: {len(robot_data)}\n")
                f.write(f"Data points per robot: ~{total_points // len(robot_data) if robot_data else 0}\n\n")
                
                f.write("ROBOT STATISTICS:\n")
                f.write("-" * 30 + "\n")
                
                for robot_id, data in robot_data.items():
                    speeds = data['speeds']
                    if speeds:
                        f.write(f"Robot {robot_id + 1} (swarm_bot_{robot_id + 1}):\n")
                        f.write(f"  Data points: {len(speeds)}\n")
                        f.write(f"  Average speed: {sum(speeds)/len(speeds):.4f} m/s\n")
                        f.write(f"  Max speed: {max(speeds):.4f} m/s\n")
                        f.write(f"  Min speed: {min(speeds):.4f} m/s\n")

                        # Calculate estimated total distance
                        total_distance = sum(speeds) * 0.1  # Assuming 10Hz
                        f.write(f"  Estimated total distance: {total_distance:.2f} m\n")

                        # Position range
                        positions = data['positions']
                        if positions:
                            x_coords = [pos[0] for pos in positions]
                            y_coords = [pos[1] for pos in positions]
                            f.write(f"  X range: {min(x_coords):.2f} to {max(x_coords):.2f} m\n")
                            f.write(f"  Y range: {min(y_coords):.2f} to {max(y_coords):.2f} m\n")
                        f.write("\n")

                # Instructions for analysis
                f.write("DATA ANALYSIS INSTRUCTIONS:\n")
                f.write("-" * 30 + "\n")
                f.write("You can analyze the CSV data using:\n")
                f.write("1. Excel/LibreOffice Calc\n")
                f.write("2. Python with matplotlib:\n")
                f.write("   import csv, matplotlib.pyplot as plt\n")
                f.write("3. Any data analysis tool that supports CSV\n\n")
                
                f.write("CSV COLUMNS:\n")
                f.write("- timestamp: Time since start (seconds)\n")
                f.write("- robot_id: Robot identifier (0-4)\n")
                f.write("- robot_name: Robot name (swarm_bot_1 to swarm_bot_5)\n")
                f.write("- pos_x, pos_y: Position coordinates (meters)\n")
                f.write("- vel_x, vel_y: Velocity components (m/s)\n")
                f.write("- speed: Total speed magnitude (m/s)\n")
            
            self.get_logger().info(f'Summary saved to: {summary_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error generating summary: {e}')

# Main function
def main(args=None):
    rclpy.init(args=args)
    # Create the logger
    logger = SwarmDataLogger()
    # Start the logger
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("\nGenerating data summary...")
        logger.generate_summary()
        print(f"Data saved to: {logger.csv_filename}")
        print(f"Summary saved to: {logger.csv_filename.replace('.csv', '_summary.txt')}")
    finally:
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()