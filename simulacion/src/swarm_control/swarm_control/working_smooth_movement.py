#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time
import math
import threading

class WorkingSmoothMovement(Node):
    def __init__(self):
        super().__init__('working_smooth_movement')
        
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocidades objetivo (suaves)
        self.target_vx = 0.0
        self.target_omega = 0.0
        
        # Velocidades actuales (filtradas)
        self.current_vx = 0.0
        self.current_omega = 0.0
        
        # Control de tiempo
        self.last_update = time.time()
        self.last_cmd_time = 0.0
        self.start_time = time.time()
        
        # Timer para calcular movimiento (alta frecuencia)
        self.calc_timer = self.create_timer(0.02, self.calculate_motion)  # 50Hz para cálculos
        
        # Timer para actualizar Gazebo (baja frecuencia)
        self.update_timer = self.create_timer(0.1, self.update_gazebo)  # 10Hz para Gazebo
        
        # Subscriber para comandos manuales
        self.cmd_sub = self.create_subscription(
            Twist,
            '/swarm_bot_1/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Flag para evitar comandos simultáneos a Gazebo
        self.updating_gazebo = False
        
        self.get_logger().info('Working smooth movement started - based on service_movement')
    
    def cmd_callback(self, msg):
        """Recibir comandos manuales"""
        self.target_vx = msg.linear.x * 0.3  # Escalar para navegación suave
        self.target_omega = msg.angular.z * 0.4
        self.last_cmd_time = time.time()
        
        self.get_logger().info(f'Manual command: vx={self.target_vx:.2f}, omega={self.target_omega:.2f}')
    
    def calculate_motion(self):
        """Calcular movimiento suave (alta frecuencia)"""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        # Patrón automático si no hay comandos manuales
        if current_time - self.last_cmd_time > 3.0:
            # Navegación automática en patrón suave
            t = current_time - self.start_time
            self.target_vx = 0.2 + 0.1 * math.sin(t * 0.3)
            self.target_omega = 0.15 * math.sin(t * 0.4)
        
        # Filtro de paso bajo para suavizar (simula inercia acuática)
        alpha = 0.1  # Factor de suavizado
        self.current_vx += alpha * (self.target_vx - self.current_vx)
        self.current_omega += alpha * (self.target_omega - self.current_omega)
        
        # Integración cinemática
        if dt < 0.1:  # Evitar saltos grandes por pausas
            self.theta += self.current_omega * dt
            self.x += self.current_vx * math.cos(self.theta) * dt
            self.y += self.current_vx * math.sin(self.theta) * dt
    
    def update_gazebo(self):
        """Actualizar Gazebo (baja frecuencia)"""
        if self.updating_gazebo:
            return  # Evitar comandos simultáneos
        
        self.updating_gazebo = True
        
        # Usar thread para no bloquear
        thread = threading.Thread(target=self._update_pose_async)
        thread.daemon = True
        thread.start()
    
    def _update_pose_async(self):
        """Actualizar pose en Gazebo - navegando EN el agua"""
        try:
            qz = math.sin(self.theta / 2.0)
            qw = math.cos(self.theta / 2.0)
            
            cmd = [
                'timeout', '0.5',
                'gz', 'service', '-s', '/world/flat_ocean/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '200',
                '--req', 
                f'name: "swarm_bot_1" '
                f'position {{x: {self.x:.4f} y: {self.y:.4f} z: 0.0}} '  # ← CAMBIO: z=0.0 (en el agua)
                f'orientation {{x: 0.0 y: 0.0 z: {qz:.4f} w: {qw:.4f}}}'
            ]
            
            # Ejecutar sin capturar salida
            result = subprocess.run(
                cmd, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL,
                timeout=0.5
            )
            
            # Log esporádico
            current_time = time.time()
            if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.2:
                mode = "Manual" if (current_time - self.last_cmd_time < 3.0) else "Auto"
                self.get_logger().info(
                    f'{mode} - Pos: ({self.x:.2f}, {self.y:.2f}), '
                    f'θ: {math.degrees(self.theta):.1f}°, '
                    f'v: {self.current_vx:.2f}, ω: {self.current_omega:.2f}'
                )
                
        except subprocess.TimeoutExpired:
            pass  # Ignorar timeouts
        except Exception as e:
            self.get_logger().debug(f'Gazebo update error (ignorado): {e}')
        finally:
            self.updating_gazebo = False

def main(args=None):
    rclpy.init(args=args)
    node = WorkingSmoothMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()