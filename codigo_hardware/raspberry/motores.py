import pigpio
import time

pi = pigpio.pi()

# Definimos ambos pines
MOTOR_IZQ_PIN = 18  # Motor izquierdo
MOTOR_DER_PIN = 12  # Motor derecho

PWM_FREQ = 50 
PWM_RANGE = 1000

# Configuramos ambos pines como salidas
pi.set_mode(MOTOR_IZQ_PIN, pigpio.OUTPUT)
pi.set_mode(MOTOR_DER_PIN, pigpio.OUTPUT)

# Ajustamos frecuencia y rango para ambos
pi.set_PWM_frequency(MOTOR_IZQ_PIN, PWM_FREQ)
pi.set_PWM_frequency(MOTOR_DER_PIN, PWM_FREQ)
pi.set_PWM_range(MOTOR_IZQ_PIN, PWM_RANGE)
pi.set_PWM_range(MOTOR_DER_PIN, PWM_RANGE)

# Secuencia de armado (ambos al mismo tiempo)
print("Armando ambos motores...")
pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, 75)
pi.set_PWM_dutycycle(MOTOR_DER_PIN, 75)
time.sleep(3)

# Ejemplo: Avanzar con ambos (Del 76 al 100 va aumentando la velocidad hacia adelante)
print("Avanzando...")
pi.set_PWM_dutycycle(MOTOR_IZQ_PIN,80)
pi.set_PWM_dutycycle(MOTOR_DER_PIN, 80)
time.sleep(5)

# Detener los motores un momento
#Mientras más lejos del punto medio (75) más acelera
print("Deteniendo ambos motores...")
pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, 75)
pi.set_PWM_dutycycle(MOTOR_DER_PIN, 75)
time.sleep(3)

#Retroceder con ambos (Del 51 al 74 va aumentando la velocidad hacia atras)
print("Retrocediendo...")
pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, 70)
pi.set_PWM_dutycycle(MOTOR_DER_PIN, 70)
time.sleep(3)

# Limpieza final para ambos motores
pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, 0)
pi.set_PWM_dutycycle(MOTOR_DER_PIN, 0)
pi.stop()