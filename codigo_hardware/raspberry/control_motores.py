import pigpio
import time
import curses

# Configuración de pines y PWM
MOTOR_IZQ_PIN = 18
MOTOR_DER_PIN = 12
PWM_FREQ = 50
PWM_RANGE = 1000

# Valores PWM de control
NEUTRAL = 75
ADELANTE = 80
REVERSA = 70

TIMEOUT_SEGURIDAD = 0.3  # Si no hay señal en 0.3 segundos, frena

def init_motores(pi):
    for pin in [MOTOR_IZQ_PIN, MOTOR_DER_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, PWM_FREQ)
        pi.set_PWM_range(pin, PWM_RANGE)
        pi.set_PWM_dutycycle(pin, NEUTRAL)
    time.sleep(3)

def apagar_motores(pi):
    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, 0)
    pi.set_PWM_dutycycle(MOTOR_DER_PIN, 0)
    pi.stop()

def teleop_robot(stdscr):
    pi = pigpio.pi()
    if not pi.connected:
        return

    stdscr.clear()
    stdscr.nodelay(True)
    
    stdscr.addstr(0, 0, "=== Control por Termius SSH ===")
    stdscr.addstr(1, 0, "MANTÉN PRESIONADA la tecla para mover.")
    stdscr.addstr(2, 0, "[W] Adelante  [S] Atrás  [A] Izq  [D] Der  [Q] Salir")
    stdscr.refresh()
    
    init_motores(pi)
    
    estado_actual = "DETENIDO"
    ultimo_comando_tiempo = time.time()

    try:
        while True:
            stdscr.addstr(4, 0, f"Estado: {estado_actual}          ")
            
            try:
                tecla = stdscr.getkey().lower()
                ultimo_comando_tiempo = time.time() # Reiniciar el cronómetro
                
                if tecla == 'w':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, ADELANTE)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, ADELANTE)
                    estado_actual = "AVANZANDO"
                elif tecla == 's':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, REVERSA)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, REVERSA)
                    estado_actual = "RETROCEDIENDO"
                elif tecla == 'a':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_P1IN, REVERSA)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, ADELANTE)
                    estado_actual = "GIRANDO IZQ"
                elif tecla == 'd':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, ADELANTE)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, REVERSA)
                    estado_actual = "GIRANDO DER"
                elif tecla == 'q':
                    break
                    
            except curses.error:
                # Si no se detecta ninguna tecla, evaluamos el tiempo
                pass
                
            # --- INTERRUPTOR DE HOMBRE MUERTO ---
            # Termius envía la tecla repetidamente si la mantienes presionada.
            # Si pasa más de 0.3s sin recibir la letra, asumimos que soltaste
            # la tecla o que se cortó el Wi-Fi, y frenamos de emergencia.
            if time.time() - ultimo_comando_tiempo > TIMEOUT_SEGURIDAD:
                pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, NEUTRAL)
                pi.set_PWM_dutycycle(MOTOR_DER_PIN, NEUTRAL)
                estado_actual = "DETENIDO (Auto-Freno)"

            time.sleep(0.05)

    finally:
        apagar_motores(pi)

if __name__ == "__main__":
    curses.wrapper(teleop_robot)