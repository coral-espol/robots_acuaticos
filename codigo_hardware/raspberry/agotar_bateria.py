import pigpio
import time
import curses

MOTOR_IZQ_PIN = 18
MOTOR_DER_PIN = 12
PWM_FREQ = 50
PWM_RANGE = 1000

NEUTRAL  = 75
ADELANTE = 80

def init_motores(pi):
    for pin in [MOTOR_IZQ_PIN, MOTOR_DER_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, PWM_FREQ)
        pi.set_PWM_range(pin, PWM_RANGE)
        pi.set_PWM_dutycycle(pin, NEUTRAL)
    time.sleep(3)  # Espera de armado del ESC

def apagar_motores(pi):
    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, NEUTRAL)  # NEUTRAL, no 0, para ESCs
    pi.set_PWM_dutycycle(MOTOR_DER_PIN, NEUTRAL)
    time.sleep(0.5)
    pi.stop()

def descargar_bateria(stdscr):
    pi = pigpio.pi()
    if not pi.connected:
        stdscr.addstr(0, 0, "ERROR: No se pudo conectar a pigpio.")
        stdscr.refresh()
        time.sleep(2)
        return

    stdscr.clear()
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "=== DESCARGA DE BATERÍA ===")
    stdscr.addstr(1, 0, "Iniciando ESCs (3 segundos)...")
    stdscr.refresh()

    init_motores(pi)

    # Arrancar motores inmediatamente al comenzar
    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, ADELANTE)
    pi.set_PWM_dutycycle(MOTOR_DER_PIN, ADELANTE)

    stdscr.addstr(1, 0, "Motores en marcha. Presiona [Q] para parar.")
    stdscr.refresh()

    try:
        while True:
            try:
                tecla = stdscr.getkey().lower()
                if tecla == 'q':
                    stdscr.addstr(3, 0, "Deteniendo motores...")
                    stdscr.refresh()
                    break
            except curses.error:
                pass  # No hay tecla, continuar normalmente

            time.sleep(0.05)

    finally:
        apagar_motores(pi)
        stdscr.addstr(4, 0, "Motores apagados. Saliendo.")
        stdscr.refresh()
        time.sleep(1)

if __name__ == "__main__":
    curses.wrapper(descargar_bateria)