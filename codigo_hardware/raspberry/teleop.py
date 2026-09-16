import serial
import json
import time
import threading
import pigpio
import curses
from datetime import datetime

# ─────────────────────────────────────────
#  CONFIGURACIÓN
# ─────────────────────────────────────────
PUERTO       = '/dev/serial0'
BAUDIOS      = 115200
ARCHIVO_DATOS = "datos.jsonl"

MOTOR_IZQ_PIN = 18
MOTOR_DER_PIN  = 12
PWM_FREQ  = 50
PWM_RANGE = 1000

NEUTRAL  = 75
ADELANTE = 80
REVERSA  = 70

TIMEOUT_SEGURIDAD = 0.3   # segundos sin tecla → freno automático

# ─────────────────────────────────────────
#  HILO DE LECTURA DEL ESP32
# ─────────────────────────────────────────
_stop_event = threading.Event()   # se activa para pedir que el hilo pare

def guardar_en_archivo(datos: dict):
    datos["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(ARCHIVO_DATOS, "a") as f:
        f.write(json.dumps(datos) + "\n")

def hilo_sensor():
    """Lee datos del ESP32 y los guarda en datos.jsonl de forma continua."""
    try:
        ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    except serial.SerialException as e:
        # Si no puede abrir el puerto simplemente termina el hilo;
        # el control de motores sigue funcionando.
        return

    while not _stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                linea = ser.readline().decode('utf-8').strip()
                if linea:
                    datos = json.loads(linea)
                    guardar_en_archivo(datos)
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except Exception:
            pass

    ser.close()

# ─────────────────────────────────────────
#  MOTORES
# ─────────────────────────────────────────
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

# ─────────────────────────────────────────
#  CONTROL POR TECLADO (curses)
# ─────────────────────────────────────────
def teleop_robot(stdscr):
    pi = pigpio.pi()
    if not pi.connected:
        return

    stdscr.clear()
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "=== Control de motores ===")
    stdscr.addstr(1, 0, "MANTÉN PRESIONADA la tecla para mover.")
    stdscr.addstr(2, 0, "[W] Adelante  [S] Atrás  [A] Izq  [D] Der  [Q] Salir")
    stdscr.addstr(3, 0, f"Sensores: guardando en '{ARCHIVO_DATOS}'")
    stdscr.refresh()

    init_motores(pi)

    estado_actual        = "DETENIDO"
    ultimo_comando_tiempo = time.time()

    try:
        while True:
            stdscr.addstr(5, 0, f"Estado: {estado_actual}          ")
            stdscr.refresh()

            try:
                tecla = stdscr.getkey().lower()
                ultimo_comando_tiempo = time.time()

                if tecla == 'w':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, ADELANTE)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, ADELANTE)
                    estado_actual = "AVANZANDO"
                elif tecla == 's':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, REVERSA)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, REVERSA)
                    estado_actual = "RETROCEDIENDO"
                elif tecla == 'a':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, REVERSA)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, ADELANTE)
                    estado_actual = "GIRANDO IZQ"
                elif tecla == 'd':
                    pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, ADELANTE)
                    pi.set_PWM_dutycycle(MOTOR_DER_PIN, REVERSA)
                    estado_actual = "GIRANDO DER"
                elif tecla == 'q':
                    break

            except curses.error:
                pass   # sin tecla presionada

            # Interruptor de hombre muerto
            if time.time() - ultimo_comando_tiempo > TIMEOUT_SEGURIDAD:
                pi.set_PWM_dutycycle(MOTOR_IZQ_PIN, NEUTRAL)
                pi.set_PWM_dutycycle(MOTOR_DER_PIN, NEUTRAL)
                estado_actual = "DETENIDO (Auto-Freno)"

            time.sleep(0.05)

    finally:
        apagar_motores(pi)

# ─────────────────────────────────────────
#  PUNTO DE ENTRADA
# ─────────────────────────────────────────
def main():
    # Arrancar el hilo de sensores en segundo plano
    t = threading.Thread(target=hilo_sensor, daemon=True)
    t.start()

    try:
        curses.wrapper(teleop_robot)
    finally:
        # Avisar al hilo que pare y esperar a que cierre el puerto limpiamente
        _stop_event.set()
        t.join(timeout=2)

if __name__ == '__main__':
    main()