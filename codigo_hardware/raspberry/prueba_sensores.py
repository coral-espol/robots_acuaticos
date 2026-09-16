import serial
import json
import time
from datetime import datetime, timedelta

# --- CONFIGURACIÓN ---
PUERTO = '/dev/serial0'
BAUDIOS = 115200
ARCHIVO_TIEMPO = "tiempo_activo.txt"   # Aquí se guarda el tiempo de actividad (prueba de batería)
INTERVALO_GUARDADO_TIEMPO = 10          # segundos entre cada actualización del archivo de tiempo

def iniciar_conexion():
    try:
        ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)
        print(f"Conectado con éxito al ESP32 en {PUERTO}")
        return ser
    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
        exit()

def formatear_tiempo(segundos):
    return str(timedelta(seconds=int(segundos)))

def guardar_tiempo_activo(inicio, motivo="en curso"):
    """
    Guarda el tiempo transcurrido en un archivo aparte, sobrescribiéndolo
    cada vez. Así siempre queda el último valor conocido en disco, incluso
    si el script se cae solo o se desconecta el wifi sin avisar.
    """
    transcurrido = time.time() - inicio
    with open(ARCHIVO_TIEMPO, "w") as f:
        f.write(f"Inicio: {datetime.fromtimestamp(inicio).strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Última actualización: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Tiempo activo: {formatear_tiempo(transcurrido)}\n")
        f.write(f"Estado: {motivo}\n")

def main():
    ser = iniciar_conexion()
    inicio = time.time()
    ultima_actualizacion_tiempo = 0.0
    print("Iniciando lectura de sensores (los datos ya no se guardan en archivo)...\n")

    # Guardamos el tiempo de inicio de una vez, por si el script se cae de inmediato
    guardar_tiempo_activo(inicio, motivo="en curso")

    while True:
        try:
            if ser.in_waiting > 0:
                linea = ser.readline().decode('utf-8').strip()

                if linea:
                    datos = json.loads(linea)
                    datos["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    tiempo_activo = time.time() - inicio

                    print("-" * 40)
                    print(f"Tiempo activo : {formatear_tiempo(tiempo_activo)}")
                    print(f"Tiempo : {datos['timestamp']}")
                    print(f"pH     : {datos.get('ph')}")
                    print(f"Temp   : {datos.get('t')} C")
                    print(f"GPS    : Lat {datos.get('lat')}, Lon {datos.get('lon')}")

            # Actualizamos el archivo de tiempo activo cada cierto intervalo,
            # no en cada lectura, para no desgastar la SD con escrituras constantes
            if time.time() - ultima_actualizacion_tiempo >= INTERVALO_GUARDADO_TIEMPO:
                guardar_tiempo_activo(inicio, motivo="en curso")
                ultima_actualizacion_tiempo = time.time()

        except json.JSONDecodeError:
            print("Trama corrupta o incompleta ignorada.")
        except UnicodeDecodeError:
            pass
        except KeyboardInterrupt:
            print("\nCerrando lectura de sensores...")
            guardar_tiempo_activo(inicio, motivo="detenido manualmente (Ctrl+C)")
            ser.close()
            break
        except Exception as e:
            print(f"Error inesperado: {e}")

if __name__ == '__main__':
    main()