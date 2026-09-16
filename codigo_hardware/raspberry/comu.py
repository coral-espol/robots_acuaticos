import serial
import json
import time
from datetime import datetime

# --- CONFIGURACIÓN ---
PUERTO = '/dev/serial0' 
BAUDIOS = 115200
ARCHIVO_DATOS = "datos.jsonl" 

def iniciar_conexion():
    try:
        ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)
        print(f"Conectado con éxito al ESP32 en {PUERTO}")
        return ser
    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
        exit()

def guardar_en_archivo(datos):
    # 1. Agregamos la marca de tiempo exacta al diccionario
    datos["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # 2. Abrimos el archivo en modo 'a' (append) para añadir la línea al final
    with open(ARCHIVO_DATOS, "a") as f:
        # 3. Convertimos el diccionario a un string JSON y le sumamos el salto de línea
        linea_json = json.dumps(datos)
        f.write(linea_json + "\n")

def main():
    ser = iniciar_conexion()
    print(f"Iniciando registro de datos en {ARCHIVO_DATOS}...\n")

    while True:
        try:
            if ser.in_waiting > 0:
                linea = ser.readline().decode('utf-8').strip()
                
                if linea:
                    # Parseamos la línea que viene del ESP32
                    datos = json.loads(linea)
                    
                    # Guardamos la lectura en el archivo JSON Lines
                    guardar_en_archivo(datos)
                    
                    # Imprimimos en consola para monitoreo en vivo
                    print("-" * 40)
                    print(f"Tiempo : {datos['timestamp']}")
                    print(f"pH     : {datos.get('ph')}")
                    print(f"Temp   : {datos.get('t')} C")
                    print(f"GPS    : Lat {datos.get('lat')}, Lon {datos.get('lon')}")

        except json.JSONDecodeError:
            print("Trama corrupta o incompleta ignorada.")
        except UnicodeDecodeError:
            pass
        except KeyboardInterrupt:
            # Esto es vital para no dejar el archivo corrupto si presionas Ctrl+C
            print("\nCerrando lectura y guardado de sensores...")
            ser.close()
            break
        except Exception as e:
            print(f"Error inesperado: {e}")

if __name__ == '__main__':
    main()