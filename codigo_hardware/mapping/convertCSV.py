import json
import csv
#'''
#With google maps
def jsonl_a_csv_para_maps(archivo_entrada, archivo_salida):
    with open(archivo_entrada, 'r') as f_in, open(archivo_salida, 'w', newline='') as f_out:
        # Definimos las columnas del CSV
        # Asegúrate de que estos nombres coincidan con los de tu JSONL
        columnas = ['latitude', 'longitude', 'temp', 'ph', 'timestamp']
        writer = csv.DictWriter(f_out, fieldnames=columnas)
        
        writer.writeheader()
        
        for linea in f_in:
            if not linea.strip(): continue
            data = json.loads(linea)
            
            # Limpiamos los datos para el CSV
            # En lugar de guardar el valor directo, lo convertimos a string con coma
            fila = {
                'latitude': data.get('lat'),
                'longitude': data.get('lon'),
                'temp': str(data.get('t', 0)).replace('.', ','),
                'ph': str(data.get('ph', 0)).replace('.', ',')
            }
            writer.writerow(fila)

    print(f"¡Listo! Archivo CSV creado: {archivo_salida}")

jsonl_a_csv_para_maps('CasaIvangrupal1.jsonl', 'trayectoria_datosGoogle1.csv')

#'''

#With kepler.gl
'''
def jsonl_a_csv_para_maps(archivo_entrada, archivo_salida):
    with open(archivo_entrada, 'r') as f_in, open(archivo_salida, 'w', newline='') as f_out:
        # Definimos las columnas del CSV
        # Asegúrate de que estos nombres coincidan con los de tu JSONL
        columnas = ['latitude', 'longitude', 'temp', 'ph', 'timestamp']
        writer = csv.DictWriter(f_out, fieldnames=columnas)
        
        writer.writeheader()
        
        for linea in f_in:
            if not linea.strip(): continue
            data = json.loads(linea)
            
            # Limpiamos los datos para el CSV
            # En lugar de guardar el valor directo, lo convertimos a string con coma
            fila = {
                'latitude': data.get('lat'),
                'longitude': data.get('lon'),
                'temp': data.get('t'),
                'ph': data.get('ph')
            }
            writer.writerow(fila)

    print(f"¡Listo! Archivo CSV creado: {archivo_salida}")

jsonl_a_csv_para_maps('datos1.jsonl', 'trayectoria_datosKepler1.csv')
'''