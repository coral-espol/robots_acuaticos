
import json

def convertir_a_trayectoria_y_puntos(archivo_entrada, archivo_salida):
    coordenadas_linea = []
    features = []
    
    with open(archivo_entrada, 'r') as f:
        for linea in f:
            if not linea.strip():
                continue
            data = json.loads(linea)
            
            lat = data.get('lat')
            lon = data.get('lon')
            
            if lat is not None and lon is not None:
                lon_f, lat_f = float(lon), float(lat)
                
                # 1. Acumulamos coordenadas para la trayectoria (LineString)
                coordenadas_linea.append([lon_f, lat_f])
                
                # 2. Extraemos los datos adicionales para cada punto
                temp = data.get('t')
                ph = data.get('ph')
                timestamp = data.get('timestamp')

                # Creamos el Feature de tipo Point
                punto_feature = {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [lon_f, lat_f]
                    },
                    "properties": {
                        "temperatura": temp,
                        "ph": ph,
                        "tiempo": timestamp
                    }
                }
                features.append(punto_feature)

    # 3. Creamos el Feature de tipo LineString para la trayectoria completa
    trayectoria_feature = {
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": coordenadas_linea
        },
        "properties": {
            "nombre": "Trayectoria GPS",
            "total_puntos": len(coordenadas_linea)
        }
    }

    # Insertamos la línea al principio (o al final) de la lista de features
    features.insert(0, trayectoria_feature)

    # Estructura final del GeoJSON
    geojson = {
        "type": "FeatureCollection",
        "features": features
    }

    # Guardamos el resultado
    with open(archivo_salida, 'w') as f:
        json.dump(geojson, f, indent=4)
    
    print(f"Archivo generado con éxito: {archivo_salida}")
    print(f"- 1 Línea de trayectoria creada.")
    print(f"- {len(features) - 1} Puntos con datos registrados.")

# Uso del script unificado
convertir_a_trayectoria_y_puntos('Lago_Espol.jsonl', 'trayectoria_lago.geojson')