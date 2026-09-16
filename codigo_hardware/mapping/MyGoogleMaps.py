import json
import csv

def convertir_para_my_maps(archivo_entrada):
    coordenadas = []
    puntos = []

    with open(archivo_entrada, 'r', encoding='utf-8') as f:
        for linea in f:
            if not linea.strip():
                continue
            data = json.loads(linea)

            lat = data.get('lat')
            lon = data.get('lon')

            if lat is not None and lon is not None:
                lat_f, lon_f = float(lat), float(lon)
                coordenadas.append((lon_f, lat_f))  # Para KML (lon, lat)

                t_val = data.get('t')
                ph_val = data.get('ph')

                puntos.append({
                    # lat/lon SIEMPRE con punto: My Maps las reconoce como
                    # coordenadas y usa un parser fijo internacional (punto decimal)
                    'latitud': lat_f,
                    'longitud': lon_f,
                    # temperatura y ph son columnas de datos "normales": My Maps
                    # las interpreta según la configuración regional (Ecuador/España
                    # usan punto para miles y coma para decimales), por eso se
                    # escriben aquí con coma para que se muestren bien (23,45 -> 23.45)
                    'temperatura': f"{t_val:.2f}".replace('.', ',') if t_val is not None else '',
                    'ph': f"{ph_val:.2f}".replace('.', ',') if ph_val is not None else '',
                    'tiempo': data.get('timestamp')
                })

    # 1. Crear el CSV para los Puntos con datos
    with open('puntos_lago2.csv', 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['latitud', 'longitud', 'temperatura', 'ph', 'tiempo'])
        writer.writeheader()
        writer.writerows(puntos)

    # 2. Crear un archivo KML para la Trayectoria (Línea)
    # El KML es un formato XML estándar internacional: aquí SIEMPRE va con
    # punto decimal, sin importar la configuración regional, así que no se toca.
    kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Trayectoria GPS</name>
    <Style id="lineaRuta">
      <LineStyle>
        <color>ff0000ff</color>
        <width>4</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>Ruta completa</name>
      <styleUrl>#lineaRuta</styleUrl>
      <LineString>
        <coordinates>
          {" ".join([f"{lon},{lat},0" for lon, lat in coordenadas])}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>"""

    with open('trayectoria_lago2.kml', 'w', encoding='utf-8') as f:
        f.write(kml_content)

    print("¡Archivos generados con éxito!")
    print("-> 'puntos_x.csv' (Súbelo como capa de puntos en My Maps)")
    print("-> 'trayectoria_x.kml' (Súbelo como capa de línea en My Maps)")

# Uso del script
convertir_para_my_maps('Lago_Espol2.jsonl')