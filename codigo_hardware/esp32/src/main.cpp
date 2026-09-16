#include <Arduino.h>
#include <ArduinoJson.h>
#include "gps.h"
#include "ph.h"
#include "temp.h"
#include "espnow.h"

// --- Variables para el envío a la Raspberry Pi ---
unsigned long tiempoAnteriorEnvio = 0;
const long intervaloEnvio = 5000; // Enviar el JSON cada 5 segundos

void setup() {
    Serial.begin(115200);
    
    // Damos tiempo a que el serial se estabilice
    delay(100);

    // Inicializamos cada sensor
    setup_temp();
    setup_ph();
    setup_gps();
    setup_espnow();

    Serial.println("INICIO: Nodo de sensores listo. Esperando datos...");
}

void enviar_json_datos() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnteriorEnvio >= intervaloEnvio) {
    tiempoAnteriorEnvio = tiempoActual;

    // Creamos el documento JSON. La versión 7 gestiona la memoria automáticamente.
    JsonDocument doc;

    // Asignamos las variables globales, las guardamos con los decimales que necesitemos
    doc["tipo"] = "datos";
    doc["ph"] = serialized(String(ph, 2)); // Guardamos el pH con 2 decimales  
    //doc["volt"] = serialized(String(volt, 4)); // Guardamos el voltaje con 4 decimales
    doc["t"] = serialized(String(temp, 2)); 
    doc["lat"] = latitud; 
    doc["lon"] = longitud;


    // Empaquetamos y enviamos por el puerto Serial
    serializeJson(doc, Serial);
    
    // Imprimimos un salto de linea para que Python sepa dónde termina el mensaje
    Serial.println(); 
  }
}

void enviar_json_esp() {
  // Solo se ejecuta si el callback de ESP-NOW levantó la bandera
  if (nuevo_dato_enjambre) {
    JsonDocument doc;
    
    doc["tipo"] = "enjambre"; // Etiqueta para Python
    doc["id_robot"] = id_robot_recibido;
    doc["lat"] = lat_recibida;
    doc["lon"] = lon_recibida;

    serializeJson(doc, Serial);
    Serial.println(); 
    
    // ¡Muy importante! Bajamos la bandera para no repetir el mensaje
    nuevo_dato_enjambre = false;
  }
}



void loop() {
    // Leemos los datos en cada ciclo
    leer_temp();
    leer_ph();
    leer_gps();

    // Activamos ESP-NOW
    loop_espnow();
    // Revisamos si ya toca empaquetar y enviar el JSON
    enviar_json_datos();
    enviar_json_esp();
}