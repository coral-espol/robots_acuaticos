#include "temp.h"
#include <Arduino.h> 
#include <OneWire.h>
#include <DallasTemperature.h>

// Variables globales reales donde guardaremos los datos limpios
float temp = 0.0;

// Pin donde conectaste el cable de datos (D25)
const int oneWireBus = 25;     

// Instancia oneWire para comunicarse con dispositivos 1-Wire
OneWire oneWire(oneWireBus);

// Referencia de oneWire a la librería Dallas Temperature
DallasTemperature sensors(&oneWire);

// --- Variables para el cronómetro ---
unsigned long tiempoAnteriorTemp = 0;
const long intervaloTemp = 2000; // 2000 milisegundos = 2 segundos

void setup_temp() {
  // Iniciamos el monitor serie
  //Serial.println("Iniciando sensor DS18B20...");
  // Iniciamos la librería del sensor
  sensors.begin();
}

void leer_temp() {
  //millis() devuelve el tiempo transcurrido desde que el programa comenzó a ejecutarse, en milisegundos. 
  //Lo usamos para controlar cada cuánto tiempo leemos el sensor.
  unsigned long tiempoActual = millis();
  // Verificamos si ya pasó 1 segundo desde la última lectura
  if (tiempoActual - tiempoAnteriorTemp >= intervaloTemp) {
    tiempoAnteriorTemp = tiempoActual; // Reiniciamos el cronómetro

    sensors.requestTemperatures(); 
    float tempC = sensors.getTempCByIndex(0);

    if(tempC != DEVICE_DISCONNECTED_C) {
      //Solo descomentar si se quiere ver los datos en el serial
      /*
      Serial.print("Temperatura detectada:");
      Serial.print(tempC);
      Serial.println(" ºC");
      */
      temp = tempC;
    } else {
      //Serial.println("Error: No se pudo leer el sensor DS18B20 (Revisa conexiones).");
    }
  }
}