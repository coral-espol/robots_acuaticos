#include "gps.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>

// Crear objeto TinyGPS++
TinyGPSPlus gps;

// Variables globales donde guardaremos los datos limpios
float latitud = 0.0;
float longitud = 0.0;

// Definir pines para comunicación serial con GPS
#define RX_PIN 16
#define TX_PIN 17

// Crear serial para el GPS (Serial2)
HardwareSerial gpsSerial(2);

// Cronómetro para la alerta de error (para que no te inunde la pantalla)
unsigned long tiempoAnteriorErrorGPS = 0;

void setup_gps() {
  // Iniciar comunicación serial con GPS NEO-6M
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  /*Serial.println("========================================");
  Serial.println("GPS u-blox NEO-6M Test");
  Serial.println("========================================");
  Serial.println("Esperando señal GPS...");
  Serial.println();*/
}

void leer_gps() {
  // Leer datos del GPS
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    //Serial.print(c); // Imprime datos crudos para ver si llegan
    gps.encode(c);
  }
  
  // Si hay nueva información de ubicación, mostrarla
  if (gps.location.isUpdated()) {
    latitud = gps.location.lat();
    longitud = gps.location.lng();
    //Solo descomentar si se quiere ver los datos en el serial
    /*
    Serial.println("----------------------------------------");
    Serial.print("Latitud   : ");
    Serial.println(gps.location.lat(), 6);
    
    Serial.print("Longitud  : ");
    Serial.println(gps.location.lng(), 6);
    
    Serial.print("Altitud   : ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.println(" metros");
    } else {
      Serial.println("No disponible");
    }
    
    Serial.print("Satélites : ");
    if (gps.satellites.isValid()) {
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("No disponible");
    }
    
    Serial.print("Velocidad : ");
    if (gps.speed.isValid()) {
      Serial.print(gps.speed.kmph());
      Serial.println(" km/h");
    } else {
      Serial.println("No disponible");
    }
    
    Serial.print("Fecha     : ");
    if (gps.date.isValid()) {
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.println(gps.date.year());
    } else {
      Serial.println("No disponible");
    }
    
    Serial.print("Hora UTC  : ");
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) Serial.print("0");
      Serial.print(gps.time.hour());
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print("0");
      Serial.print(gps.time.minute());
      Serial.print(":");
      if (gps.time.second() < 10) Serial.print("0");
      Serial.println(gps.time.second());
    } else {
      Serial.println("No disponible");
    }
    
    Serial.println("----------------------------------------");
    Serial.println();
    */
  }
  
  // Verificar si han pasado 5 segundos sin datos válidos
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    unsigned long tiempoActual = millis();
    // Avisar del error solo cada 3 segundos para dejar trabajar al resto del robot
    if (tiempoActual - tiempoAnteriorErrorGPS >= 3000) {
      tiempoAnteriorErrorGPS = tiempoActual;
      /*Serial.println("- GPS RX -> ESP32 TX (Pin 17)");
      Serial.println("- GPS TX -> ESP32 RX (Pin 16)");
      Serial.println("- GPS VCC -> 3.3V o 5V");
      Serial.println("- GPS GND -> GND");*/
    }
  }
}