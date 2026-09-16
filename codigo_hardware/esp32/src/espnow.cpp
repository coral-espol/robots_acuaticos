#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "espnow.h"
#include "gps.h" // Importamos para leer la propia latitud y longitud

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estructura del Payload
typedef struct struct_message {
  int id_robot;
  float lat;
  float lon;
} struct_message;

struct_message myData;

// --- Definición real de las variables globales (las que declaramos en el .h) ---
int id_robot_recibido = 0;
float lat_recibida = 0.0;
float lon_recibida = 0.0;
volatile bool nuevo_dato_enjambre = false;

// Variables para el control de envío propio
unsigned long tiempoAnteriorEspNow = 0;
const unsigned long intervaloEspNow = 4000; // Transmitir cada 4 segundos

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Ocultamos los prints de envío para no ensuciar el JSON de la Raspberry
}

// Callback que se dispara automáticamente cuando OTRO robot habla
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message datosRecibidos;
  memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
  
  // 1. Sobreescribimos las variables globales con lo que acaba de llegar
  id_robot_recibido = datosRecibidos.id_robot;
  lat_recibida = datosRecibidos.lat;
  lon_recibida = datosRecibidos.lon;
  
  // 2. ¡Levantamos la bandera! El main.cpp atrapará esto
  nuevo_dato_enjambre = true;
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  peerInfo.ifidx = WIFI_IF_STA; 
  
  esp_now_add_peer(&peerInfo);
  
  // Generamos el ID propio de este robot
  //randomSeed(analogRead(0));
  //myData.id_robot = random(1, 1000); 
  myData.id_robot = 2;
}

void loop_espnow() {
  unsigned long tiempoActual = millis();

  // Transmisión asíncrona de los PROPIOS datos al enjambre cada 4s
  if (tiempoActual - tiempoAnteriorEspNow >= intervaloEspNow) {
    tiempoAnteriorEspNow = tiempoActual;

    myData.lat = latitud;   // Variables del propio gps.h
    myData.lon = longitud;  

    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}