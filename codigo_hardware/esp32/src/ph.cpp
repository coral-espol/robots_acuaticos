#include "ph.h"
#include <Arduino.h>
#include <Wire.h>


int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temperature;

#define PH_PIN 35

// ---- Calibración ----
const float VOLTAGE_AT_PH7 = 1.82;   // Medir con buffer pH 7 y actualizar
const float SENSITIVITY    = 0.11; // V/pH, ajustar con buffer pH 4 o pH 10


float ph;
float volt;

// ---- Variables para millis() ----
unsigned long previousMillis = 0;  // Almacena la última vez que se leyó el sensor
const long interval = 1000;        // Intervalo de lectura en milisegundos (1 segundo)

void setup_ph() {
    Serial.begin(115200);
    Wire.begin();
}
void leer_ph() {
    unsigned long currentMillis = millis();

  // Comprueba si ya ha pasado 1 segundo (1000 ms) desde la última lectura
  if (currentMillis - previousMillis >= interval) {
    // Guarda el tiempo de esta nueva lectura
    previousMillis = currentMillis;

    for(int i=0;i<10;i++){ 
        buffer_arr[i]=analogRead(PH_PIN);
        delay(30);
    }
    for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
            if(buffer_arr[i]>buffer_arr[j]){
                temperature=buffer_arr[i];
                buffer_arr[i]=buffer_arr[j];
                buffer_arr[j]=temperature;
            }
        }
    }
    avgval=0;
    for(int i=2;i<8;i++){
        avgval+=buffer_arr[i];
        }
    
    volt = (float)avgval * 3.3 / 4096.0 / 6.0;

        // Fórmula con base física clara
        ph = 7.0 + (VOLTAGE_AT_PH7 - volt) / SENSITIVITY;

        //Serial.print("Voltage: "); Serial.println(volt, 4);
        //Serial.println(volt);
        //Serial.print("pH Val: "); Serial.println(ph, 2);
    delay(1000);
    }
}