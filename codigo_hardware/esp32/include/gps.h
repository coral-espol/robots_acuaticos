#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
extern float latitud;
extern float longitud;

// Declaramos las funciones que queremos que sean accesibles desde el main
void setup_gps();
void leer_gps();

#endif