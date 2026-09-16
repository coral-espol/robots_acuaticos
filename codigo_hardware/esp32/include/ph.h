#ifndef PH_H
#define PH_H

#include <Arduino.h>
extern float ph;
extern float volt;

// Declaramos las funciones que queremos que sean accesibles desde el main
void setup_ph();
void leer_ph();

#endif