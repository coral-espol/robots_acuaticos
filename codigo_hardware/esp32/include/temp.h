#ifndef TEMP_H
#define TEMP_H

#include <Arduino.h>
extern float temp;

// Declaramos las funciones que queremos que sean accesibles desde el main
void setup_temp();
void leer_temp();

#endif