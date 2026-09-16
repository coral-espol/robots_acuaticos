#ifndef ESPNOW_H
#define ESPNOW_H

// Variables globales para almacenar los datos recibidos de OTRO robot
extern int id_robot_recibido;
extern float lat_recibida;
extern float lon_recibida;

// Bandera que avisa al main.cpp que llegó un nuevo mensaje
extern volatile bool nuevo_dato_enjambre; 

void setup_espnow();
void loop_espnow();

#endif