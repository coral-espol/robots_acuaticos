# Simulacion del proyecto de robots acuaticos

Simulacion en ROS 2 Jazzy + Gazebo Harmonic de un enjambre de robots acuaticos de superficie, operando dentro de entornos acuáticos simulados.

Actualmente se han probado pruebas de hasta 20 robots en simultaneo.

## Estado actual del proyecto

La version activa y final de trabajo esta centrada en Model 3:

- `src/swarm_control_MODEL3` (paquete Python `swarm_control_model3`)
- `src/swarm_robot_description_MODEL3`
- `src/swarm_bringup/launch/swarm_flocking_model3.launch.py`
- `src/swarm_bringup/launch/swarm_waypoint_model3.launch.py`
- `src/swarm_worlds/worlds/shrimp_pond_static.sdf`

Los paquetes sin sufijo y `model2` se mantienen solo como legacy y comparativa.

## Algoritmos simulados

Se mantienen dos controladores sobre el mismo modelo fisico:

1. `cmd_vel_swarm`
   Boids 2D: separacion, cohesion, alineacion y geofence. Es una referencia de comportamiento de enjambre, no el algoritmo que corre en hardware real al menos hasta este momento.

2. `waypoint_nav_swarm`
   Navegacion por waypoints + evasion reactiva de colisiones. Esta version replica el comportamiento del algoritmo usado en las Raspberry Pi de los robots fisicos.

## Estructura relevante

- `src/swarm_bringup`: launch files de simulacion, spawn, bridges y logger.
- `src/swarm_control_MODEL3`: logica de control activa para model3.
- `src/swarm_robot_description_MODEL3`: URDF y recursos del robot activo.
- `src/swarm_worlds`: mundos de Gazebo.
- `data/`: salidas generadas por las pruebas, como CSV y summaries.

## Ejecucion rapida

```bash
cd simulacion
colcon build
source install/setup.bash

# Flocking/boids
ros2 launch swarm_bringup swarm_flocking_model3.launch.py

# Navegacion por waypoints
ros2 launch swarm_bringup swarm_waypoint_model3.launch.py
```

## Notas de versión

Este repositorio esta preparado para visualizar el codigo fuente y la configuracion del proyecto.

Se excluyen artefactos generados y configuraciones locales de editor como:

- `build/`, `install/`, `log/`
