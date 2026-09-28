# Pruebas de entrada — Faraday Rocketry UPV

Repositorio de mis ejercicios de entrada al departamento de Aviónica.

## Aviónica Software

| Ejercicio | Contenido |
|---|---|
| [Ejercicio 1](Software/Ejercicio_1/README.md) | Selección del STM32, configuración básica y prueba de altura con presión simulada |
| [Ejercicio 2](Software/Ejercicio_2/README.md) | Integración de barómetros, IMU y recepción de bytes GNSS |

El [informe del ejercicio 1](Software/Ejercicio_1/Ejercicio_1.pdf) recoge la elección del microcontrolador y la configuración inicial. El [informe del ejercicio 2](Software/Ejercicio_2/Ejercicio_2.pdf) documenta la integración de los dispositivos y sus límites. Cada ejercicio incluye su proyecto STM32CubeMX y su código en un directorio `firmware/` independiente.

El ejercicio 1 utiliza presión simulada. El ejercicio 2 utiliza las lecturas disponibles de dos LPS22DF para el cálculo de altura; también incorpora la lectura del acelerómetro de bajo rango y el giróscopo de la IMU, configura el canal de alto rango y recibe bytes del GNSS. El proyecto del ejercicio 2 se ha compilado, pero no se ha probado con sensores físicos ni constituye software de vuelo validado.
