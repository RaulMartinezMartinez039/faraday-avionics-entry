# Pruebas de entrada — Faraday Rocketry UPV

Repositorio de mis ejercicios de entrada al departamento de Aviónica.

## Aviónica Software

| Ejercicio | Contenido | Informe |
|---|---|---|
| [Ejercicio 1](Software/Ejercicio_1/README.md) | Selección del STM32, presupuesto energético y de memoria, configuración básica y altura con presión simulada | [PDF](Software/Ejercicio_1/Ejercicio_1.pdf) |
| [Ejercicio 2](Software/Ejercicio_2/README.md) | Selección e integración de sensores: barómetros, lectura parcial de la IMU y recepción de bytes GNSS | [PDF](Software/Ejercicio_2/Ejercicio_2.pdf) |
| [Ejercicio 3](Software/Ejercicio_3/README.md) | Filtro de estado de error, pruebas de integración y dos simulaciones de trayectoria | [PDF](Software/Ejercicio_3/Ejercicio_3.pdf) |

Los ejercicios 1 y 2 incluyen proyectos STM32CubeMX independientes en `firmware/`. El ejercicio 3 contiene una implementación en C que se ejecuta en ordenador, sus pruebas y los resultados utilizados en el informe. Su README explica cómo compilarla y ejecutarla.

El ejercicio 1 utiliza presión simulada. El ejercicio 2 adquiere presión, aceleración de bajo rango y velocidad angular, configura el acelerómetro de alto rango y recibe bytes del GNSS; todavía no procesa el alto rango ni interpreta las tramas GNSS. El filtro del ejercicio 3 se evalúa con medidas sintéticas y aún no está conectado a esos controladores. Los proyectos no se han validado con sensores físicos ni en vuelo.

## Ejercicio personal

[Simulación funcional de un FTS en Proteus](Software/Ejercicio_personal/FTS/README.md), realizada en equipo en la asignatura de Ingeniería Electrónica. Integra una entrada analógica PT100, programación del ATmega328P por registros, visualización y actuadores de demostración. Se incluyen la memoria, las figuras, el paquete original de Proteus, el esquema y el código fuente. El anexo del informe incorpora el código fuente del proyecto; el README recoge el alcance y las limitaciones de la simulación.
