# Ejercicio 3 — Estimación del movimiento y simulación

- [Informe](Ejercicio_3.pdf)
- [Interfaz del estimador](fusion.h) y [implementación](fusion.c)
- [Prueba de integración](prueba_fusion.c)
- [Trayectoria vertical](simulacion.c) y [trayectoria con orientación variable](simulacion_3d.c)

El estimador combina la predicción mediante IMU con correcciones de posición y velocidad GNSS, dos barómetros y magnetómetro. Mantiene un estado nominal con posición, velocidad, cuaternión y sesgos, y una covarianza de los 15 componentes del estado de error. Esta implementación se prueba en ordenador; no está integrada todavía en el firmware de los ejercicios anteriores.

## Compilación y ejecución

Se necesita un compilador C11 y la biblioteca matemática. Desde esta carpeta, con GCC y una terminal compatible con estos comandos:

```sh
mkdir -p build
gcc -std=c11 -O2 -Wall -Wextra -Wpedantic prueba_fusion.c fusion.c -o build/prueba_fusion -lm
gcc -std=c11 -O2 -Wall -Wextra -Wpedantic simulacion.c fusion.c -o build/simulacion -lm
gcc -std=c11 -O2 -Wall -Wextra -Wpedantic simulacion_3d.c fusion.c -o build/simulacion_3d -lm
cd build
./prueba_fusion
./simulacion
./simulacion_3d
```

En Windows con GCC puede añadirse `.exe` al nombre de los ejecutables. Las simulaciones escriben sus CSV en el directorio de ejecución: ejecutarlas desde `build/` conserva los resultados entregados en esta carpeta.

Las tres compilaciones se han comprobado con GCC en Linux sin avisos con las opciones anteriores. La prueba de integración y ambas simulaciones terminan correctamente.

## Resultados y gráficas

Los archivos [resultados_simulacion.csv](resultados_simulacion.csv) y [resultados_simulacion_3d.csv](resultados_simulacion_3d.csv) contienen las ejecuciones originales utilizadas en el informe. Las simulaciones usan `srand(1U)`, pero la secuencia de `rand()` depende de la biblioteca C: ejecutar en otro entorno puede producir muestras de ruido y resultados numéricos diferentes.

- [Gráfica disponible de errores verticales](graficas_simulacion/)
- [Gráficas de la trayectoria con orientación variable](graficas_simulacion_3d/)
- [Conjunto de gráficas 3D en PDF](graficas_simulacion_3d/graficas_simulacion_3d.pdf)

Los errores aditivos se calculan como referencia menos estimación. La altura y la velocidad ascendente son `h = -p_D` y `v_h = -v_D`. El error angular usa el cuaternión relativo `conjugado(q_est) * q_ref`, expresado en los ejes del cuerpo estimado. Las bandas de tres desviaciones típicas son marginales y no constituyen una evaluación estadística con múltiples ejecuciones.

Para volver a generar las gráficas 3D desde el CSV entregado, desde esta carpeta:

```sh
python -m pip install -r requirements.txt
python graficas_simulacion_3d/generar_graficas_3d.py resultados_simulacion_3d.csv build/graficas_3d
```

## Alcance

La primera trayectoria facilita la comprobación del movimiento vertical. La segunda añade desplazamiento horizontal y orientación variable, con masa, empuje y resistencia aerodinámica simplificados. La orientación se prescribe: no se simula la dinámica completa de rotación ni el descenso y recuperación.

Las medidas son sintéticas. Quedan pendientes la calibración y sincronización de sensores físicos, la lectura de alto rango, el procesamiento GNSS, la gestión de retrasos y pérdidas de muestras, y las pruebas de ejecución y memoria en el microcontrolador. Un intervalo de IMU superior al máximo permitido se rechaza; el programa no reconstruye por sí solo ese intervalo perdido.
