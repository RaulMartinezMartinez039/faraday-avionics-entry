# Ejercicio personal — Simulación funcional en Proteus

Proyecto académico de Ingeniería Electrónica realizado por **Raúl Martínez Martínez, Miguel Vela García y Julen Lorenzo Moa**. Se incorpora como muestra de integración de electrónica y programación de un microcontrolador.

- [Memoria original](FTS_Proteus.pdf)
- [Captura del circuito en Proteus](figuras/circuito_proteus.jpeg)
- [Diagrama de flujo del planteamiento](figuras/diagrama_flujo.jpeg)

## Objetivo y alcance

El trabajo representa en Proteus una versión didáctica y simplificada de un sistema de terminación de vuelo (FTS). Las órdenes se introducen mediante interruptores y la salida se representa con un motor DC. La documentación describe una simulación funcional, sin ensayos de hardware ni validación para un vehículo real.

También incorpora una PT100 simulada y un servomotor que representa una compuerta de ventilación. La apertura depende de la temperatura introducida manualmente; no se modela la refrigeración que produciría la compuerta.

## Qué se ha integrado

| Bloque | Trabajo realizado |
|---|---|
| Entrada analógica | PT100, puente de Wheatstone y acondicionamiento con amplificadores operacionales; conversión de la lectura ADC a temperatura |
| Microcontrolador | ATmega328P programado mediante registros, con configuración de puertos, ADC, temporizadores e interrupciones |
| Visualización | Temperatura, apertura y estados mediante displays de siete segmentos, registros 4094, multiplexado y LEDs |
| Actuador proporcional | Generación de la señal del servomotor y relación entre temperatura medida y apertura |
| Simulación funcional | Coordinación de las entradas digitales, estados indicados y salida sobre el motor de demostración |

![Circuito completo de la simulación](figuras/circuito_proteus.jpeg)

## Comprobaciones descritas en la memoria

El informe recoge la variación de la temperatura visualizada al modificar la PT100, la indicación de signo negativo, la respuesta proporcional del servomotor y la visualización de estados. Explica asimismo los problemas de integración encontrados con el servo, el multiplexado y la selección de dígitos.

Estas comprobaciones se presentan como las descritas por los autores. El material disponible no incluye una nueva ejecución de Proteus ni registros temporales que permitan verificar cuantitativamente los retardos.

## Correspondencia entre texto y anexo

La memoria original se conserva sin modificaciones. Para interpretar su alcance deben tenerse en cuenta estas discrepancias:

- El texto afirma que existe histéresis térmica de 2 °C, pero la función incluida en el anexo solo compara la temperatura con los límites definidos y no conserva un estado de histéresis.
- La explicación menciona `bloqueo_secuencia`, que no aparece en el programa del anexo. Esa protección no puede considerarse acreditada por el código adjunto.
- Los tiempos asociados a los bucles son aproximados: también incluyen conversiones, cálculos y refresco de displays. No se han aportado medidas temporales de la simulación.
- El diagrama de flujo expresa el planteamiento funcional; el propio informe distingue ese planteamiento de la implementación simplificada.
- El diagrama representa una comprobación de batería y un límite temporal de 5 s; el anexo no incluye esa lectura de batería y el texto describe un límite de unos 10 s. No deben interpretarse como una única especificación verificada.

## Archivos disponibles

Se incluyen la memoria y las dos figuras extraídas del PDF, conservando su resolución original. El código aparece como listado dentro del anexo. No se incluye un fichero fuente reconstruido desde el PDF ni un ejecutable sin verificar.

Para reproducir el proyecto faltan los archivos originales de Proteus, el código fuente y la configuración de compilación utilizada.
