# Ejercicio 2 - Integración de sensores

- [Informe](Ejercicio_2.pdf)

El proyecto STM32CubeMX y el ejecutable CMake se llaman `Faraday_L476_Ej2_Sensores`.

- [Proyecto CubeMX y código C](firmware/)
- [Configuración editable de CubeMX](firmware/Faraday_L476_Ej2_Sensores.ioc)

El programa identifica y configura dos barómetros LPS22DF por I²C1 (direcciones `0x5C` y `0x5D`). Combina las muestras nuevas disponibles para calcular la altura relativa y actualizar la detección de descenso. Si no hay muestra nueva de presión, conserva el último valor y no actualiza ese cálculo.

Por SPI1 identifica la IMU LSM6DSV320X y adquiere muestras del acelerómetro de bajo rango y del giróscopo. El acelerómetro de alto rango queda configurado, pero sus muestras aún no se adquieren ni se procesan. Por USART1 recibe bytes del GNSS mediante interrupciones y los conserva en un búfer circular; todavía no interpreta tramas ni calcula posición o velocidad.

El código se ha compilado en el proyecto STM32, pero no se ha probado con sensores físicos. La adquisición y la lógica de descenso no están validadas para vuelo. Las transacciones I²C y SPI son bloqueantes; las frecuencias configuradas en los sensores no garantizan por sí mismas que se procese cada muestra.

Para compilar mediante CMake se necesita una cadena de herramientas `arm-none-eabi` compatible. Los archivos bajo `Drivers/` proceden de STM32CubeMX/ST y conservan sus avisos de licencia.
