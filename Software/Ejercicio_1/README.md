# Ejercicio 1 - Aviónica Software

Selección razonada del STM32L476RGT6 y configuración inicial de SPI1, I2C1 y SWD mediante STM32CubeMX. Se incluye USART1 como parte de la arquitectura propuesta.

- [Informe](Ejercicio_1.pdf)
- [Proyecto CubeMX y código C](firmware/)
- [Configuración editable de CubeMX](firmware/Faraday_L476_prueb.ioc)

El programa C incorpora una presión ficticia para estimar la ocupación de Flash y RAM de un filtro, una conversión aproximada de presión a altura y un indicador de descenso. El proyecto se compiló con el perfil Debug; no se han conectado sensores reales ni se ha validado el comportamiento en hardware. Los umbrales del indicador son valores de prueba y no constituyen lógica validada para accionar mecanismos de vuelo.

Para abrir la configuración, cargar el archivo `.ioc` en STM32CubeMX. Para compilar mediante CMake se necesita una cadena de herramientas `arm-none-eabi` compatible y las dependencias indicadas en `CMakePresets.json`.

Los archivos bajo `Drivers/` proceden de STM32CubeMX/ST y conservan sus avisos de licencia.
