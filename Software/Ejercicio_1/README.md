# Ejercicio 1 - Aviónica Software

Selección razonada del STM32, configuración inicial de interfaces mediante STM32CubeMX y prueba del cálculo de altura con presión simulada.

- [Informe](Ejercicio_1.pdf)
- [Proyecto CubeMX y código C](firmware/)
- [Configuración editable de CubeMX](firmware/Faraday_L476_Ej1_Simulacion.ioc)

El proyecto y el ejecutable CMake se llaman `Faraday_L476_Ej1_Simulacion`. El programa incorpora una presión ficticia para estimar la ocupación de Flash y RAM de un filtro, una conversión aproximada de presión a altura y un indicador de descenso. No se han conectado sensores reales ni se ha validado el comportamiento en hardware. Los umbrales del indicador son valores de prueba y no constituyen lógica validada para accionar mecanismos de vuelo.

Para abrir la configuración, cargar el archivo `.ioc` en STM32CubeMX. Para compilar mediante CMake se necesita una cadena de herramientas `arm-none-eabi` compatible y las dependencias indicadas en `CMakePresets.json`.

Los archivos bajo `Drivers/` proceden de STM32CubeMX/ST y conservan sus avisos de licencia.

El PDF es la versión de entrega actual. La fuente LaTeX conservada en `historico/` corresponde a una versión anterior y no permite reconstruir este PDF.

La comparación de memoria del informe utiliza las cifras documentadas de los ejercicios 1 y 2. El cuadro 6 toma como referencia parcial el ejercicio 2 (27 048 B de Flash y 2 736 B de RAM, incluidas las reservas de heap y pila). Solo se compiló para el STM32L476; los porcentajes de los otros candidatos son ilustrativos. La memoria, la pila y los tiempos del filtro del ejercicio 3 quedan pendientes de evaluación en STM32: las pruebas en ordenador no validan esos recursos ni el funcionamiento del sistema completo.
