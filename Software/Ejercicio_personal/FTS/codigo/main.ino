#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <stdlib.h>
#include <math.h>

/* ============================================================
   1. VARIABLES GENERALES
   ============================================================ */

unsigned int VTemp;
float Vout, Vref, Vpt, RxT, Treal = 0.0;
int T = 0;
int apertura = 0;                              // apertura mostrada en display, 0-99 %
int servoCmd = 0;                              // mando del servo, 0-100 %

uint8_t armready = 0;
uint8_t fireready = 0;
uint8_t tempready = 0;
uint8_t bloqueo_secuencia = 0;                 // Enclavamiento para interruptores fijos

volatile uint8_t actualizar200ms = 0;

/* ============================================================
   2. CONSTANTES DEL PT100 Y UMBRALES
   ============================================================ */

float R17 = 100.0;                             // Ohm, rama superior referencia
float R18 = 100.0;                             // Ohm, rama superior PT100
float R19 = 72.6419;                           // Ohm, rama inferior referencia
float R20 = 56.0;                              // kOhm, ganancia diferencial
float R21 = 10.0;                              // kOhm, ganancia diferencial
float R0  = 100.0;                             // Ohm, PT100 a 0 ºC
float alpha = 0.0039083;                       // coeficiente PT100

#define PT100_SIGN  1.0                        // cambiar a -1.0 si la temperatura va invertida

#define TEMP_INICIO_REFRIGERACION  55.0        // desde aquí empieza a abrir la compuerta
#define TEMP_LIMITE_SERVO          75.0        // a esta temperatura servo al máximo
#define TEMP_LIMITE_SEGURIDAD      75.0        // a esta temperatura se bloquea FIRE
#define TEMP_MINIMA_SEGURIDAD     -40.0        // límite inferior operativo

/* ============================================================
   3. MAPA DE PINES
   ============================================================ */

/* PORTC */
#define LED_ARMED        PC1                   // LED estado ARMED
#define LED_FIRE         PC2                   // LED estado FIRE
#define LED_SAFE         PC3                   // LED estado SAFE
#define DIG1             PC4                   // selección dígito 1 del LEDMPX
#define CLK_4094         PC5                   // reloj común de los 4094

/* PORTD */
#define ARM_IN           PD0                   // entrada ARM, activa en LOW
#define FIRE_IN          PD1                   // entrada FIRE, activa en LOW
#define LED_FALLO_TERM   PD2                   // LED fallo térmico
#define DIG2             PD3                   // selección dígito 2 del LEDMPX
#define OPTO_ARM1        PD4                   // opto ARM 1
#define OPTO_ARM2        PD5                   // opto ARM 2
#define OPTO_FIRE1       PD6                   // opto FIRE 1
#define OPTO_FIRE2       PD7                   // opto FIRE 2

/* PORTB */
#define DATA_TEMP        PB0                   // dato 4094 temperatura
#define SERVO_PWM        PB1                   // salida servo OC1A
#define STB_4094         PB2                   // strobe común 4094
#define SIGN_TEMP        PB3                   // signo negativo temperatura
#define DATA_APERTURA    PB5                   // dato 4094 apertura

/* ============================================================
   4. TABLA 7 SEGMENTOS, CÁTODO COMÚN
   ============================================================ */

unsigned char tabla7seg[10] = {
    0b00111111,                                // 0
    0b00000110,                                // 1
    0b01011011,                                // 2
    0b01001111,                                // 3
    0b01100110,                                // 4
    0b01101101,                                // 5
    0b01111101,                                // 6
    0b00000111,                                // 7
    0b01111111,                                // 8
    0b01101111                                 // 9
};

/* ============================================================
   5. PROTOTIPOS
   ============================================================ */

void espera_us(unsigned int us);
void espera_ms(unsigned int ms);

void actualizar_sistema(void);
void leer_temperatura(void);
void actualizar_tempready(void);
void calcular_apertura_y_servo(void);
void actualizar_led_fallo_termico(void);

void activar_arm(void);
void desactivar_arm(void);
void activar_fire(void);
void desactivar_fire(void);

void estado_safe(void);
void estado_armed(void);
void estado_fire(void);

void enviar_4094_doble(unsigned char datoTemp, unsigned char datoApertura);
void display_apertura(int dato);
void apagar_digitos(void);
void encender_dig1(void);
void encender_dig2(void);

void pulso_servo(int mando);
void servicio_visualizacion(void);

/* ============================================================
   6. SETUP
   ============================================================ */

void setup() {
    DDRC  = (1 << LED_ARMED) | (1 << LED_FIRE) | (1 << LED_SAFE) |
            (1 << DIG1) | (1 << CLK_4094);     // PC0 ADC; PC1-PC5 salidas usadas
    PORTC = 0x00;                              // salidas inicialmente a LOW

    DDRD  = (1 << LED_FALLO_TERM) | (1 << DIG2) |
            (1 << OPTO_ARM1) | (1 << OPTO_ARM2) |
            (1 << OPTO_FIRE1) | (1 << OPTO_FIRE2); // PD0/PD1 entradas; resto salidas
    PORTD = (1 << ARM_IN) | (1 << FIRE_IN);    // pull-up ARM/FIRE, activos en LOW

    DDRB  = (1 << DATA_TEMP) | (1 << SERVO_PWM) |
            (1 << STB_4094) | (1 << SIGN_TEMP) |
            (1 << DATA_APERTURA);              // PB0,PB1,PB2,PB3,PB5 salidas
    PORTB = 0x00;                              // salidas inicialmente a LOW

    ADCSRA = 0b10000111;                       // ADC ON, prescaler 128
    ADMUX  = 0b01000000;                       // AVCC como referencia, ADC0

    TCCR1A = (1 << COM1A1);                    // Timer1 PWM no inversor en OC1A
    TCCR1B = (1 << WGM13) | (1 << CS11);       // modo 8, prescaler 8
    ICR1   = 20000;                            // periodo 20 ms aprox. para servo

    TCCR2A = 0x00;                             // Timer2 modo normal
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
    TIMSK2 |= (1 << TOIE2);                    // interrupción overflow Timer2

    desactivar_arm();                          // optos ARM apagados
    desactivar_fire();                         // optos FIRE apagados
    estado_safe();                             // estado inicial SAFE
    apagar_digitos();                          // multiplexado apagado al inicio

    actualizar_sistema();                      // primera lectura del sistema
    pulso_servo(0);                            // servo cerrado al inicio

    sei();                                     // habilita interrupciones globales
}

/* ============================================================
   7. LOOP PRINCIPAL (BLINDADO PARA INTERRUPTORES Y TEMPERATURA)
   ============================================================ */

void loop() {
    unsigned int i, j;

    if (actualizar200ms) {                     // actualización periódica
        actualizar200ms = 0;
        actualizar_sistema();
    }

    servicio_visualizacion();                  // refresco continuo del display

    // --- 1. LÓGICA DE RESETEO DE SEGURIDAD ---
    // Si la palanca ARM se baja físicamente (OFF), limpiamos el bloqueo
    if ((PIND & (1 << ARM_IN)) != 0) {
        bloqueo_secuencia = 0;
    }

    // --- 2. SECUENCIA DE LANZAMIENTO ---
    // Solo inicia si ARM se pone en ON y NO estamos bloqueados
    if (((PIND & (1 << ARM_IN)) == 0) && (bloqueo_secuencia == 0)) {
        actualizar_sistema();

        // REGLA ANTI-FUEGO PREMATURO: Si FIRE ya está subido, bloqueamos.
        if ((PIND & (1 << FIRE_IN)) == 0) {
            bloqueo_secuencia = 1;
            return; // Aborta esta vuelta del loop
        }

        if (tempready == 1) {                  
            armready  = 1;
            fireready = 0;

            activar_arm();                     
            estado_armed();                    

            for (i = 0; i < 10000; i++) {      // Ventana de 10 s aprox.
                actualizar_sistema();
                servicio_visualizacion();

                // ABORTO INSTANTÁNEO: El operador baja la palanca ARM
                if ((PIND & (1 << ARM_IN)) != 0) {
                    break; 
                }

                // ABORTO TÉRMICO
                if (tempready == 0) {
                    break; 
                }

                // INTENTO DE DISPARO
                if ((tempready == 1) &&
                    (armready == 1) &&
                    ((PIND & (1 << FIRE_IN)) == 0) &&
                    (fireready == 0)) {
                    fireready = 1;             
                }

                if (fireready == 1) {
                    activar_fire();            
                    estado_fire();             

                    for (j = 0; j < 500; j++) { // 500ms de fuego aprox.
                        actualizar_sistema();
                        servicio_visualizacion();
                        espera_ms(1);          
                    }

                    desactivar_fire();         
                    fireready = 0;
                    
                    // Fuego completado: Forzamos bloqueo hasta que se baje ARM
                    bloqueo_secuencia = 1; 
                    break;
                }

                espera_ms(1);
            }

            // Si salimos del bucle (por tiempo, disparo, o aborto), bloqueamos y limpiamos
            bloqueo_secuencia = 1; 
            armready  = 0;
            fireready = 0;

            desactivar_arm();                  
            desactivar_fire();                 
            estado_safe();                     
        }
    }
}

/* ============================================================
   8. TIMER2: FLAG DE ACTUALIZACIÓN CADA ~200 ms
   ============================================================ */

ISR(TIMER2_OVF_vect) {
    static uint8_t contador200ms = 0;

    contador200ms++;                           // cuenta overflows de Timer2

    if (contador200ms >= 12) {                 // 12 overflows ˜ 200 ms
        actualizar200ms = 1;                   // pide actualización en loop
        contador200ms = 0;
    }
}

/* ============================================================
   9. RETARDOS
   ============================================================ */

void espera_us(unsigned int us) {
    while (us > 0) {
        _delay_loop_2(4);                      // aprox. 1 us a 16 MHz
        us--;
    }
}

void espera_ms(unsigned int ms) {
    while (ms > 0) {
        _delay_loop_2(4000);                   // aprox. 1 ms a 16 MHz
        ms--;
    }
}

/* ============================================================
   10. ACTUALIZACIÓN GENERAL DEL SISTEMA
   ============================================================ */

void actualizar_sistema(void) {
    leer_temperatura();                        // calcula Treal y T
    actualizar_tempready();                    // comprueba rango operativo
    calcular_apertura_y_servo();               // calcula apertura y servo
    actualizar_led_fallo_termico();            // LED fallo térmico
}

/* ============================================================
   11. LECTURA Y CÁLCULO DE TEMPERATURA
   ============================================================ */

void leer_temperatura(void) {
    float G;

    ADMUX = 0b01000000;                        // selecciona ADC0
    ADCSRA |= (1 << ADSC);                     // inicia conversión ADC
    while ((ADCSRA & (1 << ADSC)) != 0);       // espera fin de conversión

    VTemp = ADC;                               // lectura ADC 0-1023
    Vout  = 5.0 * VTemp / 1023.0;              // tensión del acondicionador

    G    = R20 / R21;                          // ganancia del amplificador
    Vref = 5.0 * R19 / (R17 + R19);            // tensión de referencia del puente
    Vpt  = Vref + PT100_SIGN * (Vout / G);     // tensión equivalente del PT100

    if (Vpt < 0.01) Vpt = 0.01;                // evita división por cero
    if (Vpt > 4.99) Vpt = 4.99;                // limita al rango físico

    RxT   = R18 * Vpt / (5.0 - Vpt);           // resistencia equivalente PT100
    Treal = (RxT - R0) / (R0 * alpha);         // temperatura calculada
    T     = (int)(Treal);                      // temperatura entera para display
}

/* ============================================================
   12. CONTROL TÉRMICO (CON HISTÉRESIS)
   ============================================================ */

void actualizar_tempready(void) {
    if (tempready == 1) {
        // Si el sistema está operativo, comprobamos si sobrepasa los límites estrictos
        if ((Treal < TEMP_MINIMA_SEGURIDAD) || (Treal >= TEMP_LIMITE_SEGURIDAD)) {
            tempready = 0;
            fireready = 0;
            desactivar_fire();
        }
    } else {
        // Histéresis: Si estaba en fallo térmico, necesita un margen de 2 grados para resetearse
        if ((Treal >= TEMP_MINIMA_SEGURIDAD + 2.0) && (Treal < TEMP_LIMITE_SEGURIDAD - 2.0)) {
            tempready = 1;
        }
    }
}

void calcular_apertura_y_servo(void) {
    if (Treal <= TEMP_INICIO_REFRIGERACION) {
        apertura = 0;                          // compuerta cerrada
        servoCmd = 0;                          // servo mínimo
    }
    else if (Treal < TEMP_LIMITE_SERVO) {
        apertura = (int)((Treal - TEMP_INICIO_REFRIGERACION) * 99.0 /
                   (TEMP_LIMITE_SERVO - TEMP_INICIO_REFRIGERACION));

        servoCmd = (int)((Treal - TEMP_INICIO_REFRIGERACION) * 100.0 /
                   (TEMP_LIMITE_SERVO - TEMP_INICIO_REFRIGERACION));
    }
    else {
        apertura = 99;                         // apertura máxima mostrada
        servoCmd = 100;                        // servo máximo
    }

    if (apertura < 0) apertura = 0;             // saturación inferior display
    if (apertura > 99) apertura = 99;           // saturación superior display

    if (servoCmd < 0) servoCmd = 0;             // saturación inferior servo
    if (servoCmd > 100) servoCmd = 100;         // saturación superior servo

    pulso_servo(servoCmd);                      // actualiza PWM servo
}

void actualizar_led_fallo_termico(void) {
    // Si la temperatura no está lista, encendemos la luz de fallo. Sincronización perfecta.
    if (tempready == 0) {
        PORTD |=  (1 << LED_FALLO_TERM);        // LED fallo térmico ON
    } else {
        PORTD &= ~(1 << LED_FALLO_TERM);        // LED fallo térmico OFF
    }
}

/* ============================================================
   13. ARM / FIRE
   ============================================================ */

void activar_arm(void) {
    PORTD |= (1 << OPTO_ARM1) | (1 << OPTO_ARM2); // activa optos ARM
}

void desactivar_arm(void) {
    PORTD &= ~((1 << OPTO_ARM1) | (1 << OPTO_ARM2)); // apaga optos ARM
}

void activar_fire(void) {
    PORTD |= (1 << OPTO_FIRE1) | (1 << OPTO_FIRE2); // activa optos FIRE
}

void desactivar_fire(void) {
    PORTD &= ~((1 << OPTO_FIRE1) | (1 << OPTO_FIRE2)); // apaga optos FIRE
}

/* ============================================================
   14. LEDS DE ESTADO
   ============================================================ */

void estado_safe(void) {
    PORTC &= ~(1 << LED_ARMED);                // ARMED OFF
    PORTC &= ~(1 << LED_FIRE);                 // FIRE OFF
    PORTC |=  (1 << LED_SAFE);                 // SAFE ON
}

void estado_armed(void) {
    PORTC |=  (1 << LED_ARMED);                // ARMED ON
    PORTC &= ~(1 << LED_FIRE);                 // FIRE OFF
    PORTC &= ~(1 << LED_SAFE);                 // SAFE OFF
}

void estado_fire(void) {
    PORTC |=  (1 << LED_ARMED);                // ARMED ON
    PORTC |=  (1 << LED_FIRE);                 // FIRE ON
    PORTC &= ~(1 << LED_SAFE);                 // SAFE OFF
}

/* ============================================================
   15. ENVÍO A 4094
   ============================================================ */

void enviar_4094_doble(unsigned char datoTemp, unsigned char datoApertura) {
    unsigned char i;

    for (i = 0; i < 8; i++) {
        if (datoTemp & 0x01) PORTB |=  (1 << DATA_TEMP);      // bit temperatura
        else                 PORTB &= ~(1 << DATA_TEMP);

        if (datoApertura & 0x80) PORTB |=  (1 << DATA_APERTURA); // bit apertura
        else                     PORTB &= ~(1 << DATA_APERTURA);

        espera_us(2);

        PORTC |=  (1 << CLK_4094);             // flanco subida CLK
        espera_us(2);
        PORTC &= ~(1 << CLK_4094);             // flanco bajada CLK
        espera_us(2);

        datoTemp     = datoTemp / 2;           // temperatura LSB primero
        datoApertura = datoApertura * 2;       // apertura MSB primero
    }

    PORTB |=  (1 << STB_4094);                 // STROBE activa salida
    espera_us(2);
    PORTB &= ~(1 << STB_4094);
    espera_us(2);
}

/* ============================================================
   16. DISPLAY MULTIPLEXADO DE APERTURA
   ============================================================ */

void apagar_digitos(void) {
    PORTC &= ~(1 << DIG1);                     // DIG1 OFF
    PORTD &= ~(1 << DIG2);                     // DIG2 OFF
}

void encender_dig1(void) {
    PORTC |= (1 << DIG1);                      // DIG1 ON
}

void encender_dig2(void) {
    PORTD |= (1 << DIG2);                      // DIG2 ON
}

void display_apertura(int dato) {
    unsigned char dec, uni, datoTempBCD;
    int tempAbs;

    if (dato < 0)  dato = 0;                   // evita negativos
    if (dato > 99) dato = 99;                  // limita a 2 dígitos

    dec = dato / 10;                           // decenas
    uni = dato % 10;                           // unidades

    if (T < 0) {
        PORTB |= (1 << SIGN_TEMP);             // signo negativo ON
        tempAbs = abs(T);
    } else {
        PORTB &= ~(1 << SIGN_TEMP);            // signo negativo OFF
        tempAbs = T;
    }

    if (tempAbs > 99) tempAbs = 99;            // temperatura limitada a 2 dígitos

    datoTempBCD = ((tempAbs / 10) << 4) + (tempAbs % 10); // temperatura en BCD

    apagar_digitos();                          // apaga dígitos antes de cambiar segmentos
    espera_us(200);

    enviar_4094_doble(datoTempBCD, tabla7seg[dec]); // carga decenas apertura
    encender_dig1();                           // enciende primer dígito
    espera_ms(4);

    apagar_digitos();                          // apaga antes de cambiar al otro
    espera_us(200);

    enviar_4094_doble(datoTempBCD, tabla7seg[uni]); // carga unidades apertura
    encender_dig2();                           // enciende segundo dígito
    espera_ms(4);

    apagar_digitos();                          // evita ghosting
}

/* ============================================================
   17. SERVO
   ============================================================ */

void pulso_servo(int mando) {
    unsigned int us;

    if (mando < 0) mando = 0;                  // mínimo servo
    if (mando > 100) mando = 100;              // máximo servo

    us = 1000 + ((unsigned long)mando * 556) / 100; // pulso calibrado del servo

    cli();                                     // evita escritura interrumpida de OCR1A
    OCR1A = us;                                // actualiza posición servo
    sei();                                     // reactiva interrupciones
}

/* ============================================================
   18. SERVICIO DE VISUALIZACIÓN
   ============================================================ */

void servicio_visualizacion(void) {
    display_apertura(apertura);                // refresca display de apertura
}