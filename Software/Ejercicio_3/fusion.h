/* fusion.h: variables compartidas por las funciones de fusion. */

#ifndef FUSION_H
#define FUSION_H

/* El filtro trabaja con 15 componentes de error:
   3 de posicion, 3 de velocidad, 3 de orientacion,
   3 de sesgo del acelerometro y 3 del giroscopio. */
#define FUSION_DIM_ERROR 15U

typedef struct
{
    /* =====================================================
       MOVIMIENTO: prediccion con IMU y correccion con GNSS
       ===================================================== */

    /* Posicion respecto al origen, en metros.
       Orden: norte, este y abajo (NED).
       Los barometros tambien corrigen la componente vertical. */
    float posicion_ned_m[3];

    /* Velocidad en los mismos ejes, en metros por segundo. */
    float velocidad_ned_m_s[3];

    /* =====================================================
       ORIENTACION: giroscopio y referencias disponibles
       ===================================================== */

    /* Cuaternion que transforma del cuerpo al sistema NED.
       Orden: componente escalar, x, y, z.
       El giroscopio propaga la orientacion.
       El magnetometro aporta una referencia de direccion.
       En reposo, el acelerometro aporta la vertical. */
    float q_cuerpo_a_ned[4];

    /* =====================================================
       IMU: errores sistematicos estimados
       ===================================================== */

    /* Sesgo del acelerometro en los ejes del cuerpo, en m/s^2.
       Se resta de la medida antes de predecir el movimiento. */
    float sesgo_acc_m_s2[3];

    /* Sesgo del giroscopio en los ejes del cuerpo, en rad/s.
       Se resta antes de predecir la orientacion. */
    float sesgo_giro_rad_s[3];

    /* =====================================================
       FILTRO: incertidumbre de la estimacion conjunta
       ===================================================== */

    /* Covarianza de los 15 componentes de error.
       La diagonal contiene sus varianzas.
       Los demas elementos representan sus correlaciones. */
    float P[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

} FusionEstado;
/* Intensidades continuas del ruido.
   Se supone igual intensidad en los tres ejes
   e independencia entre las cuatro fuentes.
   Los valores deben ser finitos y no negativos. */
typedef struct
{
    /* Ruido de aceleracion:
       unidades (m/s^2)^2 * s. */
    float q_acc;

    /* Ruido de velocidad angular:
       unidades (rad/s)^2 * s. */
    float q_giro;

    /* Crecimiento de la varianza del sesgo del acelerometro:
       unidades (m/s^2)^2 / s. */
    float q_sesgo_acc;

    /* Crecimiento de la varianza del sesgo del giroscopio:
       unidades (rad/s)^2 / s. */
    float q_sesgo_giro;

} FusionRuido;
/* Inicializa el estimador.
   q_inicial: orientacion cuerpo -> NED, orden escalar, x, y, z.
   sigma_inicial: desviaciones tipicas de los 15 errores.
   Devuelve 1 si se inicializa correctamente y 0 si hay
   parametros invalidos. */
int fusion_inicializar(
    FusionEstado *estado,
    const float q_inicial[4],
    const float sigma_inicial[FUSION_DIM_ERROR]
);
/* Convierte las medidas de la IMU y resta los sesgos.
   Entradas en los ejes del cuerpo: g y grados/s.
   Salidas en los mismos ejes: m/s^2 y rad/s.
   Requiere medidas validas y un estado inicializado. */
void fusion_preparar_imu(
    const FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float fuerza_corregida[3],
    float giro_corregido[3]
);
/* Calcula el cuaternion del giro durante dt segundos.
   giro_rad_s: velocidad angular corregida en el cuerpo.
   dq: cuaternion de salida, orden escalar, x, y, z.
   Devuelve 1 si el calculo es valido y 0 si falla. */
int fusion_incremento_rotacion(
    const float giro_rad_s[3],
    float dt,
    float dq[4]
);
/* Producto de Hamilton: resultado = a multiplicado por b.
   Cuaterniones en orden escalar, x, y, z. */
void fusion_producto_cuaterniones(
    const float a[4],
    const float b[4],
    float resultado[4]
);
/* Normaliza un cuaternion sin cambiar la rotacion representada.
   Devuelve 1 si tiene exito y 0 si el cuaternion es invalido. */
int fusion_normalizar_cuaternion(float q[4]);
/* Obtiene la matriz de rotacion cuerpo -> NED.
   Requiere un cuaternion valido y normalizado.
   Orden del cuaternion: escalar, x, y, z. */
void fusion_matriz_rotacion(
    const float q[4],
    float C[3][3]
);
/* Transforma la fuerza especifica del cuerpo a NED
   y suma la gravedad para obtener la aceleracion. */
void fusion_aceleracion_ned(
    const float q[4],
    const float fuerza_corregida[3],
    float gravedad_m_s2,
    float aceleracion_ned[3]
);
/* Actualiza posicion y velocidad con la aceleracion en NED.
   Devuelve 1 si la operacion es correcta y 0 si se rechaza. */
int fusion_integrar_movimiento(
    FusionEstado *estado,
    const float aceleracion_ned[3],
    float dt
);
/* Prediccion del movimiento y la orientacion con la IMU.
   Todavia no propaga la matriz de incertidumbre P. */
int fusion_predecir_nominal(
    FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float dt,
    float gravedad_m_s2
);
/* Construye S de forma que S * v = u x v.
   Requiere un vector finito y memoria de salida valida. */
void fusion_matriz_producto_vectorial(
    const float u[3],
    float S[3][3]
);
/* Sensibilidad de la aceleracion NED a los errores
   de orientacion local y sesgo del acelerometro.
   Requiere q unitario, datos finitos y salidas distintas. */
void fusion_jacobianos_aceleracion(
    const float q[4],
    const float fuerza_corregida[3],
    float J_orientacion[3][3],
    float J_sesgo_acc[3][3]
);
/* Construye el modelo continuo del estado de error.
   Orden: posicion, velocidad, orientacion,
   sesgo del acelerometro y sesgo del giroscopio.
   Requiere q unitario, entradas finitas y memoria valida.
   La salida A no debe solaparse con las entradas. */
void fusion_matriz_error_continuo(
    const float q[4],
    const float fuerza_corregida[3],
    const float giro_corregido[3],
    float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
);
/* Transicion del error:
   Phi = I + A*dt + 0.5*(A*dt)*(A*dt).
   Devuelve 1 si el calculo termina correctamente.
   Si falla, no modifica Phi. */
int fusion_matriz_transicion(
    const float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    float dt,
    float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
);
/* Intensidad del ruido expresada en el estado de error:
   W = G * Qc * G^T.
   Supone ruido independiente e igual en los tres ejes.
   Devuelve 0 si los parametros son invalidos,
   sin modificar W. */
int fusion_intensidad_ruido_estado(
    const FusionRuido *ruido,
    float W[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
);
/* Covarianza del ruido acumulado durante dt.
   Usa Simpson y la transicion de segundo orden.
   Devuelve 0 si falla, sin modificar Qd. */
int fusion_ruido_discreto(
    const float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    const FusionRuido *ruido,
    float dt,
    float Qd[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
);
/* Propaga P mediante Phi*P*Phi^T + Qd.
   Requiere P y Qd simetricas y semidefinidas positivas.
   Comprueba valores finitos, pero no verifica
   completamente esas propiedades matematicas.
   Si falla, no modifica el estado. */
int fusion_propagar_covarianza(
    FusionEstado *estado,
    const float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    const float Qd[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
);
/* Prediccion conjunta del estado nominal y su covarianza.
   Entradas IMU en ejes del cuerpo: g y grados/s.
   Devuelve 1 si termina correctamente.
   Si falla, devuelve 0 sin modificar el estado. */
int fusion_predecir(
    FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float dt,
    float gravedad_m_s2,
    const FusionRuido *ruido
);
/* Resultado de preparar una correccion escalar. */
typedef struct
{
    float innovacion;
    float varianza_innovacion;
    float nis;  /* Innovacion normalizada al cuadrado. */
    float ganancia[FUSION_DIM_ERROR];

} FusionCorreccionEscalar;

/* Calcula S, K y NIS a partir del estado predicho.
   R es una varianza positiva, no una desviacion tipica.
   Requiere P valida y ruido de medida no correlacionado
   con el error de prediccion.
   No modifica estado.
   Si falla, no modifica correccion. */
int fusion_preparar_correccion_escalar(
    const FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    float innovacion,
    float R,
    FusionCorreccionEscalar *correccion
);
/* Aplica las 15 correcciones al estado nominal.
   Orden: posicion, velocidad, orientacion,
   sesgo acc y sesgo giro.
   La correccion angular se expresa en radianes,
   en ejes del cuerpo, y se aplica a la derecha.
   No modifica P: falta actualizarla y reiniciar el error.
   Si falla, conserva el estado original. */
int fusion_inyectar_error_nominal(
    FusionEstado *estado,
    const float delta[FUSION_DIM_ERROR]
);
/* Actualiza P mediante la forma de Joseph.
   K debe corresponder a la misma P, H y R utilizadas
   al preparar la correccion.
   No modifica el estado nominal.
   Todavia no realiza el ajuste de covarianza
   asociado al reinicio del error de orientacion.
   Si falla, conserva el estado original. */
int fusion_corregir_covarianza_escalar(
    FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    const float K[FUSION_DIM_ERROR],
    float R
);
/* Ajusta P tras inyectar una correccion angular local.
   delta_theta: los mismos tres angulos usados en la inyeccion.
   Utiliza el jacobiano de reinicio de primer orden.
   No modifica el estado nominal.
   Si falla, conserva el estado original. */
int fusion_reiniciar_covarianza(
    FusionEstado *estado,
    const float delta_theta[3]
);
typedef enum
{
    FUSION_CORRECCION_ERROR = -1,
    FUSION_CORRECCION_RECHAZADA = 0,
    FUSION_CORRECCION_APLICADA = 1

} FusionResultadoCorreccion;

/* Correccion escalar completa.
   La innovacion y H deben corresponder al estado recibido.
   R es la varianza de la medida.
   umbral_nis es el limite admitido para r^2/S.
   Solo modifica estado si se aplica toda la correccion. */
FusionResultadoCorreccion fusion_corregir_escalar(
    FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    float innovacion,
    float R,
    float umbral_nis
);
/* Medida GNSS preparada para el estimador.
   Todas las componentes corresponden al mismo instante.
   Posicion y velocidad expresadas en el NED del lanzamiento. */
typedef struct
{
    /* Orden: pN, pE, pD, vN, vE, vD.
       Posiciones en m y velocidades en m/s. */
    float valor[6];

    /* Varianzas: m^2 para posicion y (m/s)^2 para velocidad. */
    float varianza[6];

    /* 1: componente disponible y validada previamente.
       0: no utilizar esta componente. */
    unsigned char usar[6];

} FusionMedidaGnss;

/* Procesa las componentes GNSS secuencialmente.
   Devuelve el numero de componentes aplicadas: 0 a 6.
   Devuelve -1 si hay un error de datos o de calculo;
   en ese caso no modifica el estado original.
   Una componente rechazada por NIS se omite.
   Requiere medidas nuevas y sincronizadas con el estado. */
int fusion_corregir_gnss(
    FusionEstado *estado,
    const FusionMedidaGnss *medida,
    float umbral_nis
);
/* Combina dos alturas mediante su promedio.
   Entradas:
     alturas en m;
     varianzas y covarianza en m^2.

   Requiere lecturas previamente validadas y compatibles
   en tiempo y referencia vertical.

   Las salidas deben ocupar posiciones de memoria distintas.
   Devuelve 1 si tiene exito y 0 si los datos son invalidos.
   Si falla, no modifica las salidas. */
int fusion_combinar_alturas(
    float altura_5C,
    float varianza_5C,
    float altura_5D,
    float varianza_5D,
    float covarianza,
    float *altura_combinada,
    float *varianza_combinada
);
/* Corrige el estimador con una altura relativa al lanzamiento.
   altura_m: metros, positiva hacia arriba.
   varianza_m2: varianza de la altura, en m^2.

   Requiere una medida nueva, validada y correspondiente
   al instante del estado que se corrige.

   Devuelve ERROR, RECHAZADA o APLICADA. */
FusionResultadoCorreccion fusion_corregir_barometro(
    FusionEstado *estado,
    float altura_m,
    float varianza_m2,
    float umbral_nis
);
typedef struct
{
    /* Orden: barometro 5C, barometro 5D. */
    float altura_m[2];
    float varianza_m2[2];

    /* 1: lectura nueva y previamente validada.
       0: lectura no disponible para esta actualizacion. */
    unsigned char usar[2];

} FusionMedidasBarometros;

/* Selecciona o combina las alturas y aplica una correccion.
   Las lecturas utilizadas deben corresponder al instante
   del estado y compartir la referencia vertical.

   umbral_diferencia: compatibilidad entre barometros.
   umbral_nis: compatibilidad con el estimador.

   RECHAZADA tambien indica ausencia de medidas utilizables. */
FusionResultadoCorreccion fusion_corregir_barometros(
    FusionEstado *estado,
    const FusionMedidasBarometros *medidas,
    float covarianza_m2,
    float umbral_diferencia,
    float umbral_nis
);
/* Predice el campo magnetico en los ejes del cuerpo.
   campo_ned: referencia terrestre en NED.
   campo_cuerpo: campo predicho, en las mismas unidades.
   Requiere q valido y normalizado, y datos finitos. */
void fusion_predecir_campo_magnetico(
    const float q[4],
    const float campo_ned[3],
    float campo_cuerpo[3]
);
/* Construye una fila de la matriz de medida magnetica.
   componente: 0 = X, 1 = Y, 2 = Z.
   campo_predicho: campo esperado en los ejes del cuerpo.
   Devuelve 0 si los datos son invalidos, sin modificar H. */
int fusion_fila_medida_magnetica(
    const float campo_predicho[3],
    unsigned int componente,
    float H[FUSION_DIM_ERROR]
);
/* Comprueba la intensidad del campo magnetico.
   Ambos campos y el limite deben usar las mismas unidades.
   Devuelve:
    -1: datos invalidos.
     0: lectura incompatible.
     1: intensidad compatible.
   No comprueba la direccion ni modifica el estimador. */
int fusion_comprobar_intensidad_magnetica(
    const float campo_medido[3],
    const float campo_ned[3],
    float limite_diferencia
);
/* Corrige una componente magnetica: 0 = X, 1 = Y, 2 = Z.
   medida y campo_ned deben usar las mismas unidades.
   varianza: unidades de campo al cuadrado.
   Requiere lectura previamente validada y sincronizada.
   Solo modifica el estado si se aplica la correccion. */
FusionResultadoCorreccion fusion_corregir_componente_magnetica(
    FusionEstado *estado,
    const float campo_ned[3],
    unsigned int componente,
    float medida,
    float varianza,
    float umbral_nis
);
/* Corrige con una lectura magnetica completa.
   Campo medido calibrado y expresado en ejes del cuerpo.
   Campo de referencia expresado en NED.
   Ambos campos y limite_intensidad usan las mismas unidades.
   Las varianzas se expresan en esas unidades al cuadrado.

   Requiere lectura nueva, sin saturacion y sincronizada.
   Supone R diagonal en los ejes del cuerpo.

   Devuelve el numero de componentes aplicadas: 0 a 3.
   Devuelve -1 si hay error, sin modificar el estado.
   Si falla la comprobacion de intensidad, devuelve 0. */
int fusion_corregir_magnetometro(
    FusionEstado *estado,
    const float campo_medido[3],
    const float campo_ned[3],
    const float varianza[3],
    float limite_intensidad,
    float umbral_nis
);
/* Estimador junto con su referencia temporal.
   Inicializar mediante fusion_inicializar_estimador
   antes de procesar muestras. */
/* Estimador y seguimiento temporal de los sensores. */
typedef struct
{
    FusionEstado estado;
    double tiempo_s;

    double ultimo_gnss_s;
    double ultimo_baro_s[2];
    double ultimo_mag_s;

    unsigned char gnss_procesado;
    unsigned char baro_procesado[2];
    unsigned char mag_procesado;

} FusionEstimador;

/* Inicializa el estado y fija su instante inicial.
   Devuelve 1 si tiene exito.
   Devuelve 0 si falla, sin modificar el estimador. */
int fusion_inicializar_estimador(
    FusionEstimador *estimador,
    const float q_inicial[4],
    const float sigma_inicial[FUSION_DIM_ERROR],
    double tiempo_inicial_s
);
/* Predice hasta el instante de una nueva muestra IMU.
   Requiere un estimador previamente inicializado.
   Entradas en ejes del cuerpo: g y grados/s.

   tiempo_muestra_s: mismo origen temporal que el estimador.
   dt_max_s: intervalo maximo permitido, en segundos.

   Devuelve 1 si tiene exito.
   Devuelve 0 si falla, sin modificar estado ni tiempo. */
int fusion_predecir_hasta(
    FusionEstimador *estimador,
    const float aceleracion_g[3],
    const float giro_dps[3],
    double tiempo_muestra_s,
    float dt_max_s,
    float gravedad_m_s2,
    const FusionRuido *ruido
);
/* Procesa una lectura GNSS sincronizada con el estado.
   Requiere un estimador inicializado.

   Devuelve:
   -1: error de datos o de calculo.
    0: ninguna componente aplicada, o lectura omitida
       por no cumplir las condiciones temporales.
   1 a 6: numero de componentes aplicadas.

   Una lectura evaluada sin error queda marcada
   como procesada, aunque todas sus componentes
   sean rechazadas por NIS. */
int fusion_procesar_gnss(
    FusionEstimador *estimador,
    const FusionMedidaGnss *medida,
    double tiempo_medida_s,
    float umbral_nis
);
/* Procesa las lecturas barometricas sincronizadas.
   tiempo_medida_s: instantes del 5C y del 5D.
   Solo se revisan las lecturas marcadas con usar = 1.

   Devuelve ERROR, RECHAZADA o APLICADA.
   RECHAZADA tambien indica que no quedan lecturas
   utilizables tras el control temporal.

   Las lecturas evaluadas sin error se marcan
   como procesadas, incluso si se rechazan. */
FusionResultadoCorreccion fusion_procesar_barometros(
    FusionEstimador *estimador,
    const FusionMedidasBarometros *medidas,
    const double tiempo_medida_s[2],
    float covarianza_m2,
    float umbral_diferencia,
    float umbral_nis
);
/* Procesa una lectura magnetica nueva y sincronizada.
   Requiere un estimador inicializado y una lectura
   calibrada, previamente validada y sin saturacion.

   Campos y limite_intensidad en las mismas unidades.
   Varianzas en unidades de campo al cuadrado.

   Devuelve -1 si hay error; de 0 a 3 componentes aplicadas.
   El 0 tambien indica omision por control temporal.
   Una lectura evaluada sin error queda registrada,
   aunque se rechace por intensidad o por NIS. */
int fusion_procesar_magnetometro(
    FusionEstimador *estimador,
    const float campo_medido[3],
    const float campo_ned[3],
    const float varianza[3],
    double tiempo_medida_s,
    float limite_intensidad,
    float umbral_nis
);
#endif