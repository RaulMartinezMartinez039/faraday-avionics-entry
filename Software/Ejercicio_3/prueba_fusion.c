#include <stdio.h>
#include "fusion.h"

int main(void)
{
    FusionEstado estado;

    /* ESCENARIO DE PRUEBA:
       cuerpo alineado con NED.
       Esta orientacion no representa automaticamente
       un cohete vertical en la plataforma. */
    const float q_inicial[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    /* Valores ilustrativos para probar la inicializacion.
       Todavia no son parametros calibrados de los sensores. */
    const float sigma_inicial[FUSION_DIM_ERROR] = {
        2.0f, 2.0f, 2.0f,       /* Posicion: m */
        0.1f, 0.1f, 0.1f,       /* Velocidad: m/s */
        0.05f, 0.05f, 0.05f,    /* Orientacion: rad */
        0.2f, 0.2f, 0.2f,       /* Sesgo acelerometro: m/s^2 */
        0.01f, 0.01f, 0.01f     /* Sesgo giroscopio: rad/s */
    };

    if (!fusion_inicializar(&estado, q_inicial, sigma_inicial))
    {
        printf("Error en la inicializacion.\n");
        return 1;
    }

    printf("Inicializacion correcta.\n");
    printf("Componente escalar del cuaternion: %.3f\n",
           estado.q_cuerpo_a_ned[0]);
    printf("Varianza inicial de posicion norte: %.3f m^2\n",
           estado.P[0][0]);
    /* PRUEBA DE PREPARACION DE LA IMU:
   cuerpo alineado con NED y en reposo.
   El acelerometro ideal indica -1 g en el eje abajo. */
const float aceleracion_g[3] = {0.0f, 0.0f, -1.0f};
const float giro_dps[3] = {0.0f, 0.0f, 90.0f};

float fuerza_corregida[3];
float giro_corregido[3];

fusion_preparar_imu(
    &estado,
    aceleracion_g,
    giro_dps,
    fuerza_corregida,
    giro_corregido
);

printf("Fuerza especifica Z: %.5f m/s^2\n",
       fuerza_corregida[2]);
printf("Velocidad angular Z: %.5f rad/s\n",
       giro_corregido[2]);
    /* PRUEBA: giro de 90 grados alrededor de Z en un segundo.
   Utilizamos la velocidad angular ya convertida y corregida. */
float dq[4];

if (!fusion_incremento_rotacion(giro_corregido, 1.0f, dq))
{
    printf("Error al calcular el incremento de giro.\n");
    return 1;
}

printf("Cuaternion del incremento: %.5f %.5f %.5f %.5f\n",
       dq[0], dq[1], dq[2], dq[3]);     
    /* PRUEBA: dos incrementos consecutivos de 90 grados
   alrededor del mismo eje deben sumar 180 grados. */
float q_90[4];
float q_180[4];

fusion_producto_cuaterniones(
    estado.q_cuerpo_a_ned, dq, q_90
);

fusion_producto_cuaterniones(
    q_90, dq, q_180
);

printf("Orientacion tras 90 grados: %.5f %.5f %.5f %.5f\n",
       q_90[0], q_90[1], q_90[2], q_90[3]);

printf("Orientacion tras 180 grados: %.5f %.5f %.5f %.5f\n",
       q_180[0], q_180[1], q_180[2], q_180[3]);
    /* PRUEBA: un cuaternion escalado debe recuperar norma uno. */
float q_escalado[4] = {2.0f, 0.0f, 0.0f, 0.0f};

if (!fusion_normalizar_cuaternion(q_escalado))
{
    printf("Error al normalizar el cuaternion.\n");
    return 1;
}

printf("Cuaternion normalizado: %.5f %.5f %.5f %.5f\n",
       q_escalado[0], q_escalado[1],
       q_escalado[2], q_escalado[3]);

/* Un cuaternion completamente nulo debe rechazarse. */
float q_nulo[4] = {0.0f, 0.0f, 0.0f, 0.0f};

if (fusion_normalizar_cuaternion(q_nulo))
{
    printf("Error: se ha aceptado un cuaternion nulo.\n");
    return 1;
}

printf("Cuaternion nulo rechazado correctamente.\n");
/* PRUEBA: transformacion del eje X tras girar 90 grados.
   Antes de construir la matriz, normalizamos la orientacion. */
if (!fusion_normalizar_cuaternion(q_90))
{
    printf("Error: orientacion de prueba invalida.\n");
    return 1;
}

float C[3][3];
fusion_matriz_rotacion(q_90, C);

/* Para el vector del cuerpo [1, 0, 0],
   el resultado es la primera columna de C. */
printf("Eje X del cuerpo en NED: %.5f %.5f %.5f\n",
       C[0][0], C[1][0], C[2][0]);   
    /* PRUEBA: vehiculo en reposo, con sus ejes alineados con NED.
   La fuerza especifica apunta hacia arriba: -g en el eje Z. */
float q_reposo[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float fuerza_reposo[3] = {0.0f, 0.0f, -9.80665f};
float aceleracion_reposo[3];

fusion_aceleracion_ned(
    q_reposo,
    fuerza_reposo,
    9.80665f,
    aceleracion_reposo
);

printf(
    "Aceleracion en reposo NED: %.5f, %.5f, %.5f m/s^2\n",
    aceleracion_reposo[0],
    aceleracion_reposo[1],
    aceleracion_reposo[2]
);        
/* PRUEBA INDEPENDIENTE:
   partir del origen y del reposo;
   acelerar hacia el norte a 2 m/s^2 durante 1 segundo. */
FusionEstado prueba_movimiento = {0};
float aceleracion_prueba[3] = {2.0f, 0.0f, 0.0f};

if (!fusion_integrar_movimiento(
        &prueba_movimiento, aceleracion_prueba, 1.0f))
{
    printf("Error al integrar el movimiento.\n");
    return 1;
}

printf(
    "Movimiento: posicion norte = %.3f m; "
    "velocidad norte = %.3f m/s\n",
    prueba_movimiento.posicion_ned_m[0],
    prueba_movimiento.velocidad_ned_m_s[0]
);
/* PRUEBA CONJUNTA: un segundo en reposo.
   100 intervalos de 0,01 s, sin giro y con ejes alineados con NED. */
FusionEstado prueba_prediccion;

float q_inicial_prueba[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float acc_reposo_g[3] = {0.0f, 0.0f, -1.0f};
float giro_reposo_dps[3] = {0.0f, 0.0f, 0.0f};

if (!fusion_inicializar(
        &prueba_prediccion, q_inicial_prueba, sigma_inicial))
{
    printf("Error al inicializar la prueba de prediccion.\n");
    return 1;
}

for (unsigned int i = 0U; i < 100U; ++i)
{
    if (!fusion_predecir_nominal(
            &prueba_prediccion,
            acc_reposo_g,
            giro_reposo_dps,
            0.01f,
            9.80665f))
    {
        printf("Error en la prediccion con IMU.\n");
        return 1;
    }
}

printf(
    "Prediccion en reposo: posicion abajo = %.5f m; "
    "velocidad abajo = %.5f m/s\n",
    prueba_prediccion.posicion_ned_m[2],
    prueba_prediccion.velocidad_ned_m_s[2]
);
/* PRUEBA: eje X por eje Y debe dar eje Z. */
const float eje_x[3] = {1.0f, 0.0f, 0.0f};
const float eje_y[3] = {0.0f, 1.0f, 0.0f};

float S_producto[3][3];
float producto_vectorial[3];

fusion_matriz_producto_vectorial(eje_x, S_producto);

for (unsigned int i = 0U; i < 3U; ++i)
{
    producto_vectorial[i] =
        S_producto[i][0] * eje_y[0]
      + S_producto[i][1] * eje_y[1]
      + S_producto[i][2] * eje_y[2];
}

printf(
    "Producto X por Y: %.3f %.3f %.3f\n",
    producto_vectorial[0],
    producto_vectorial[1],
    producto_vectorial[2]
);
/* PRUEBA: sensibilidades con cuerpo alineado con NED
   y fuerza especifica [0, 0, -g]. */
float J_orientacion_prueba[3][3];
float J_sesgo_prueba[3][3];

fusion_jacobianos_aceleracion(
    q_reposo,
    fuerza_reposo,
    J_orientacion_prueba,
    J_sesgo_prueba
);

printf(
    "J orientacion: N/y = %.5f; E/x = %.5f\n",
    J_orientacion_prueba[0][1],
    J_orientacion_prueba[1][0]
);

printf(
    "J sesgo: diagonal = %.1f %.1f %.1f\n",
    J_sesgo_prueba[0][0],
    J_sesgo_prueba[1][1],
    J_sesgo_prueba[2][2]
);
/* PRUEBA: bloques del modelo continuo en reposo. */
float A_prueba[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

fusion_matriz_error_continuo(
    q_reposo,
    fuerza_reposo,
    giro_reposo_dps,
    A_prueba
);

/* giro_reposo_dps contiene solo ceros:
   en este caso concreto coincide con cero rad/s. */

printf(
    "A: pN/vN = %.1f; vN/thetaY = %.5f\n",
    A_prueba[0][3],
    A_prueba[3][7]
);

printf(
    "A: vN/baX = %.1f; thetaX/bgX = %.1f\n",
    A_prueba[3][9],
    A_prueba[6][12]
);
/* PRUEBA: transicion durante 0.01 segundos. */
float Phi_prueba[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

if (!fusion_matriz_transicion((const float (*)[FUSION_DIM_ERROR])A_prueba, 0.01f, Phi_prueba))
{
    printf("Error al calcular la matriz de transicion.\n");
    return 1;
}

printf(
    "Phi: pN/pN = %.3f; pN/vN = %.5f\n",
    Phi_prueba[0][0],
    Phi_prueba[0][3]
);

printf(
    "Phi: vN/baX = %.5f; pN/baX = %.7f\n",
    Phi_prueba[3][9],
    Phi_prueba[0][9]
);
/* PARAMETROS SINTETICOS:
   solo para comprobar los calculos del filtro.
   No representan valores calibrados de nuestra IMU. */
const FusionRuido ruido_prueba = {
    .q_acc = 0.01f,
    .q_giro = 0.0001f,
    .q_sesgo_acc = 0.000001f,
    .q_sesgo_giro = 0.00000001f
};

printf(
    "Varianza anadida al sesgo acc en 1 s: %.8f\n",
    ruido_prueba.q_sesgo_acc * 1.0f
);
/* PRUEBA: colocar cada intensidad en su grupo de errores. */
float W_prueba[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

if (!fusion_intensidad_ruido_estado(
        &ruido_prueba, W_prueba))
{
    printf("Error en los parametros de ruido.\n");
    return 1;
}

printf(
    "W: posicion = %.6f; velocidad = %.6f\n",
    W_prueba[0][0],
    W_prueba[3][3]
);

printf(
    "W: orientacion = %.8f; sesgo acc = %.8f; "
    "sesgo giro = %.8f\n",
    W_prueba[6][6],
    W_prueba[9][9],
    W_prueba[12][12]
);
/* PRUEBA AISLADA:
   el error de velocidad se integra en posicion.
   Solo introducimos ruido del acelerometro. */
float A_ruido_prueba[FUSION_DIM_ERROR][FUSION_DIM_ERROR] = {0};

for (unsigned int i = 0U; i < 3U; ++i)
{
    A_ruido_prueba[i][3U + i] = 1.0f;
}

const FusionRuido ruido_solo_acc = {
    .q_acc = 0.01f,
    .q_giro = 0.0f,
    .q_sesgo_acc = 0.0f,
    .q_sesgo_giro = 0.0f
};

float Qd_prueba[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

if (!fusion_ruido_discreto(
        (const float (*)[FUSION_DIM_ERROR])A_ruido_prueba,
        &ruido_solo_acc,
        1.0f,
        Qd_prueba))
{
    printf("Error al calcular el ruido discreto.\n");
    return 1;
}

printf(
    "Qd: pN/pN = %.8f; pN/vN = %.8f; vN/vN = %.8f\n",
    Qd_prueba[0][0],
    Qd_prueba[0][3],
    Qd_prueba[3][3]
);
/* PRUEBA: propagar una covarianza inicial conocida
   con el modelo simplificado del ejemplo anterior. */
FusionEstado prueba_covarianza;

if (!fusion_inicializar(
        &prueba_covarianza, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prueba de P.\n");
    return 1;
}

float Phi_covarianza[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

if (!fusion_matriz_transicion(
        (const float (*)[FUSION_DIM_ERROR])A_ruido_prueba, 1.0f, Phi_covarianza))
{
    printf("Error al calcular la transicion de prueba.\n");
    return 1;
}

if (!fusion_propagar_covarianza(
        &prueba_covarianza, (const float (*)[FUSION_DIM_ERROR])Phi_covarianza, (const float (*)[FUSION_DIM_ERROR])Qd_prueba))
{
    printf("Error al propagar la covarianza.\n");
    return 1;
}

printf(
    "P: varianza pN = %.6f; varianza vN = %.6f; "
    "covarianza pN/vN = %.6f\n",
    prueba_covarianza.P[0][0],
    prueba_covarianza.P[3][3],
    prueba_covarianza.P[0][3]
);
/* PRUEBA CONJUNTA:
   reposo ideal durante un segundo, con propagacion de P. */
FusionEstado prueba_completa;

if (!fusion_inicializar(
        &prueba_completa, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prediccion completa.\n");
    return 1;
}

float varianza_vertical_inicial = prueba_completa.P[2][2];

for (unsigned int i = 0U; i < 100U; ++i)
{
    if (!fusion_predecir(
            &prueba_completa,
            acc_reposo_g,
            giro_reposo_dps,
            0.01f,
            9.80665f,
            &ruido_prueba))
    {
        printf("Error en la prediccion completa.\n");
        return 1;
    }
}

printf(
    "Prediccion completa: pD = %.5f m; vD = %.5f m/s\n",
    prueba_completa.posicion_ned_m[2],
    prueba_completa.velocidad_ned_m_s[2]
);

printf(
    "Varianza de posicion abajo: %.6f -> %.6f m^2\n",
    varianza_vertical_inicial,
    prueba_completa.P[2][2]
);
/* PRUEBA: una medida de posicion norte.
   Estado inicial: pN = 0 m y P_NN = 4 m^2.
   Medida: 3 m, con R = 1 m^2. */
FusionEstado prueba_medida;

if (!fusion_inicializar(
        &prueba_medida, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prueba de correccion.\n");
    return 1;
}

float H_norte[FUSION_DIM_ERROR] = {0};
H_norte[0] = 1.0f;

float medida_norte = 3.0f;
float innovacion_norte =
    medida_norte - prueba_medida.posicion_ned_m[0];

FusionCorreccionEscalar correccion_norte;

if (!fusion_preparar_correccion_escalar(
        &prueba_medida,
        H_norte,
        innovacion_norte,
        1.0f,
        &correccion_norte))
{
    printf("Error al preparar la correccion.\n");
    return 1;
}

printf(
    "Correccion: S = %.3f; K_pN = %.3f; NIS = %.3f\n",
    correccion_norte.varianza_innovacion,
    correccion_norte.ganancia[0],
    correccion_norte.nis
);

printf(
    "Cambio propuesto en pN: %.3f m\n",
    correccion_norte.ganancia[0]
        * correccion_norte.innovacion
);
/* PRUEBA DE INYECCION NOMINAL:
   aplicar el cambio propuesto por la medida norte.
   Esta prueba aislada todavia no actualiza P. */
float delta_prueba[FUSION_DIM_ERROR];

for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
{
    delta_prueba[i] =
        correccion_norte.ganancia[i]
        * correccion_norte.innovacion;
}

if (!fusion_inyectar_error_nominal(
        &prueba_medida, delta_prueba))
{
    printf("Error al aplicar la correccion nominal.\n");
    return 1;
}

printf(
    "Posicion norte corregida: %.3f m\n",
    prueba_medida.posicion_ned_m[0]
);

printf(
    "Cuaternion corregido: %.3f %.3f %.3f %.3f\n",
    prueba_medida.q_cuerpo_a_ned[0],
    prueba_medida.q_cuerpo_a_ned[1],
    prueba_medida.q_cuerpo_a_ned[2],
    prueba_medida.q_cuerpo_a_ned[3]
);
/* PRUEBA: completar la actualizacion de P
   para la medida norte utilizada anteriormente. */
float varianza_antes_correccion = prueba_medida.P[0][0];

if (!fusion_corregir_covarianza_escalar(
        &prueba_medida,
        H_norte,
        correccion_norte.ganancia,
        1.0f))
{
    printf("Error al corregir la covarianza.\n");
    return 1;
}

printf(
    "Varianza norte corregida: %.3f -> %.3f m^2\n",
    varianza_antes_correccion,
    prueba_medida.P[0][0]
);

printf(
    "Posicion norte tras actualizar P: %.3f m\n",
    prueba_medida.posicion_ned_m[0]
);
/* PRUEBA: reinicio tras la correccion anterior.
   Sus componentes angulares son cero, por lo que J = I. */
if (!fusion_reiniciar_covarianza(
        &prueba_medida, &delta_prueba[6]))
{
    printf("Error al reiniciar la covarianza.\n");
    return 1;
}

printf(
    "Tras reinicio: pN = %.3f m; varianza pN = %.3f m^2\n",
    prueba_medida.posicion_ned_m[0],
    prueba_medida.P[0][0]
);
/* PRUEBA AISLADA DEL BLOQUE ANGULAR:
   comprobar la transformacion de una covarianza conocida. */
FusionEstado prueba_reset;

if (!fusion_inicializar(
        &prueba_reset, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prueba de reinicio.\n");
    return 1;
}

/* Varianzas angulares distintas para observar el efecto. */
prueba_reset.P[6][6] = 0.01f;
prueba_reset.P[7][7] = 0.04f;

const float delta_theta_reset[3] = {0.0f, 0.0f, 0.02f};

if (!fusion_reiniciar_covarianza(
        &prueba_reset, delta_theta_reset))
{
    printf("Error en la prueba angular de reinicio.\n");
    return 1;
}

printf(
    "Reset angular: Pxx = %.6f; Pyy = %.6f; Pxy = %.6f\n",
    prueba_reset.P[6][6],
    prueba_reset.P[7][7],
    prueba_reset.P[6][7]
);
/* PRUEBA: correccion completa y rechazo de una discrepancia.
   Umbral ilustrativo para esta prueba: NIS <= 9. */
FusionEstado prueba_correccion_completa;

if (!fusion_inicializar(
        &prueba_correccion_completa,
        q_inicial,
        sigma_inicial))
{
    printf("Error al inicializar la correccion completa.\n");
    return 1;
}

float H_pos_norte[FUSION_DIM_ERROR] = {0};
H_pos_norte[0] = 1.0f;

float r_norte =
    3.0f - prueba_correccion_completa.posicion_ned_m[0];

FusionResultadoCorreccion resultado_correccion =
    fusion_corregir_escalar(
        &prueba_correccion_completa,
        H_pos_norte,
        r_norte,
        1.0f,
        9.0f
    );

if (resultado_correccion != FUSION_CORRECCION_APLICADA)
{
    printf("Error: la medida de prueba no se ha aplicado.\n");
    return 1;
}

printf(
    "Correccion completa: pN = %.3f m; P_NN = %.3f m^2\n",
    prueba_correccion_completa.posicion_ned_m[0],
    prueba_correccion_completa.P[0][0]
);

/* Una segunda medida muy alejada debe rechazarse. */
float posicion_antes =
    prueba_correccion_completa.posicion_ned_m[0];
float varianza_antes =
    prueba_correccion_completa.P[0][0];

float r_incompatible =
    100.0f - prueba_correccion_completa.posicion_ned_m[0];

resultado_correccion = fusion_corregir_escalar(
    &prueba_correccion_completa,
    H_pos_norte,
    r_incompatible,
    1.0f,
    9.0f
);

if (resultado_correccion != FUSION_CORRECCION_RECHAZADA)
{
    printf("Error: se esperaba rechazar la segunda medida.\n");
    return 1;
}

/* Al rechazar, estos valores deben permanecer exactamente iguales. */
if (prueba_correccion_completa.posicion_ned_m[0] != posicion_antes ||
    prueba_correccion_completa.P[0][0] != varianza_antes)
{
    printf("Error: el rechazo ha modificado la estimacion.\n");
    return 1;
}

printf("Medida incompatible rechazada sin cambiar pN ni P_NN.\n");
/* PRUEBA: seis componentes GNSS sinteticas.
   Se parte del origen, velocidad cero y P inicial diagonal. */
FusionEstado prueba_gnss;

if (!fusion_inicializar(
        &prueba_gnss, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prueba GNSS.\n");
    return 1;
}

const FusionMedidaGnss medida_gnss_prueba = {
    .valor = {
        3.0f, -2.0f, 1.0f,
        0.2f, -0.1f, 0.05f
    },

    .varianza = {
        1.0f, 1.0f, 4.0f,
        0.04f, 0.04f, 0.09f
    },

    .usar = {1U, 1U, 1U, 1U, 1U, 1U}
};

int componentes_aplicadas = fusion_corregir_gnss(
    &prueba_gnss, &medida_gnss_prueba, 9.0f
);

if (componentes_aplicadas != 6)
{
    printf(
        "Error: se esperaban 6 componentes aplicadas; resultado = %d.\n",
        componentes_aplicadas
    );
    return 1;
}

printf("GNSS: componentes aplicadas = %d\n",
       componentes_aplicadas);

printf(
    "Posicion NED: %.3f %.3f %.3f m\n",
    prueba_gnss.posicion_ned_m[0],
    prueba_gnss.posicion_ned_m[1],
    prueba_gnss.posicion_ned_m[2]
);

printf(
    "Velocidad NED: %.3f %.3f %.3f m/s\n",
    prueba_gnss.velocidad_ned_m_s[0],
    prueba_gnss.velocidad_ned_m_s[1],
    prueba_gnss.velocidad_ned_m_s[2]
);
/* PRUEBA: promedio de dos alturas.
   Valores ilustrativos, no calibrados. */
float altura_baro_combinada;
float varianza_baro_combinada;

if (!fusion_combinar_alturas(
        10.0f, 4.0f,  /* Altura y varianza del 5C. */
        12.0f, 4.0f,  /* Altura y varianza del 5D. */
        0.0f,        /* Errores independientes en esta prueba. */
        &altura_baro_combinada,
        &varianza_baro_combinada))
{
    printf("Error al combinar las alturas.\n");
    return 1;
}

printf(
    "Barometros independientes: altura = %.3f m; "
    "varianza = %.3f m^2\n",
    altura_baro_combinada,
    varianza_baro_combinada
);

/* Mismas varianzas, ahora con una parte del error compartida. */
if (!fusion_combinar_alturas(
        10.0f, 4.0f,
        12.0f, 4.0f,
        2.0f,
        &altura_baro_combinada,
        &varianza_baro_combinada))
{
    printf("Error al combinar alturas correlacionadas.\n");
    return 1;
}

printf(
    "Barometros correlacionados: altura = %.3f m; "
    "varianza = %.3f m^2\n",
    altura_baro_combinada,
    varianza_baro_combinada
);
/* PRUEBA: combinar dos alturas y corregir el estimador.
   Valores ilustrativos, no calibrados. */
FusionEstado prueba_baro_fusion;

if (!fusion_inicializar(
        &prueba_baro_fusion, q_inicial, sigma_inicial))
{
    printf("Error al inicializar la prueba barometrica.\n");
    return 1;
}

/* Altura inicial de 10 m: posicion abajo de -10 m. */
prueba_baro_fusion.posicion_ned_m[2] = -10.0f;

float altura_para_fusion;
float varianza_para_fusion;

if (!fusion_combinar_alturas(
        10.0f, 4.0f,
        12.0f, 4.0f,
        0.0f,
        &altura_para_fusion,
        &varianza_para_fusion))
{
    printf("Error al preparar la medida barometrica.\n");
    return 1;
}

FusionResultadoCorreccion resultado_baro_fusion =
    fusion_corregir_barometro(
        &prueba_baro_fusion,
        altura_para_fusion,
        varianza_para_fusion,
        9.0f
    );

if (resultado_baro_fusion != FUSION_CORRECCION_APLICADA)
{
    printf("Error: correccion barometrica no aplicada.\n");
    return 1;
}

printf(
    "Altura tras la correccion: %.3f m\n",
    -prueba_baro_fusion.posicion_ned_m[2]
);

printf(
    "Varianza vertical tras la correccion: %.3f m^2\n",
    prueba_baro_fusion.P[2][2]
);
/* PRUEBA: disponibilidad de sensores y discrepancia. */
FusionEstado prueba_gestion_baro;

if (!fusion_inicializar(
        &prueba_gestion_baro, q_inicial, sigma_inicial))
{
    return 1;
}

prueba_gestion_baro.posicion_ned_m[2] = -10.0f;

FusionMedidasBarometros lecturas_baro = {
    .altura_m = {11.0f, 0.0f},
    .varianza_m2 = {4.0f, 0.0f},
    .usar = {1U, 0U}
};

/* Solo el 5C esta disponible. */
if (fusion_corregir_barometros(
        &prueba_gestion_baro, &lecturas_baro,
        0.0f, 9.0f, 9.0f) != FUSION_CORRECCION_APLICADA)
{
    printf("Error al utilizar un solo barometro.\n");
    return 1;
}

printf("Altura con un barometro: %.3f m\n",
       -prueba_gestion_baro.posicion_ned_m[2]);

/* Ahora ambos son utilizables, pero discrepan mucho. */
lecturas_baro.usar[1] = 1U;
lecturas_baro.altura_m[1] = 40.0f;
lecturas_baro.varianza_m2[1] = 4.0f;

if (fusion_corregir_barometros(
        &prueba_gestion_baro, &lecturas_baro,
        0.0f, 9.0f, 9.0f) != FUSION_CORRECCION_RECHAZADA)
{
    printf("Error: discrepancia barometrica no rechazada.\n");
    return 1;
}

printf("Altura tras rechazar la pareja: %.3f m\n",
       -prueba_gestion_baro.posicion_ned_m[2]);
       /* PRUEBA: campo ilustrativo, expresado en microteslas. */
const float campo_ned_prueba[3] = {20.0f, 0.0f, 40.0f};
float campo_predicho[3];

/* Con los ejes alineados, las componentes no cambian. */
fusion_predecir_campo_magnetico(
    q_inicial, campo_ned_prueba, campo_predicho
);

printf(
    "Campo con ejes alineados: %.3f %.3f %.3f uT\n",
    campo_predicho[0],
    campo_predicho[1],
    campo_predicho[2]
);

/* Reutilizamos el cuaternion del giro de +90 grados en Z. */
fusion_predecir_campo_magnetico(
    q_90, campo_ned_prueba, campo_predicho
);

printf(
    "Campo tras girar 90 grados: %.3f %.3f %.3f uT\n",
    campo_predicho[0],
    campo_predicho[1],
    campo_predicho[2]
);
/* PRUEBA: fila correspondiente a la componente X. */
const float campo_para_H[3] = {20.0f, 0.0f, 40.0f};
float H_mag_x[FUSION_DIM_ERROR];

if (!fusion_fila_medida_magnetica(
        campo_para_H, 0U, H_mag_x))
{
    printf("Error al construir la fila magnetica.\n");
    return 1;
}

printf(
    "H magnetica X: thetaX = %.3f; "
    "thetaY = %.3f; thetaZ = %.3f\n",
    H_mag_x[6],
    H_mag_x[7],
    H_mag_x[8]
);
/* Valores ilustrativos en microteslas.
   El limite de 5 uT solo se utiliza para esta prueba. */
const float referencia_intensidad[3] = {20.0f, 0.0f, 40.0f};

const float medida_intensidad_valida[3] = {
    0.0f, -20.0f, 40.0f
};

const float medida_intensidad_alterada[3] = {
    0.0f, -40.0f, 80.0f
};

int compatible = fusion_comprobar_intensidad_magnetica(
    medida_intensidad_valida,
    referencia_intensidad,
    5.0f
);

int incompatible = fusion_comprobar_intensidad_magnetica(
    medida_intensidad_alterada,
    referencia_intensidad,
    5.0f
);

if (compatible != 1 || incompatible != 0)
{
    printf("Error en la comprobacion de intensidad.\n");
    return 1;
}

printf("Intensidad compatible aceptada y alterada rechazada.\n");
/* PRUEBA: la componente Y medida es ligeramente negativa.
   Campo ilustrativo en uT; varianza en uT^2. */
FusionEstado prueba_mag_componente;

if (!fusion_inicializar(
        &prueba_mag_componente, q_inicial, sigma_inicial))
{
    return 1;
}

const float referencia_mag_componente[3] = {
    20.0f, 0.0f, 40.0f
};

FusionResultadoCorreccion resultado_mag_componente =
    fusion_corregir_componente_magnetica(
        &prueba_mag_componente,
        referencia_mag_componente,
        1U,       /* Componente Y. */
        -1.0f,    /* Medida: -1 uT; prediccion inicial: 0 uT. */
        1.0f,     /* Varianza: 1 uT^2. */
        9.0f      /* Umbral NIS ilustrativo. */
    );

if (resultado_mag_componente != FUSION_CORRECCION_APLICADA)
{
    printf("Error: correccion magnetica no aplicada.\n");
    return 1;
}

float campo_tras_correccion[3];

fusion_predecir_campo_magnetico(
    prueba_mag_componente.q_cuerpo_a_ned,
    referencia_mag_componente,
    campo_tras_correccion
);

printf(
    "Campo Y predicho: 0.000 -> %.3f uT; medida: -1.000 uT\n",
    campo_tras_correccion[1]
);
/* PRUEBA: lectura completa y rechazo por intensidad.
   Valores ilustrativos en uT y uT^2. */
FusionEstado prueba_mag_completa;

if (!fusion_inicializar(
        &prueba_mag_completa, q_inicial, sigma_inicial))
{
    return 1;
}

const float referencia_mag_completa[3] = {
    20.0f, 0.0f, 40.0f
};

const float lectura_mag_completa[3] = {
    20.0f, -1.0f, 40.0f
};

const float varianzas_mag[3] = {
    1.0f, 1.0f, 1.0f
};

int aplicadas_mag = fusion_corregir_magnetometro(
    &prueba_mag_completa,
    lectura_mag_completa,
    referencia_mag_completa,
    varianzas_mag,
    5.0f,
    9.0f
);

if (aplicadas_mag != 3)
{
    printf("Error: se esperaban tres componentes magneticas.\n");
    return 1;
}

printf("Magnetometro: componentes aplicadas = %d\n",
       aplicadas_mag);

/* Una lectura con el doble de intensidad debe rechazarse. */
const float lectura_mag_perturbada[3] = {
    40.0f, 0.0f, 80.0f
};

FusionEstado antes_rechazo_mag = prueba_mag_completa;

int resultado_rechazo_mag = fusion_corregir_magnetometro(
    &prueba_mag_completa,
    lectura_mag_perturbada,
    referencia_mag_completa,
    varianzas_mag,
    5.0f,
    9.0f
);

if (resultado_rechazo_mag != 0)
{
    printf("Error: lectura perturbada no rechazada.\n");
    return 1;
}

for (unsigned int i = 0U; i < 4U; ++i)
{
    if (prueba_mag_completa.q_cuerpo_a_ned[i] !=
        antes_rechazo_mag.q_cuerpo_a_ned[i])
    {
        printf("Error: el rechazo ha cambiado la orientacion.\n");
        return 1;
    }
}

printf("Lectura perturbada rechazada sin cambiar la orientacion.\n");
/* PRUEBA DE INTEGRACION:
   un ciclo en reposo, con ejes del cuerpo alineados con NED.
   Reutilizamos q_inicial, sigma_inicial y ruido_prueba. */

FusionEstimador prueba_integracion;

if (!fusion_inicializar_estimador(
        &prueba_integracion,
        q_inicial,
        sigma_inicial,
        0.0))
{
    printf("Error al inicializar la integracion.\n");
    return 1;
}

/* Todas las medidas de este ciclo comparten este instante. */
const double tiempo_ciclo = 0.01;

/* IMU ideal en reposo, con cuerpo alineado con NED. */
const float acc_integracion[3] = {0.0f, 0.0f, -1.0f};
const float giro_integracion[3] = {0.0f, 0.0f, 0.0f};

/* GNSS: posicion y velocidad nulas.
   Varianzas ilustrativas y positivas. */
const FusionMedidaGnss gnss_integracion = {
    .valor = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .varianza = {1.0f, 1.0f, 4.0f, 0.04f, 0.04f, 0.09f},
    .usar = {1U, 1U, 1U, 1U, 1U, 1U}
};

/* Ambos barometros indican altura cero. */
const FusionMedidasBarometros baro_integracion = {
    .altura_m = {0.0f, 0.0f},
    .varianza_m2 = {4.0f, 4.0f},
    .usar = {1U, 1U}
};

const double tiempos_baro_integracion[2] = {
    tiempo_ciclo, tiempo_ciclo
};

/* Con cuerpo alineado con NED, el campo ideal
   tiene las mismas componentes en ambos sistemas. */
const float campo_integracion[3] = {20.0f, 0.0f, 40.0f};
const float varianza_mag_integracion[3] = {1.0f, 1.0f, 1.0f};

/* 1. Avanzar el estado hasta tiempo_ciclo. */
if (!fusion_predecir_hasta(
        &prueba_integracion,
        acc_integracion,
        giro_integracion,
        tiempo_ciclo,
        0.02f,       /* dt maximo ilustrativo. */
        9.80665f,
        &ruido_prueba))
{
    printf("Error en la prediccion integrada.\n");
    return 1;
}

/* 2. Corregir GNSS en ese mismo instante. */
int gnss_aplicadas_integracion = fusion_procesar_gnss(
    &prueba_integracion,
    &gnss_integracion,
    tiempo_ciclo,
    9.0f
);

/* 3. Corregir con los barometros. */
FusionResultadoCorreccion baro_resultado_integracion =
    fusion_procesar_barometros(
        &prueba_integracion,
        &baro_integracion,
        tiempos_baro_integracion,
        0.0f,       /* Covarianza entre barometros. */
        9.0f,
        9.0f
    );

/* 4. Corregir con el magnetometro. */
int mag_aplicadas_integracion = fusion_procesar_magnetometro(
    &prueba_integracion,
    campo_integracion,
    campo_integracion,
    varianza_mag_integracion,
    tiempo_ciclo,
    5.0f,
    9.0f
);

if (gnss_aplicadas_integracion != 6 ||
    baro_resultado_integracion != FUSION_CORRECCION_APLICADA ||
    mag_aplicadas_integracion != 3)
{
    printf("Error: resultados inesperados en la integracion.\n");
    return 1;
}

printf(
    "Ciclo integrado: t = %.3f s; GNSS = %d; "
    "barometros = %d; magnetometro = %d\n",
    prueba_integracion.tiempo_s,
    gnss_aplicadas_integracion,
    (int)baro_resultado_integracion,
    mag_aplicadas_integracion
);
/* Repetir el mismo instante IMU debe impedir otra prediccion. */
if (fusion_predecir_hasta(
        &prueba_integracion,
        acc_integracion,
        giro_integracion,
        tiempo_ciclo,
        0.02f,
        9.80665f,
        &ruido_prueba) != 0)
{
    printf("Error: se ha repetido la prediccion IMU.\n");
    return 1;
}

/* Ninguna lectura externa debe aplicarse por segunda vez. */
int gnss_repetido = fusion_procesar_gnss(
    &prueba_integracion,
    &gnss_integracion,
    tiempo_ciclo,
    9.0f
);

FusionResultadoCorreccion baro_repetido =
    fusion_procesar_barometros(
        &prueba_integracion,
        &baro_integracion,
        tiempos_baro_integracion,
        0.0f,
        9.0f,
        9.0f
    );

int mag_repetido = fusion_procesar_magnetometro(
    &prueba_integracion,
    campo_integracion,
    campo_integracion,
    varianza_mag_integracion,
    tiempo_ciclo,
    5.0f,
    9.0f
);

if (gnss_repetido != 0 ||
    baro_repetido != FUSION_CORRECCION_RECHAZADA ||
    mag_repetido != 0)
{
    printf("Error: se ha reutilizado una lectura externa.\n");
    return 1;
}

printf("Repeticiones IMU, GNSS, barometros y magnetometro omitidas.\n");
    return 0;
}