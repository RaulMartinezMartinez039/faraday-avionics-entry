#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "fusion.h"

/* Tiempo y gravedad. */
static const double DT_SIM = 0.01;
static const float G_SIM = 9.80665f;

/* Desviaciones tipicas del ruido por muestra y por eje. */
static const float SIGMA_ACC = 0.10f;      /* m/s^2 */
static const float SIGMA_GIRO = 0.002f;    /* rad/s */

static const float SIGMA_GNSS_POS[3] = {
    2.0f, 2.0f, 3.0f                     /* m: N, E, D */
};

static const float SIGMA_GNSS_VEL[3] = {
    0.2f, 0.2f, 0.3f                     /* m/s: N, E, D */
};

static const float SIGMA_BARO = 2.0f;      /* m, cada sensor */
static const float SIGMA_MAG = 0.5f;      /* microteslas */

/* Sesgos reales usados SOLO para generar las lecturas.
Expresados en los ejes del cuerpo. */
static const float SESGO_ACC_REAL[3] = {
    0.08f, -0.05f, 0.03f                 /* m/s^2 */
};

static const float SESGO_GIRO_REAL[3] = {
    0.001f, -0.0007f, 0.0005f            /* rad/s */
};

/* Referencia magnetica sintetica en NED. */
static const float CAMPO_NED_SIM[3] = {
    20.0f, 0.0f, 40.0f                   /* microteslas */
};
/* Numero pseudoaleatorio estrictamente entre 0 y 1.
Evitamos los extremos, especialmente log(0). */
static double simulacion_uniforme(void)
{
    return ((double)rand() + 1.0)
    / ((double)RAND_MAX + 2.0);
}

/* Ruido gaussiano de media cero y desviacion tipica uno.
Transformacion de Box-Muller. */
static float simulacion_normal(void)
{
    const double dos_pi = 6.283185307179586;

    double u1 = simulacion_uniforme();
    double u2 = simulacion_uniforme();

    double z = sqrt(-2.0 * log(u1))
    * cos(dos_pi * u2);

    return (float)z;
}

typedef struct
{
    double altura_m;
    double velocidad_vertical_m_s;
    double aceleracion_vertical_m_s2;
} SimulacionReferencia;

static SimulacionReferencia simulacion_referencia(double tiempo_s)
{
    SimulacionReferencia referencia = {0};

    if (tiempo_s < 2.0)
    {
        /* Cohete en reposo sobre la plataforma. */
        return referencia;
    }

    if (tiempo_s < 7.0)
    {
        /* Ascenso propulsado: aceleracion neta hacia arriba. */
        const double aceleracion = 20.0;
        const double tau = tiempo_s - 2.0;

        referencia.altura_m =
            0.5 * aceleracion * tau * tau;

        referencia.velocidad_vertical_m_s =
            aceleracion * tau;

        referencia.aceleracion_vertical_m_s2 =
            aceleracion;

        return referencia;
    }

    /* Ascenso balistico tras el apagado del motor. */
    const double tiempo_desde_apagado = tiempo_s - 7.0;
    const double altura_apagado = 250.0;
    const double velocidad_apagado = 100.0;
    const double gravedad = (double)G_SIM;

    referencia.altura_m =
        altura_apagado
        + velocidad_apagado * tiempo_desde_apagado
        - 0.5 * gravedad
        * tiempo_desde_apagado * tiempo_desde_apagado;

    referencia.velocidad_vertical_m_s =
        velocidad_apagado
        - gravedad * tiempo_desde_apagado;

    referencia.aceleracion_vertical_m_s2 =
        -gravedad;

    return referencia;
}

static void simulacion_generar_imu(
    const SimulacionReferencia *referencia,
    float aceleracion_g[3],
    float giro_dps[3])
{
    const float radianes_a_grados = 57.295779513f;

    const float fuerza_ideal[3] = {
        (float)(referencia->aceleracion_vertical_m_s2
            + (double)G_SIM),
        0.0f,
        0.0f
    };

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        const float fuerza_medida =
            fuerza_ideal[i]
            + SESGO_ACC_REAL[i]
            + SIGMA_ACC * simulacion_normal();

        const float giro_medido =
            SESGO_GIRO_REAL[i]
            + SIGMA_GIRO * simulacion_normal();

        aceleracion_g[i] = fuerza_medida / G_SIM;

        giro_dps[i] = giro_medido * radianes_a_grados;
    }
}

static FusionMedidaGnss simulacion_generar_gnss(
    const SimulacionReferencia *referencia)
{
    FusionMedidaGnss medida = {0};

    const float posicion_ned[3] = {
        0.0f,
        0.0f,
        (float)(-referencia->altura_m)
    };

    const float velocidad_ned[3] = {
        0.0f,
        0.0f,
        (float)(-referencia->velocidad_vertical_m_s)
    };

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        medida.valor[i] =
            posicion_ned[i]
            + SIGMA_GNSS_POS[i] * simulacion_normal();

        medida.valor[i + 3U] =
            velocidad_ned[i]
            + SIGMA_GNSS_VEL[i] * simulacion_normal();

        medida.varianza[i] =
            SIGMA_GNSS_POS[i] * SIGMA_GNSS_POS[i];

        medida.varianza[i + 3U] =
            SIGMA_GNSS_VEL[i] * SIGMA_GNSS_VEL[i];

        medida.usar[i] = 1U;
        medida.usar[i + 3U] = 1U;
    }

    return medida;
}

static FusionMedidasBarometros simulacion_generar_barometros(
    const SimulacionReferencia *referencia)
{
    FusionMedidasBarometros medidas = {0};

    for (unsigned int i = 0U; i < 2U; ++i)
    {
        medidas.altura_m[i] =
            (float)referencia->altura_m
            + SIGMA_BARO * simulacion_normal();

        medidas.varianza_m2[i] =
            SIGMA_BARO * SIGMA_BARO;

        medidas.usar[i] = 1U;
    }

    return medidas;
}

static void simulacion_generar_magnetometro(
    float campo_medido[3],
    float varianza[3])
{
    const float campo_ideal[3] = {
        -CAMPO_NED_SIM[2],
        CAMPO_NED_SIM[1],
        CAMPO_NED_SIM[0]
    };

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        campo_medido[i] =
            campo_ideal[i]
            + SIGMA_MAG * simulacion_normal();

        varianza[i] =
            SIGMA_MAG * SIGMA_MAG;
    }
}

static int simulacion_inicializar_estimador(
    FusionEstimador *estimador,
    FusionRuido *ruido)
{
    const float q_inicial[4] = {
        0.7071067812f,
        0.0f,
        0.7071067812f,
        0.0f
    };

    const float sigma_inicial[FUSION_DIM_ERROR] = {
        2.0f,  2.0f,  2.0f,   /* Posicion: m. */
        0.1f,  0.1f,  0.1f,   /* Velocidad: m/s. */
        0.05f, 0.05f, 0.05f,  /* Orientacion: rad. */
        0.2f,  0.2f,  0.2f,   /* Sesgo acelerometro: m/s^2. */
        0.01f, 0.01f, 0.01f   /* Sesgo giroscopo: rad/s. */
    };

    if (!fusion_inicializar_estimador(
        estimador, q_inicial, sigma_inicial, 0.0))
    {
        return 0;
    }

    ruido->q_acc =
        SIGMA_ACC * SIGMA_ACC * (float)DT_SIM;

    ruido->q_giro =
        SIGMA_GIRO * SIGMA_GIRO * (float)DT_SIM;

    ruido->q_sesgo_acc = 0.0f;
    ruido->q_sesgo_giro = 0.0f;

    return 1;
}

static int simulacion_aplicar_correcciones(
    FusionEstimador *estimador,
    const SimulacionReferencia *referencia,
    unsigned int paso)
{
    const double tiempo_s = estimador->tiempo_s;
    const float umbral_nis = 9.0f;

    /* GNSS: una lectura cada diez pasos. */
    if (paso % 10U == 0U)
    {
        FusionMedidaGnss medida =
            simulacion_generar_gnss(referencia);

        int resultado = fusion_procesar_gnss(
            estimador,
            &medida,
            tiempo_s,
            umbral_nis);

        if (resultado < 0)
        {
            return 0;
        }
    }

    /* Barometros: una lectura de cada sensor en cada paso. */
    FusionMedidasBarometros medidas =
        simulacion_generar_barometros(referencia);

    const double tiempos_baro[2] = {
        tiempo_s,
        tiempo_s
    };

    FusionResultadoCorreccion resultado_baro =
        fusion_procesar_barometros(
            estimador,
            &medidas,
            tiempos_baro,
            0.0f,
            9.0f,
            umbral_nis);

    if (resultado_baro == FUSION_CORRECCION_ERROR)
    {
        return 0;
    }

    /* Magnetometro: una lectura cada cinco pasos. */
    if (paso % 5U == 0U)
    {
        float campo_medido[3];
        float varianza[3];

        simulacion_generar_magnetometro(
            campo_medido, varianza);

        int resultado = fusion_procesar_magnetometro(
            estimador,
            campo_medido,
            CAMPO_NED_SIM,
            varianza,
            tiempo_s,
            5.0f,
            umbral_nis);

        if (resultado < 0)
        {
            return 0;
        }
    }

    return 1;
}

static int simulacion_guardar_resultado(
    FILE *archivo,
    const FusionEstimador *estimador,
    const SimulacionReferencia *referencia)
{
    const FusionEstado *estado = &estimador->estado;

    const double altura_estimada =
        -(double)estado->posicion_ned_m[2];

    const double velocidad_estimada =
        -(double)estado->velocidad_ned_m_s[2];

    const double error_altura =
        referencia->altura_m - altura_estimada;

    const double error_velocidad =
        referencia->velocidad_vertical_m_s
        - velocidad_estimada;

    if (fprintf(
        archivo,
        "%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
        estimador->tiempo_s,
        referencia->altura_m,
        altura_estimada,
        referencia->velocidad_vertical_m_s,
        velocidad_estimada,
        error_altura,
        error_velocidad) < 0)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (fprintf(
            archivo,
            ",%.9g",
            (double)estado->sesgo_acc_m_s2[i]) < 0)
        {
            return 0;
        }
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (fprintf(
            archivo,
            ",%.9g",
            (double)estado->sesgo_giro_rad_s[i]) < 0)
        {
            return 0;
        }
    }

    return fputc('\n', archivo) != EOF;
}

int main(void)
{
    FusionEstimador estimador;
    FusionRuido ruido;

    srand(1U);

    if (!simulacion_inicializar_estimador(
        &estimador, &ruido))
    {
        fprintf(stderr, "Error al inicializar el estimador.\n");
        return EXIT_FAILURE;
    }
    FILE *archivo = fopen("resultados_simulacion.csv", "w");

    if (archivo == NULL)
    {
        perror("No se pudo abrir el archivo de resultados");
        return EXIT_FAILURE;
    }

    SimulacionReferencia referencia_inicial =
        simulacion_referencia(estimador.tiempo_s);

    if (fputs(
        "tiempo_s,"
        "altura_ref_m,altura_est_m,"
        "velocidad_ref_m_s,velocidad_est_m_s,"
        "error_altura_m,error_velocidad_m_s,"
        "sesgo_acc_x_m_s2,sesgo_acc_y_m_s2,sesgo_acc_z_m_s2,"
        "sesgo_giro_x_rad_s,sesgo_giro_y_rad_s,"
        "sesgo_giro_z_rad_s\n",
        archivo) == EOF
        || !simulacion_guardar_resultado(
        archivo, &estimador, &referencia_inicial))
    {
        fprintf(stderr, "Error al escribir los datos iniciales.\n");
        fclose(archivo);
        return EXIT_FAILURE;
    }
    const double tiempo_apogeo_s =
        7.0 + 100.0 / (double)G_SIM;

    const unsigned int numero_pasos =
        (unsigned int)(tiempo_apogeo_s / DT_SIM);

    const float dt_max_s = (float)(1.5 * DT_SIM);

    for (unsigned int paso = 1U;
        paso <= numero_pasos;
        ++paso)
    {
        const double tiempo_s =
            (double)paso * DT_SIM;

        const double tiempo_medio_s =
            0.5 * (estimador.tiempo_s + tiempo_s);

        /* Referencia durante el intervalo de prediccion. */
        SimulacionReferencia referencia_imu =
            simulacion_referencia(tiempo_medio_s);

        float aceleracion_g[3];
        float giro_dps[3];

        simulacion_generar_imu(
            &referencia_imu,
            aceleracion_g,
            giro_dps);

        if (!fusion_predecir_hasta(
            &estimador,
            aceleracion_g,
            giro_dps,
            tiempo_s,
            dt_max_s,
            G_SIM,
            &ruido))
        {
            fprintf(
                stderr,
                "Error de prediccion en t = %.3f s.\n",
                tiempo_s);
            fclose(archivo);
            return EXIT_FAILURE;
        }

        /* Referencia en el instante de las correcciones. */
        SimulacionReferencia referencia =
            simulacion_referencia(tiempo_s);

        if (!simulacion_aplicar_correcciones(
            &estimador,
            &referencia,
            paso))
        {
            fprintf(
                stderr,
                "Error de correccion en t = %.3f s.\n",
                tiempo_s);
            fclose(archivo);
            return EXIT_FAILURE;
        }

        if (!simulacion_guardar_resultado(
            archivo, &estimador, &referencia))
        {
            fprintf(
                stderr,
                "Error al guardar resultados en t = %.3f s.\n",
                tiempo_s);

            fclose(archivo);
            return EXIT_FAILURE;
        }
    }
    if (fclose(archivo) == EOF)
    {
        fprintf(stderr, "Error al cerrar el archivo de resultados.\n");
        return EXIT_FAILURE;
    }

    printf("Resultados guardados en resultados_simulacion.csv\n");
    SimulacionReferencia referencia_final =
        simulacion_referencia(estimador.tiempo_s);

    printf("Tiempo final: %.3f s\n", estimador.tiempo_s);

    printf(
        "Altura: referencia = %.3f m, estimada = %.3f m\n",
        referencia_final.altura_m,
        (double)(-estimador.estado.posicion_ned_m[2]));

    printf(
        "Velocidad vertical: referencia = %.3f m/s, "
        "estimada = %.3f m/s\n",
        referencia_final.velocidad_vertical_m_s,
        (double)(-estimador.estado.velocidad_ned_m_s[2]));

    return EXIT_SUCCESS;
}
