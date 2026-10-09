/* Aqui incorporaremos las funciones por etapas:
   1. Inicializacion del estimador.
   2. Prediccion con acelerometro y giroscopio.
   3. Correccion con GNSS.
   4. Correccion con barometros.
   5. Correccion con magnetometro. */
#include "fusion.h"
#include <math.h>
#include <stddef.h>

/* =========================================================
   ETAPA 1: INICIALIZACION, ANTES DE PROCESAR LOS SENSORES
   ========================================================= */

int fusion_inicializar(
    FusionEstado *estado,
    const float q_inicial[4],
    const float sigma_inicial[FUSION_DIM_ERROR]
)
{
    /* 1. Comprobamos los argumentos antes de cambiar el estado. */
    if (estado == NULL ||
        q_inicial == NULL ||
        sigma_inicial == NULL)
    {
        return 0;
    }

    /* 2. Comprobamos que el cuaternion pueda normalizarse. */
    float norma2 = 0.0f;

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        norma2 += q_inicial[i] * q_inicial[i];
    }

    if (!isfinite(norma2) || norma2 < 1.0e-12f)
    {
        return 0;
    }

    /* 3. Las incertidumbres deben ser positivas y finitas.
       Comprobamos tambien que sus cuadrados sean validos. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        float sigma = sigma_inicial[i];
        float varianza = sigma * sigma;

        if (!isfinite(sigma) || sigma <= 0.0f ||
            !isfinite(varianza) || varianza <= 0.0f)
        {
            return 0;
        }
    }

    /* 4. Preparamos un estado inicial:
       posicion = origen, velocidad = cero y sesgos = cero.
       La matriz P tambien empieza a cero antes de rellenarla. */
    FusionEstado inicial = {0};

    /* 5. Guardamos la orientacion con norma unitaria. */
    float norma = sqrtf(norma2);

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        inicial.q_cuerpo_a_ned[i] = q_inicial[i] / norma;
    }

    /* 6. Guardamos las varianzas en la diagonal de P.
       Inicialmente suponemos errores no correlacionados. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        inicial.P[i][i] =
            sigma_inicial[i] * sigma_inicial[i];
    }

    /* 7. Solo modificamos el estimador si todo es valido. */
    *estado = inicial;

    return 1;
}
/* =========================================================
   ETAPA 2: IMU - UNIDADES Y CORRECCION DE SESGOS
   ========================================================= */

void fusion_preparar_imu(
    const FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float fuerza_corregida[3],
    float giro_corregido[3]
)
{
    const float g0 = 9.80665f;
    const float grados_a_radianes = 0.01745329252f;

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        /* ACELEROMETRO:
           convertimos g a m/s^2 y restamos el sesgo.
           El resultado sigue siendo fuerza especifica:
           todavia no hemos incorporado la gravedad. */
        fuerza_corregida[i] =
            aceleracion_g[i] * g0 - estado->sesgo_acc_m_s2[i];

        /* GIROSCOPIO:
           convertimos grados/s a rad/s y restamos el sesgo. */
        giro_corregido[i] =
            giro_dps[i] * grados_a_radianes
            - estado->sesgo_giro_rad_s[i];
    }
}
/* =========================================================
   ETAPA 3: GIROSCOPIO - INCREMENTO DE ORIENTACION
   ========================================================= */

int fusion_incremento_rotacion(
    const float giro_rad_s[3],
    float dt,
    float dq[4]
)
{
    /* 1. Comprobamos las entradas. */
    if (giro_rad_s == NULL || dq == NULL ||
        !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(giro_rad_s[i]))
        {
            return 0;
        }
    }

    /* 2. Magnitud de la velocidad angular y angulo recorrido.
       hypotf calcula la norma evitando cuadrados muy grandes. */
    float velocidad = hypotf(
        hypotf(giro_rad_s[0], giro_rad_s[1]),
        giro_rad_s[2]
    );

    float angulo = velocidad * dt;

    if (!isfinite(velocidad) || !isfinite(angulo))
    {
        return 0;
    }

    /* 3. Sin giro, el incremento es el cuaternion identidad. */
    if (velocidad == 0.0f)
    {
        dq[0] = 1.0f;
        dq[1] = 0.0f;
        dq[2] = 0.0f;
        dq[3] = 0.0f;
        return 1;
    }

    /* 4. Convertimos el giro en un cuaternion.
       La direccion del giro la aporta el vector del giroscopio. */
    float factor = sinf(0.5f * angulo) / velocidad;

    dq[0] = cosf(0.5f * angulo);
    dq[1] = factor * giro_rad_s[0];
    dq[2] = factor * giro_rad_s[1];
    dq[3] = factor * giro_rad_s[2];

    return 1;
}
/* =========================================================
   ETAPA 4: ORIENTACION - COMBINACION DE ROTACIONES
   ========================================================= */

void fusion_producto_cuaterniones(
    const float a[4],
    const float b[4],
    float resultado[4]
)
{
    /* Calculamos primero en variables auxiliares.
       Asi podemos guardar el resultado sobre una entrada
       sin sobrescribir valores que todavia necesitamos. */
    float producto[4];

    /* Componente escalar. */
    producto[0] =
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];

    /* Componentes vectoriales X, Y y Z. */
    producto[1] =
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];

    producto[2] =
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];

    producto[3] =
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        resultado[i] = producto[i];
    }
}
/* =========================================================
   ORIENTACION: NORMALIZACION DESPUES DE COMBINAR GIROS
   ========================================================= */

int fusion_normalizar_cuaternion(float q[4])
{
    if (q == NULL)
    {
        return 0;
    }

    /* Comprobamos las cuatro componentes. */
    for (unsigned int i = 0U; i < 4U; ++i)
    {
        if (!isfinite(q[i]))
        {
            return 0;
        }
    }

    /* Calculamos la norma del cuaternion. */
    float norma = hypotf(
        hypotf(q[0], q[1]),
        hypotf(q[2], q[3])
    );

    if (!isfinite(norma) || norma < 1.0e-6f)
    {
        return 0;
    }

    /* Dividimos todas las componentes por la misma norma. */
    for (unsigned int i = 0U; i < 4U; ++i)
    {
        q[i] /= norma;
    }

    return 1;
}
/* =========================================================
   IMU: TRANSFORMACION DE LOS EJES DEL CUERPO A NED
   ========================================================= */

void fusion_matriz_rotacion(
    const float q[4],
    float C[3][3]
)
{
    /* Orientacion: cuaternion unitario de Hamilton. */
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    /* Fila 0: componente norte del vector transformado. */
    C[0][0] = 1.0f - 2.0f*(y*y + z*z);
    C[0][1] = 2.0f*(x*y - w*z);
    C[0][2] = 2.0f*(x*z + w*y);

    /* Fila 1: componente este. */
    C[1][0] = 2.0f*(x*y + w*z);
    C[1][1] = 1.0f - 2.0f*(x*x + z*z);
    C[1][2] = 2.0f*(y*z - w*x);

    /* Fila 2: componente abajo. */
    C[2][0] = 2.0f*(x*z - w*y);
    C[2][1] = 2.0f*(y*z + w*x);
    C[2][2] = 1.0f - 2.0f*(x*x + y*y);
}
void fusion_aceleracion_ned(
    const float q[4],
    const float fuerza_corregida[3],
    float gravedad_m_s2,
    float aceleracion_ned[3]
)
{
    /* Se requiere un cuaternion normalizado y medidas validas.
       gravedad_m_s2 es la magnitud positiva de la gravedad. */

    /* ORIENTACION: matriz del cuerpo al sistema NED. */
    float C[3][3];
    fusion_matriz_rotacion(q, C);

    /* ACELEROMETRO: transformar la fuerza especifica.
       Cada fila calcula una componente: norte, este o abajo. */
    float aceleracion[3];

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        aceleracion[i] =
            C[i][0] * fuerza_corregida[0]
          + C[i][1] * fuerza_corregida[1]
          + C[i][2] * fuerza_corregida[2];
    }

    /* GRAVEDAD: en NED, abajo es el sentido positivo.
       Se suma solo a la componente vertical. */
    aceleracion[2] += gravedad_m_s2;

    /* Entregar las tres componentes calculadas. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        aceleracion_ned[i] = aceleracion[i];
    }
}
int fusion_integrar_movimiento(
    FusionEstado *estado,
    const float aceleracion_ned[3],
    float dt
)
{
    /* Comprobar los argumentos y el intervalo de tiempo. */
    if (estado == NULL || aceleracion_ned == NULL ||
        !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    float nueva_posicion[3];
    float nueva_velocidad[3];

    /* MOVIMIENTO: integrar por separado norte, este y abajo. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        float p = estado->posicion_ned_m[i];
        float v = estado->velocidad_ned_m_s[i];
        float a = aceleracion_ned[i];

        if (!isfinite(p) || !isfinite(v) || !isfinite(a))
        {
            return 0;
        }

        /* Posicion: utiliza la velocidad inicial del intervalo. */
        nueva_posicion[i] = p + v * dt + 0.5f * a * dt * dt;

        /* Velocidad: sumar el cambio producido por la aceleracion. */
        nueva_velocidad[i] = v + a * dt;

        if (!isfinite(nueva_posicion[i]) ||
            !isfinite(nueva_velocidad[i]))
        {
            return 0;
        }
    }

    /* Guardar solo cuando las tres componentes son correctas. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        estado->posicion_ned_m[i] = nueva_posicion[i];
        estado->velocidad_ned_m_s[i] = nueva_velocidad[i];
    }

    return 1;
}
int fusion_predecir_nominal(
    FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float dt,
    float gravedad_m_s2
)
{
    /* ENTRADAS: comprobar argumentos y tiempo entre muestras. */
    if (estado == NULL ||
        aceleracion_g == NULL || giro_dps == NULL ||
        !isfinite(dt) || dt <= 0.0f ||
        !isfinite(gravedad_m_s2) || gravedad_m_s2 <= 0.0f)
    {
        return 0;
    }

    /* Trabajar sobre una copia.
       Si una operacion falla, el estado original se conserva. */
    FusionEstado siguiente = *estado;

    if (!fusion_normalizar_cuaternion(siguiente.q_cuerpo_a_ned))
    {
        return 0;
    }

    /* IMU: conversion de unidades y correccion de sesgos. */
    float fuerza[3];
    float giro[3];

    fusion_preparar_imu(
        &siguiente, aceleracion_g, giro_dps, fuerza, giro
    );

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(fuerza[i]) || !isfinite(giro[i]))
        {
            return 0;
        }
    }

    /* GIROSCOPO: orientacion a mitad del intervalo. */
    float dq_mitad[4];
    float q_mitad[4];

    if (!fusion_incremento_rotacion(giro, 0.5f * dt, dq_mitad))
    {
        return 0;
    }

    fusion_producto_cuaterniones(
        siguiente.q_cuerpo_a_ned, dq_mitad, q_mitad
    );

    if (!fusion_normalizar_cuaternion(q_mitad))
    {
        return 0;
    }

    /* ACELEROMETRO: aceleracion en NED usando esa orientacion. */
    float aceleracion_ned[3];

    fusion_aceleracion_ned(
        q_mitad, fuerza, gravedad_m_s2, aceleracion_ned
    );

    /* MOVIMIENTO: actualizar posicion y velocidad. */
    if (!fusion_integrar_movimiento(
            &siguiente, aceleracion_ned, dt))
    {
        return 0;
    }

    /* GIROSCOPO: orientacion al final del intervalo completo. */
    float dq_completo[4];
    float q_final[4];

    if (!fusion_incremento_rotacion(giro, dt, dq_completo))
    {
        return 0;
    }

    fusion_producto_cuaterniones(
        siguiente.q_cuerpo_a_ned, dq_completo, q_final
    );

    if (!fusion_normalizar_cuaternion(q_final))
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        siguiente.q_cuerpo_a_ned[i] = q_final[i];
    }

    /* Los sesgos y P conservan por ahora sus valores.
       Guardar la prediccion cuando todo ha terminado bien. */
    *estado = siguiente;

    return 1;
}
/* =========================================================
   ESTADO DE ERROR: MATRIZ DEL PRODUCTO VECTORIAL
   ========================================================= */

void fusion_matriz_producto_vectorial(
    const float u[3],
    float S[3][3]
)
{
    const float x = u[0];
    const float y = u[1];
    const float z = u[2];

    S[0][0] = 0.0f;
    S[0][1] = -z;
    S[0][2] = y;

    S[1][0] = z;
    S[1][1] = 0.0f;
    S[1][2] = -x;

    S[2][0] = -y;
    S[2][1] = x;
    S[2][2] = 0.0f;
}
/* =========================================================
   ESTADO DE ERROR: EFECTO SOBRE LA ACELERACION
   ========================================================= */

void fusion_jacobianos_aceleracion(
    const float q[4],
    const float fuerza_corregida[3],
    float J_orientacion[3][3],
    float J_sesgo_acc[3][3]
)
{
    float C[3][3];
    float S[3][3];

    fusion_matriz_rotacion(q, C);
    fusion_matriz_producto_vectorial(fuerza_corregida, S);

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            /* Orientacion: J = -C * S. */
            float suma = 0.0f;

            for (unsigned int k = 0U; k < 3U; ++k)
            {
                suma += C[i][k] * S[k][j];
            }

            J_orientacion[i][j] = -suma;

            /* Sesgo del acelerometro: J = -C. */
            J_sesgo_acc[i][j] = -C[i][j];
        }
    }
}
/* =========================================================
   ESTADO DE ERROR: MODELO CONTINUO DE LOS 15 COMPONENTES
   ========================================================= */

void fusion_matriz_error_continuo(
    const float q[4],
    const float fuerza_corregida[3],
    const float giro_corregido[3],
    float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
)
{
    float J_orientacion[3][3];
    float J_sesgo_acc[3][3];
    float S_giro[3][3];

    fusion_jacobianos_aceleracion(
        q, fuerza_corregida,
        J_orientacion, J_sesgo_acc
    );

    fusion_matriz_producto_vectorial(
        giro_corregido, S_giro
    );

    /* Primero dejamos toda la matriz a cero. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            A[i][j] = 0.0f;
        }
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        /* Posicion: su derivada depende de la velocidad. */
        A[i][3U + i] = 1.0f;

        /* Orientacion: efecto del sesgo del giroscopio. */
        A[6U + i][12U + i] = -1.0f;

        for (unsigned int j = 0U; j < 3U; ++j)
        {
            /* Velocidad: efecto del error de orientacion. */
            A[3U + i][6U + j] = J_orientacion[i][j];

            /* Velocidad: efecto del sesgo del acelerometro. */
            A[3U + i][9U + j] = J_sesgo_acc[i][j];

            /* Orientacion: efecto del giro del cuerpo. */
            A[6U + i][6U + j] = -S_giro[i][j];
        }
    }

    /* Las filas de los sesgos permanecen a cero:
       su variacion se incorporara como ruido del proceso. */
}
/* =========================================================
   ESTADO DE ERROR: TRANSICION DURANTE UN INTERVALO
   ========================================================= */

int fusion_matriz_transicion(
    const float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    float dt,
    float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
)
{
    if (A == NULL || Phi == NULL ||
        !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    float B[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float resultado[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

    /* B = A * dt. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            B[i][j] = A[i][j] * dt;

            if (!isfinite(B[i][j]))
            {
                return 0;
            }
        }
    }

    /* Phi = I + B + 0.5 * B * B. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            float producto = 0.0f;

            for (unsigned int k = 0U; k < FUSION_DIM_ERROR; ++k)
            {
                producto += B[i][k] * B[k][j];
            }

            float identidad = (i == j) ? 1.0f : 0.0f;

            resultado[i][j] =
                identidad + B[i][j] + 0.5f * producto;

            if (!isfinite(resultado[i][j]))
            {
                return 0;
            }
        }
    }

    /* Guardar solo cuando todos los elementos son validos. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            Phi[i][j] = resultado[i][j];
        }
    }

    return 1;
}
/* =========================================================
   RUIDO: CONTRIBUCION DIRECTA AL ESTADO DE ERROR
   ========================================================= */

int fusion_intensidad_ruido_estado(
    const FusionRuido *ruido,
    float W[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
)
{
    if (ruido == NULL || W == NULL)
    {
        return 0;
    }

    /* Copiar y validar antes de modificar la salida. */
    const float intensidades[4] = {
        ruido->q_acc,
        ruido->q_giro,
        ruido->q_sesgo_acc,
        ruido->q_sesgo_giro
    };

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        if (!isfinite(intensidades[i]) ||
            intensidades[i] < 0.0f)
        {
            return 0;
        }
    }

    /* Sin correlaciones directas entre las fuentes. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            W[i][j] = 0.0f;
        }
    }

    /* Orden: posicion, velocidad, orientacion,
       sesgo acc y sesgo giro. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        W[3U + i][3U + i] = intensidades[0];
        W[6U + i][6U + i] = intensidades[1];
        W[9U + i][9U + i] = intensidades[2];
        W[12U + i][12U + i] = intensidades[3];
    }

    return 1;
}
/* =========================================================
   RUIDO: COVARIANZA ACUMULADA DURANTE EL INTERVALO
   ========================================================= */

int fusion_ruido_discreto(
    const float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    const FusionRuido *ruido,
    float dt,
    float Qd[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
)
{
    if (A == NULL || ruido == NULL || Qd == NULL ||
        !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    float W[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float resultado[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

    if (!fusion_intensidad_ruido_estado(ruido, W))
    {
        return 0;
    }

    /* Simpson: primer punto, tau = 0.
       Phi(0) es la identidad, por lo que queda W. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            resultado[i][j] = (dt / 6.0f) * W[i][j];
        }
    }

    /* Segundo punto: dt/2, peso 4.
       Tercer punto: dt, peso 1. */
    for (unsigned int punto = 0U; punto < 2U; ++punto)
    {
        float tiempo = (punto == 0U) ? 0.5f * dt : dt;
        float peso = (punto == 0U) ? 4.0f : 1.0f;
        float factor = (dt / 6.0f) * peso;

        if (!fusion_matriz_transicion(A, tiempo, Phi))
        {
            return 0;
        }

        /* Calculamos solo un triangulo y lo reflejamos. */
        for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
        {
            for (unsigned int j = i; j < FUSION_DIM_ERROR; ++j)
            {
                float suma = 0.0f;

                /* W es diagonal bajo nuestras hipotesis:
                   (Phi*W*Phi^T)[i][j]
                   = suma_k Phi[i][k]*W[k][k]*Phi[j][k]. */
                for (unsigned int k = 0U;
                     k < FUSION_DIM_ERROR; ++k)
                {
                    suma +=
                        Phi[i][k] * W[k][k] * Phi[j][k];
                }

                float valor =
                    resultado[i][j] + factor * suma;

                if (!isfinite(valor))
                {
                    return 0;
                }

                resultado[i][j] = valor;
                resultado[j][i] = valor;
            }
        }
    }

    /* Guardar solo si todos los calculos han terminado bien. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            Qd[i][j] = resultado[i][j];
        }
    }

    return 1;
}
/* =========================================================
   INCERTIDUMBRE: PROPAGACION DE LA MATRIZ P
   ========================================================= */

int fusion_propagar_covarianza(
    FusionEstado *estado,
    const float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR],
    const float Qd[FUSION_DIM_ERROR][FUSION_DIM_ERROR]
)
{
    if (estado == NULL || Phi == NULL || Qd == NULL)
    {
        return 0;
    }

    /* Validar las matrices de entrada. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            if (!isfinite(estado->P[i][j]) ||
                !isfinite(Phi[i][j]) ||
                !isfinite(Qd[i][j]))
            {
                return 0;
            }
        }
    }

    float auxiliar[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float nueva_P[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

    /* Primer producto: auxiliar = Phi * P. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            float suma = 0.0f;

            for (unsigned int k = 0U; k < FUSION_DIM_ERROR; ++k)
            {
                suma += Phi[i][k] * estado->P[k][j];
            }

            if (!isfinite(suma))
            {
                return 0;
            }

            auxiliar[i][j] = suma;
        }
    }

    /* Segundo producto y ruido:
       nueva_P = auxiliar * Phi^T + Qd. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            float suma = 0.0f;

            for (unsigned int k = 0U; k < FUSION_DIM_ERROR; ++k)
            {
                suma += auxiliar[i][k] * Phi[j][k];
            }

            nueva_P[i][j] = suma + Qd[i][j];

            if (!isfinite(nueva_P[i][j]))
            {
                return 0;
            }
        }
    }

    /* Eliminar pequenas asimetrias de redondeo. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        if (nueva_P[i][i] < 0.0f)
        {
            return 0;
        }

        for (unsigned int j = i + 1U;
             j < FUSION_DIM_ERROR; ++j)
        {
            float media =
                0.5f * nueva_P[i][j]
              + 0.5f * nueva_P[j][i];

            nueva_P[i][j] = media;
            nueva_P[j][i] = media;
        }
    }

    /* Guardar cuando toda la operacion es valida. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            estado->P[i][j] = nueva_P[i][j];
        }
    }

    return 1;
}
/* =========================================================
   PREDICCION CONJUNTA: ESTADO NOMINAL E INCERTIDUMBRE
   ========================================================= */

int fusion_predecir(
    FusionEstado *estado,
    const float aceleracion_g[3],
    const float giro_dps[3],
    float dt,
    float gravedad_m_s2,
    const FusionRuido *ruido
)
{
    if (estado == NULL ||
        aceleracion_g == NULL ||
        giro_dps == NULL ||
        ruido == NULL ||
        !isfinite(dt) || dt <= 0.0f ||
        !isfinite(gravedad_m_s2) || gravedad_m_s2 <= 0.0f)
    {
        return 0;
    }

    /* Conservar el original hasta completar ambas predicciones. */
    FusionEstado siguiente = *estado;

    if (!fusion_normalizar_cuaternion(
            siguiente.q_cuerpo_a_ned))
    {
        return 0;
    }

    /* Preparar las entradas del modelo de error. */
    float fuerza[3];
    float giro[3];

    fusion_preparar_imu(
        &siguiente, aceleracion_g, giro_dps, fuerza, giro
    );

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(fuerza[i]) || !isfinite(giro[i]))
        {
            return 0;
        }
    }

    /* Orientacion representativa del intervalo:
       el mismo criterio que en la prediccion nominal. */
    float dq_mitad[4];
    float q_mitad[4];

    if (!fusion_incremento_rotacion(
            giro, 0.5f * dt, dq_mitad))
    {
        return 0;
    }

    fusion_producto_cuaterniones(
        siguiente.q_cuerpo_a_ned, dq_mitad, q_mitad
    );

    if (!fusion_normalizar_cuaternion(q_mitad))
    {
        return 0;
    }

    /* Aproximar A como constante durante este intervalo. */
    float A[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float Phi[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float Qd[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

    fusion_matriz_error_continuo(
        q_mitad, fuerza, giro, A
    );

    if (!fusion_matriz_transicion((const float (*)[FUSION_DIM_ERROR])A, dt, Phi))
    {
        return 0;
    }

    if (!fusion_ruido_discreto((const float (*)[FUSION_DIM_ERROR])A, ruido, dt, Qd))
    {
        return 0;
    }

    /* Actualizar solo P dentro de la copia. */
    if (!fusion_propagar_covarianza(&siguiente, (const float (*)[FUSION_DIM_ERROR])Phi, (const float (*)[FUSION_DIM_ERROR])Qd))
    {
        return 0;
    }

    /* Actualizar movimiento y orientacion.
       Esta funcion conserva la P que acabamos de calcular. */
    if (!fusion_predecir_nominal(
            &siguiente,
            aceleracion_g,
            giro_dps,
            dt,
            gravedad_m_s2))
    {
        return 0;
    }

    /* Guardar conjuntamente estado nominal y covarianza. */
    *estado = siguiente;

    return 1;
}
/* =========================================================
   CORRECCION: GANANCIA E INNOVACION DE UNA MEDIDA ESCALAR
   ========================================================= */

int fusion_preparar_correccion_escalar(
    const FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    float innovacion,
    float R,
    FusionCorreccionEscalar *correccion
)
{
    if (estado == NULL || H == NULL || correccion == NULL ||
        !isfinite(innovacion) ||
        !isfinite(R) || R <= 0.0f)
    {
        return 0;
    }

    /* Validar H y la covarianza de entrada. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        if (!isfinite(H[i]))
        {
            return 0;
        }

        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            if (!isfinite(estado->P[i][j]))
            {
                return 0;
            }
        }
    }

    /* Vector auxiliar: PHt = P * H^T. */
    float PHt[FUSION_DIM_ERROR];

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        float suma = 0.0f;

        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            suma += estado->P[i][j] * H[j];
        }

        if (!isfinite(suma))
        {
            return 0;
        }

        PHt[i] = suma;
    }

    /* Varianza de la innovacion: S = H * P * H^T + R. */
    float S = R;

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        S += H[i] * PHt[i];
    }

    if (!isfinite(S) || S <= 0.0f)
    {
        return 0;
    }

    FusionCorreccionEscalar resultado = {0};

    resultado.innovacion = innovacion;
    resultado.varianza_innovacion = S;

    /* NIS = r^2 / S.
       Normalizar primero evita calcular r^2 directamente. */
    float innovacion_normalizada = innovacion / sqrtf(S);
    resultado.nis =
        innovacion_normalizada * innovacion_normalizada;

    if (!isfinite(resultado.nis))
    {
        return 0;
    }

    /* Ganancia: K = P * H^T / S. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        resultado.ganancia[i] = PHt[i] / S;

        if (!isfinite(resultado.ganancia[i]))
        {
            return 0;
        }
    }

    *correccion = resultado;

    return 1;
}
/* =========================================================
   CORRECCION: INYECCION DEL ERROR EN EL ESTADO NOMINAL
   ========================================================= */

int fusion_inyectar_error_nominal(
    FusionEstado *estado,
    const float delta[FUSION_DIM_ERROR]
)
{
    if (estado == NULL || delta == NULL)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        if (!isfinite(delta[i]))
        {
            return 0;
        }
    }

    FusionEstado siguiente = *estado;

    /* Variables con correccion aditiva. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        siguiente.posicion_ned_m[i] += delta[i];
        siguiente.velocidad_ned_m_s[i] += delta[3U + i];
        siguiente.sesgo_acc_m_s2[i] += delta[9U + i];
        siguiente.sesgo_giro_rad_s[i] += delta[12U + i];

        if (!isfinite(siguiente.posicion_ned_m[i]) ||
            !isfinite(siguiente.velocidad_ned_m_s[i]) ||
            !isfinite(siguiente.sesgo_acc_m_s2[i]) ||
            !isfinite(siguiente.sesgo_giro_rad_s[i]))
        {
            return 0;
        }
    }

    /* Orientacion: convertir el vector de correccion
       angular en un cuaternion de rotacion. */
    float angulo = hypotf(
        hypotf(delta[6], delta[7]),
        delta[8]
    );

    if (!isfinite(angulo))
    {
        return 0;
    }

    float factor;

    if (angulo < 1.0e-6f)
    {
        /* Limite de sin(angulo/2)/angulo para angulo -> 0. */
        factor = 0.5f;
    }
    else
    {
        factor = sinf(0.5f * angulo) / angulo;
    }

    float dq[4] = {
        cosf(0.5f * angulo),
        factor * delta[6],
        factor * delta[7],
        factor * delta[8]
    };

    if (!fusion_normalizar_cuaternion(
            siguiente.q_cuerpo_a_ned))
    {
        return 0;
    }

    float q_corregido[4];

    fusion_producto_cuaterniones(
        siguiente.q_cuerpo_a_ned, dq, q_corregido
    );

    if (!fusion_normalizar_cuaternion(q_corregido))
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 4U; ++i)
    {
        siguiente.q_cuerpo_a_ned[i] = q_corregido[i];
    }

    *estado = siguiente;

    return 1;
}
/* =========================================================
   CORRECCION: COVARIANZA MEDIANTE LA FORMA DE JOSEPH
   ========================================================= */

int fusion_corregir_covarianza_escalar(
    FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    const float K[FUSION_DIM_ERROR],
    float R
)
{
    if (estado == NULL || H == NULL || K == NULL ||
        !isfinite(R) || R <= 0.0f)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        if (!isfinite(H[i]) || !isfinite(K[i]))
        {
            return 0;
        }
    }

    float M[FUSION_DIM_ERROR][FUSION_DIM_ERROR];
    float U[FUSION_DIM_ERROR][FUSION_DIM_ERROR];

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        for (unsigned int j = 0U; j < FUSION_DIM_ERROR; ++j)
        {
            float identidad = (i == j) ? 1.0f : 0.0f;

            /* M = I - K*H. */
            M[i][j] = identidad - K[i] * H[j];

            /* U = K*R*K^T, con R escalar. */
            U[i][j] = K[i] * R * K[j];
        }
    }

    /* Reutilizar el calculo M*P*M^T + U.
       La funcion comprueba los valores finitos,
       simetriza y guarda solo si termina correctamente. */
    return fusion_propagar_covarianza(estado, (const float (*)[FUSION_DIM_ERROR])M, (const float (*)[FUSION_DIM_ERROR])U);
}
/* =========================================================
   CORRECCION: REINICIO DE LAS COORDENADAS DEL ERROR
   ========================================================= */

int fusion_reiniciar_covarianza(
    FusionEstado *estado,
    const float delta_theta[3]
)
{
    if (estado == NULL || delta_theta == NULL)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(delta_theta[i]))
        {
            return 0;
        }
    }

    float S[3][3];
    fusion_matriz_producto_vectorial(delta_theta, S);

    float J[FUSION_DIM_ERROR][FUSION_DIM_ERROR] = {0};
    float cero[FUSION_DIM_ERROR][FUSION_DIM_ERROR] = {0};

    /* Identidad para posicion, velocidad y sesgos. */
    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        J[i][i] = 1.0f;
    }

    /* Bloque angular: I - 0.5 * [delta_theta]_x. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        for (unsigned int j = 0U; j < 3U; ++j)
        {
            J[6U + i][6U + j] -= 0.5f * S[i][j];
        }
    }

    /* Cambio de coordenadas: P = J * P * J^T.
       No se anade ruido en esta operacion. */
    return fusion_propagar_covarianza(estado, (const float (*)[FUSION_DIM_ERROR])J, (const float (*)[FUSION_DIM_ERROR])cero);
}
/* =========================================================
   CORRECCION ESCALAR COMPLETA
   ========================================================= */

FusionResultadoCorreccion fusion_corregir_escalar(
    FusionEstado *estado,
    const float H[FUSION_DIM_ERROR],
    float innovacion,
    float R,
    float umbral_nis
)
{
    if (estado == NULL || H == NULL ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return FUSION_CORRECCION_ERROR;
    }

    FusionEstado siguiente = *estado;
    FusionCorreccionEscalar correccion;

    /* 1. Ganancia e innovacion normalizada.
       Esta funcion valida tambien innovacion, R, H y P. */
    if (!fusion_preparar_correccion_escalar(
            &siguiente, H, innovacion, R, &correccion))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* 2. Comprobar compatibilidad de la medida. */
    if (correccion.nis > umbral_nis)
    {
        return FUSION_CORRECCION_RECHAZADA;
    }

    /* 3. Correccion propuesta del estado de error. */
    float delta[FUSION_DIM_ERROR];

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        delta[i] =
            correccion.ganancia[i] * correccion.innovacion;

        if (!isfinite(delta[i]))
        {
            return FUSION_CORRECCION_ERROR;
        }
    }

    /* 4. Joseph: usar la covarianza previa a la correccion. */
    if (!fusion_corregir_covarianza_escalar(
            &siguiente, H, correccion.ganancia, R))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* 5. Incorporar la correccion al estado nominal. */
    if (!fusion_inyectar_error_nominal(&siguiente, delta))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* 6. Expresar P respecto a la nueva orientacion nominal. */
    if (!fusion_reiniciar_covarianza(
            &siguiente, &delta[6]))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* Guardar solo cuando toda la correccion esta completa. */
    *estado = siguiente;

    return FUSION_CORRECCION_APLICADA;
}
/* =========================================================
   GNSS: CORRECCION SECUENCIAL DE POSICION Y VELOCIDAD
   ========================================================= */

int fusion_corregir_gnss(
    FusionEstado *estado,
    const FusionMedidaGnss *medida,
    float umbral_nis
)
{
    if (estado == NULL || medida == NULL ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return -1;
    }

    /* Validar todas las componentes seleccionadas. */
    for (unsigned int componente = 0U;
         componente < 6U; ++componente)
    {
        if (medida->usar[componente] > 1U)
        {
            return -1;
        }

        if (medida->usar[componente] == 0U)
        {
            continue;
        }

        if (!isfinite(medida->valor[componente]) ||
            !isfinite(medida->varianza[componente]) ||
            medida->varianza[componente] <= 0.0f)
        {
            return -1;
        }
    }

    FusionEstado siguiente = *estado;
    int aplicadas = 0;

    for (unsigned int componente = 0U;
         componente < 6U; ++componente)
    {
        if (medida->usar[componente] == 0U)
        {
            continue;
        }

        /* H selecciona una posicion o una velocidad. */
        float H[FUSION_DIM_ERROR] = {0};
        H[componente] = 1.0f;

        /* Usar la estimacion vigente tras las
           correcciones anteriores de este mensaje GNSS. */
        float prediccion;

        if (componente < 3U)
        {
            prediccion =
                siguiente.posicion_ned_m[componente];
        }
        else
        {
            prediccion =
                siguiente.velocidad_ned_m_s[componente - 3U];
        }

        float innovacion =
            medida->valor[componente] - prediccion;

        FusionResultadoCorreccion resultado =
            fusion_corregir_escalar(
                &siguiente,
                H,
                innovacion,
                medida->varianza[componente],
                umbral_nis
            );

        if (resultado == FUSION_CORRECCION_ERROR)
        {
            return -1;
        }

        if (resultado == FUSION_CORRECCION_APLICADA)
        {
            ++aplicadas;
        }

        /* Si se rechaza por NIS, continuar con la siguiente. */
    }

    if (aplicadas > 0)
    {
        *estado = siguiente;
    }

    return aplicadas;
}
int fusion_combinar_alturas(
    float altura_5C,
    float varianza_5C,
    float altura_5D,
    float varianza_5D,
    float covarianza,
    float *altura_combinada,
    float *varianza_combinada
)
{
    /* Comprobar argumentos y valores individuales. */
    if (altura_combinada == NULL ||
        varianza_combinada == NULL ||
        altura_combinada == varianza_combinada ||
        !isfinite(altura_5C) ||
        !isfinite(altura_5D) ||
        !isfinite(varianza_5C) || varianza_5C <= 0.0f ||
        !isfinite(varianza_5D) || varianza_5D <= 0.0f ||
        !isfinite(covarianza))
    {
        return 0;
    }

    /* Una covarianza valida debe cumplir:
       covarianza^2 <= varianza_5C * varianza_5D.

       Usamos double en estos productos para evitar
       desbordamientos intermedios con entradas float. */
    double cov = (double)covarianza;
    double producto_varianzas =
        (double)varianza_5C * (double)varianza_5D;

    if (cov * cov > producto_varianzas)
    {
        return 0;
    }

    /* Promedio de las alturas y varianza de ese promedio. */
    float altura =
        0.5f * altura_5C + 0.5f * altura_5D;

    float varianza = (float)(
        0.25 * (
            (double)varianza_5C +
            (double)varianza_5D +
            2.0 * cov
        )
    );

    /* La correccion escalar requiere varianza positiva. */
    if (!isfinite(altura) ||
        !isfinite(varianza) || varianza <= 0.0f)
    {
        return 0;
    }

    *altura_combinada = altura;
    *varianza_combinada = varianza;

    return 1;
}
FusionResultadoCorreccion fusion_corregir_barometro(
    FusionEstado *estado,
    float altura_m,
    float varianza_m2,
    float umbral_nis
)
{
    /* Validar los argumentos antes de acceder al estado. */
    if (estado == NULL ||
        !isfinite(altura_m) ||
        !isfinite(varianza_m2) || varianza_m2 <= 0.0f ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* Modelo de altura: h = -p_D. */
    float H[FUSION_DIM_ERROR] = {0};
    H[2] = -1.0f;

    /* Medida menos prediccion: h_baro - (-p_D). */
    float innovacion =
        altura_m + estado->posicion_ned_m[2];

    return fusion_corregir_escalar(
        estado,
        H,
        innovacion,
        varianza_m2,
        umbral_nis
    );
}
FusionResultadoCorreccion fusion_corregir_barometros(
    FusionEstado *estado,
    const FusionMedidasBarometros *medidas,
    float covarianza_m2,
    float umbral_diferencia,
    float umbral_nis
)
{
    if (estado == NULL || medidas == NULL ||
        !isfinite(umbral_diferencia) ||
        umbral_diferencia <= 0.0f ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* Revisar solo los datos marcados para su uso. */
    for (unsigned int i = 0U; i < 2U; ++i)
    {
        if (medidas->usar[i] > 1U)
        {
            return FUSION_CORRECCION_ERROR;
        }

        if (medidas->usar[i] &&
            (!isfinite(medidas->altura_m[i]) ||
             !isfinite(medidas->varianza_m2[i]) ||
             medidas->varianza_m2[i] <= 0.0f))
        {
            return FUSION_CORRECCION_ERROR;
        }
    }

    /* Sin referencias barometricas disponibles. */
    if (!medidas->usar[0] && !medidas->usar[1])
    {
        return FUSION_CORRECCION_RECHAZADA;
    }

    float altura;
    float varianza;

    if (medidas->usar[0] && medidas->usar[1])
    {
        /* Preparar el promedio y validar la covarianza.
           Todavia no se modifica el estimador. */
        if (!fusion_combinar_alturas(
                medidas->altura_m[0],
                medidas->varianza_m2[0],
                medidas->altura_m[1],
                medidas->varianza_m2[1],
                covarianza_m2,
                &altura,
                &varianza))
        {
            return FUSION_CORRECCION_ERROR;
        }

        /* Comprobar la diferencia entre las dos alturas.
           Double evita desbordamientos en los productos. */
        double diferencia =
            (double)medidas->altura_m[0] -
            (double)medidas->altura_m[1];

        double varianza_diferencia =
            (double)medidas->varianza_m2[0] +
            (double)medidas->varianza_m2[1] -
            2.0 * (double)covarianza_m2;

        if (varianza_diferencia < 0.0)
        {
            return FUSION_CORRECCION_ERROR;
        }

        if (diferencia * diferencia >
            (double)umbral_diferencia * varianza_diferencia)
        {
            return FUSION_CORRECCION_RECHAZADA;
        }
    }
    else
    {
        /* Utilizar directamente el unico sensor disponible. */
        unsigned int i = medidas->usar[0] ? 0U : 1U;

        altura = medidas->altura_m[i];
        varianza = medidas->varianza_m2[i];
    }

    /* Una unica actualizacion, con la medida preparada. */
    return fusion_corregir_barometro(
        estado, altura, varianza, umbral_nis
    );
}
/* MAGNETOMETRO: CAMPO ESPERADO EN LOS EJES DEL CUERPO. */

void fusion_predecir_campo_magnetico(
    const float q[4],
    const float campo_ned[3],
    float campo_cuerpo[3]
)
{
    float C[3][3];
    fusion_matriz_rotacion(q, C);

    float prediccion[3];

    /* Campo en el cuerpo = C transpuesta * campo en NED. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        prediccion[i] =
            C[0][i] * campo_ned[0]
          + C[1][i] * campo_ned[1]
          + C[2][i] * campo_ned[2];
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        campo_cuerpo[i] = prediccion[i];
    }
}
/* MAGNETOMETRO: SENSIBILIDAD A LOS ERRORES DE ORIENTACION. */

int fusion_fila_medida_magnetica(
    const float campo_predicho[3],
    unsigned int componente,
    float H[FUSION_DIM_ERROR]
)
{
    if (campo_predicho == NULL || H == NULL ||
        componente >= 3U)
    {
        return 0;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(campo_predicho[i]))
        {
            return 0;
        }
    }

    float S[3][3];
    fusion_matriz_producto_vectorial(campo_predicho, S);

    /* Los demas bloques de la fila permanecen a cero. */
    float fila[FUSION_DIM_ERROR] = {0};

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        fila[6U + i] = S[componente][i];
    }

    for (unsigned int i = 0U; i < FUSION_DIM_ERROR; ++i)
    {
        H[i] = fila[i];
    }

    return 1;
}
int fusion_comprobar_intensidad_magnetica(
    const float campo_medido[3],
    const float campo_ned[3],
    float limite_diferencia
)
{
    if (campo_medido == NULL || campo_ned == NULL ||
        !isfinite(limite_diferencia) ||
        limite_diferencia < 0.0f)
    {
        return -1;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(campo_medido[i]) ||
            !isfinite(campo_ned[i]))
        {
            return -1;
        }
    }

    float intensidad_medida = hypotf(
        hypotf(campo_medido[0], campo_medido[1]),
        campo_medido[2]
    );

    float intensidad_referencia = hypotf(
        hypotf(campo_ned[0], campo_ned[1]),
        campo_ned[2]
    );

    if (!isfinite(intensidad_medida) ||
        !isfinite(intensidad_referencia) ||
        intensidad_referencia <= 0.0f)
    {
        return -1;
    }

    if (intensidad_medida <= 0.0f)
    {
        return 0;
    }

    float diferencia = fabsf(
        intensidad_medida - intensidad_referencia
    );

    return (diferencia <= limite_diferencia) ? 1 : 0;
}
FusionResultadoCorreccion fusion_corregir_componente_magnetica(
    FusionEstado *estado,
    const float campo_ned[3],
    unsigned int componente,
    float medida,
    float varianza,
    float umbral_nis
)
{
    if (estado == NULL || campo_ned == NULL ||
        componente >= 3U ||
        !isfinite(medida) ||
        !isfinite(varianza) || varianza <= 0.0f ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return FUSION_CORRECCION_ERROR;
    }

    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(campo_ned[i]))
        {
            return FUSION_CORRECCION_ERROR;
        }
    }

    FusionEstado siguiente = *estado;

    if (!fusion_normalizar_cuaternion(
            siguiente.q_cuerpo_a_ned))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* 1. Campo esperado con la orientacion vigente. */
    float campo_predicho[3];

    fusion_predecir_campo_magnetico(
        siguiente.q_cuerpo_a_ned,
        campo_ned,
        campo_predicho
    );

    /* 2. Sensibilidad de la componente seleccionada. */
    float H[FUSION_DIM_ERROR];

    if (!fusion_fila_medida_magnetica(
            campo_predicho, componente, H))
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* 3. Medida menos prediccion. */
    float innovacion = medida - campo_predicho[componente];

    /* 4. Aplicar la rutina comun. */
    FusionResultadoCorreccion resultado =
        fusion_corregir_escalar(
            &siguiente,
            H,
            innovacion,
            varianza,
            umbral_nis
        );

    if (resultado == FUSION_CORRECCION_APLICADA)
    {
        *estado = siguiente;
    }

    return resultado;
}
int fusion_corregir_magnetometro(
    FusionEstado *estado,
    const float campo_medido[3],
    const float campo_ned[3],
    const float varianza[3],
    float limite_intensidad,
    float umbral_nis
)
{
    if (estado == NULL || campo_medido == NULL ||
        campo_ned == NULL || varianza == NULL ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return -1;
    }

    /* Validar las tres varianzas antes de corregir. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        if (!isfinite(varianza[i]) || varianza[i] <= 0.0f)
        {
            return -1;
        }
    }

    /* 1. Comprobar la intensidad de la lectura completa. */
    int compatible = fusion_comprobar_intensidad_magnetica(
        campo_medido,
        campo_ned,
        limite_intensidad
    );

    if (compatible < 0)
    {
        return -1;
    }

    if (compatible == 0)
    {
        return 0;
    }

    /* Trabajar sobre una copia hasta terminar. */
    FusionEstado siguiente = *estado;
    int aplicadas = 0;

    /* 2. Procesar X, Y y Z en ese orden. */
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        FusionResultadoCorreccion resultado =
            fusion_corregir_componente_magnetica(
                &siguiente,
                campo_ned,
                i,
                campo_medido[i],
                varianza[i],
                umbral_nis
            );

        if (resultado == FUSION_CORRECCION_ERROR)
        {
            return -1;
        }

        if (resultado == FUSION_CORRECCION_APLICADA)
        {
            ++aplicadas;
        }

        /* Una componente rechazada por NIS se omite. */
    }

    /* 3. Guardar las correcciones aceptadas. */
    if (aplicadas > 0)
    {
        *estado = siguiente;
    }

    return aplicadas;
}
int fusion_inicializar_estimador(
    FusionEstimador *estimador,
    const float q_inicial[4],
    const float sigma_inicial[FUSION_DIM_ERROR],
    double tiempo_inicial_s
)
{
    if (estimador == NULL || !isfinite(tiempo_inicial_s))
    {
        return 0;
    }

    FusionEstimador inicial = {0};

    if (!fusion_inicializar(
            &inicial.estado, q_inicial, sigma_inicial))
    {
        return 0;
    }

    inicial.tiempo_s = tiempo_inicial_s;

    *estimador = inicial;

    return 1;
}
int fusion_predecir_hasta(
    FusionEstimador *estimador,
    const float aceleracion_g[3],
    const float giro_dps[3],
    double tiempo_muestra_s,
    float dt_max_s,
    float gravedad_m_s2,
    const FusionRuido *ruido
)
{
    if (estimador == NULL ||
        !isfinite(tiempo_muestra_s) ||
        !isfinite(estimador->tiempo_s) ||
        !isfinite(dt_max_s) || dt_max_s <= 0.0f)
    {
        return 0;
    }

    /* Diferencia entre el instante nuevo y el del estado. */
    double intervalo =
        tiempo_muestra_s - estimador->tiempo_s;

    /* Rechazar muestras repetidas, antiguas
       o separadas por un intervalo excesivo. */
    if (!isfinite(intervalo) ||
        intervalo <= 0.0 ||
        intervalo > (double)dt_max_s)
    {
        return 0;
    }

    float dt = (float)intervalo;

    if (!isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    /* Esta funcion conserva el estado si falla. */
    if (!fusion_predecir(
            &estimador->estado,
            aceleracion_g,
            giro_dps,
            dt,
            gravedad_m_s2,
            ruido))
    {
        return 0;
    }

    /* La prediccion ha terminado: actualizar su instante. */
    estimador->tiempo_s = tiempo_muestra_s;

    return 1;
}
/* Control temporal para medidas sincronizadas.
   Devuelve 1 si se puede procesar y 0 en otro caso.
   No modifica el estimador. */
static int fusion_medida_procesable(
    double tiempo_medida_s,
    double tiempo_estado_s,
    double ultimo_tiempo_s,
    unsigned char ya_procesada
)
{
    if (!isfinite(tiempo_medida_s) ||
        !isfinite(tiempo_estado_s) ||
        ya_procesada > 1U)
    {
        return 0;
    }

    /* En esta integracion sincronizada, ambos instantes
       deben proceder de la misma marca temporal. */
    if (tiempo_medida_s != tiempo_estado_s)
    {
        return 0;
    }

    if (ya_procesada)
    {
        if (!isfinite(ultimo_tiempo_s) ||
            tiempo_medida_s <= ultimo_tiempo_s)
        {
            return 0;
        }
    }

    return 1;
}
int fusion_procesar_gnss(
    FusionEstimador *estimador,
    const FusionMedidaGnss *medida,
    double tiempo_medida_s,
    float umbral_nis
)
{
    if (estimador == NULL || medida == NULL ||
        !isfinite(tiempo_medida_s) ||
        !isfinite(estimador->tiempo_s) ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f ||
        estimador->gnss_procesado > 1U)
    {
        return -1;
    }

    if (estimador->gnss_procesado &&
        !isfinite(estimador->ultimo_gnss_s))
    {
        return -1;
    }

    /* 1. Comprobar sincronizacion y evitar repeticiones. */
    if (!fusion_medida_procesable(
            tiempo_medida_s,
            estimador->tiempo_s,
            estimador->ultimo_gnss_s,
            estimador->gnss_procesado))
    {
        return 0;
    }

    /* 2. Aplicar la correccion GNSS existente. */
    int aplicadas = fusion_corregir_gnss(
        &estimador->estado,
        medida,
        umbral_nis
    );

    if (aplicadas < 0)
    {
        return -1;
    }

    /* 3. Registrar que esta lectura ya se ha evaluado. */
    estimador->ultimo_gnss_s = tiempo_medida_s;
    estimador->gnss_procesado = 1U;

    return aplicadas;
}
FusionResultadoCorreccion fusion_procesar_barometros(
    FusionEstimador *estimador,
    const FusionMedidasBarometros *medidas,
    const double tiempo_medida_s[2],
    float covarianza_m2,
    float umbral_diferencia,
    float umbral_nis
)
{
    if (estimador == NULL || medidas == NULL ||
        tiempo_medida_s == NULL ||
        !isfinite(estimador->tiempo_s) ||
        !isfinite(umbral_diferencia) ||
        umbral_diferencia <= 0.0f ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f)
    {
        return FUSION_CORRECCION_ERROR;
    }

    /* Copia local para seleccionar las lecturas.
       No modificamos los datos recibidos. */
    FusionMedidasBarometros seleccionadas = *medidas;

    for (unsigned int i = 0U; i < 2U; ++i)
    {
        if (medidas->usar[i] > 1U)
        {
            return FUSION_CORRECCION_ERROR;
        }

        if (!medidas->usar[i])
        {
            continue;
        }

        if (!isfinite(tiempo_medida_s[i]) ||
            estimador->baro_procesado[i] > 1U)
        {
            return FUSION_CORRECCION_ERROR;
        }

        if (estimador->baro_procesado[i] &&
            !isfinite(estimador->ultimo_baro_s[i]))
        {
            return FUSION_CORRECCION_ERROR;
        }

        seleccionadas.usar[i] =
            (unsigned char)fusion_medida_procesable(
                tiempo_medida_s[i],
                estimador->tiempo_s,
                estimador->ultimo_baro_s[i],
                estimador->baro_procesado[i]
            );
    }

    /* Ninguna lectura supera el control temporal. */
    if (!seleccionadas.usar[0] && !seleccionadas.usar[1])
    {
        return FUSION_CORRECCION_RECHAZADA;
    }

    /* La rutina existente selecciona o combina alturas. */
    FusionResultadoCorreccion resultado =
        fusion_corregir_barometros(
            &estimador->estado,
            &seleccionadas,
            covarianza_m2,
            umbral_diferencia,
            umbral_nis
        );

    if (resultado == FUSION_CORRECCION_ERROR)
    {
        return resultado;
    }

    /* Registrar solo las lecturas que hemos evaluado. */
    for (unsigned int i = 0U; i < 2U; ++i)
    {
        if (seleccionadas.usar[i])
        {
            estimador->ultimo_baro_s[i] = tiempo_medida_s[i];
            estimador->baro_procesado[i] = 1U;
        }
    }

    return resultado;
}
int fusion_procesar_magnetometro(
    FusionEstimador *estimador,
    const float campo_medido[3],
    const float campo_ned[3],
    const float varianza[3],
    double tiempo_medida_s,
    float limite_intensidad,
    float umbral_nis
)
{
    if (estimador == NULL ||
        campo_medido == NULL || campo_ned == NULL ||
        varianza == NULL ||
        !isfinite(tiempo_medida_s) ||
        !isfinite(estimador->tiempo_s) ||
        !isfinite(limite_intensidad) ||
        limite_intensidad < 0.0f ||
        !isfinite(umbral_nis) || umbral_nis <= 0.0f ||
        estimador->mag_procesado > 1U)
    {
        return -1;
    }

    if (estimador->mag_procesado &&
        !isfinite(estimador->ultimo_mag_s))
    {
        return -1;
    }

    /* 1. Comprobar sincronizacion y evitar repeticiones. */
    if (!fusion_medida_procesable(
            tiempo_medida_s,
            estimador->tiempo_s,
            estimador->ultimo_mag_s,
            estimador->mag_procesado))
    {
        return 0;
    }

    /* 2. Comprobar intensidad y corregir las componentes. */
    int aplicadas = fusion_corregir_magnetometro(
        &estimador->estado,
        campo_medido,
        campo_ned,
        varianza,
        limite_intensidad,
        umbral_nis
    );

    if (aplicadas < 0)
    {
        return -1;
    }

    /* 3. Registrar la lectura evaluada. */
    estimador->ultimo_mag_s = tiempo_medida_s;
    estimador->mag_procesado = 1U;

    return aplicadas;
}