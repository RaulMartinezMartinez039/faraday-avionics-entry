#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "fusion.h"

/* Escenario sintetico inspirado en ISPIDA, no reconstruccion de telemetria.
   Compilar junto a fusion.c; no junto a simulacion.c (ambos tienen main).
   IMU/barometros: 100 Hz; GNSS: 10 Hz; magnetometro: 20 Hz.
   Ruido, sesgos y campo sintetico conservados del primer escenario.
   rand() puede generar realizaciones diferentes segun la biblioteca de C. */

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


/* Parametros publicados: https://astg.at/projects/ispida
   Aproximaciones: masa final = masa seca, empuje medio, CD constante,
   atmosfera exponencial, gravedad constante y aire en reposo.
   No se modelan momentos aerodinamicos, rail, latencias ni saturaciones. */
static const double PI_SIM = 3.14159265358979323846;
static const double DESPEGUE_S = 2.0;
static const double COMBUSTION_S = 9.5;
static const double MASA_INICIAL = 47.0;
static const double MASA_FINAL = 29.6;
static const double IMPULSO_NS = 32100.0;
static const double DIAMETRO_M = 0.152;
static const double CD_SIM = 0.5;
#define SUBPASOS 10U /* Par: permite evaluar exactamente el punto medio. */

typedef struct
{
    /* pN,pE,pD,vN,vE,vD,q0,q1,q2,q3; referencia en doble precision. */
    double y[10];
    double a[3];
    double omega[3];
} SimulacionReferencia;

static void producto(const double a[4], const double b[4], double r[4])
{
    double t[4] = {
        a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
        a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
        a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
        a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
    for (unsigned i=0; i<4; ++i) r[i]=t[i];
}

static void matriz(const double q[4], double C[3][3])
{
    double w=q[0], x=q[1], y=q[2], z=q[3];
    C[0][0]=1-2*(y*y+z*z); C[0][1]=2*(x*y-w*z); C[0][2]=2*(x*z+w*y);
    C[1][0]=2*(x*y+w*z); C[1][1]=1-2*(x*x+z*z); C[1][2]=2*(y*z-w*x);
    C[2][0]=2*(x*z-w*y); C[2][1]=2*(y*z+w*x); C[2][2]=1-2*(x*x+y*y);
}

static double masa(double t)
{
    double u=fmax(0.0, fmin(1.0, (t-DESPEGUE_S)/COMBUSTION_S));
    return MASA_INICIAL+(MASA_FINAL-MASA_INICIAL)*u;
}

/* fase 0: plataforma; 1: motor encendido; 2: vuelo sin propulsion.
   Se fija por intervalo para no mezclar fuerzas en el instante de apagado. */
static void derivada(double t, const double y[10], int fase,
                     double dy[10], double omega[3])
{
    for (unsigned i=0; i<10; ++i) dy[i]=0.0;
    for (unsigned i=0; i<3; ++i) omega[i]=0.0;
    if (fase==0) return; /* La plataforma compensa la gravedad. */
    double tau=t-DESPEGUE_S, rad=PI_SIM/180.0;
    double phi=10.0*rad*tau;
    omega[0]=10.0*rad;
    omega[1]=-0.2*rad*cos(phi);
    omega[2]= 0.2*rad*sin(phi);
    double C[3][3]; matriz(y+6,C);
    double h=fmax(0.0,-y[2]);
    double rho=1.225*exp(-h/8500.0);
    double v=hypot(hypot(y[3],y[4]),y[5]);
    double arrastre=0.5*rho*CD_SIM*(PI_SIM*DIAMETRO_M*DIAMETRO_M/4.0)*v/masa(t);
    double empuje=(fase==1 ? IMPULSO_NS/COMBUSTION_S : 0.0);
    for (unsigned i=0; i<3; ++i) {
        dy[i]=y[i+3];
        dy[i+3]=empuje/masa(t)*C[i][0]-arrastre*y[i+3];
    }
    dy[5]+=(double)G_SIM;
    double wq[4]={0,omega[0],omega[1],omega[2]}, dq[4];
    producto(y+6,wq,dq);
    for (unsigned i=0; i<4; ++i) dy[i+6]=0.5*dq[i];
}

static void actualizar_auxiliares(SimulacionReferencia *r, double t, int fase)
{
    double dy[10]; derivada(t,r->y,fase,dy,r->omega);
    for (unsigned i=0; i<3; ++i) r->a[i]=dy[i+3];
}

/* RK4 independiente del integrador del estimador, con subpasos de 1 ms. */
static void paso_referencia(SimulacionReferencia *r, double t, double dt, int fase)
{
    double k1[10],k2[10],k3[10],k4[10],z[10],w[3];
    derivada(t,r->y,fase,k1,w);
    for (unsigned i=0; i<10; ++i) z[i]=r->y[i]+0.5*dt*k1[i];
    derivada(t+0.5*dt,z,fase,k2,w);
    for (unsigned i=0; i<10; ++i) z[i]=r->y[i]+0.5*dt*k2[i];
    derivada(t+0.5*dt,z,fase,k3,w);
    for (unsigned i=0; i<10; ++i) z[i]=r->y[i]+dt*k3[i];
    derivada(t+dt,z,fase,k4,w);
    for (unsigned i=0; i<10; ++i)
        r->y[i]+=dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6.0;
    double n=hypot(hypot(r->y[6],r->y[7]),hypot(r->y[8],r->y[9]));
    for (unsigned i=6; i<10; ++i) r->y[i]/=n;
}

static SimulacionReferencia referencia_inicial(void)
{
    SimulacionReferencia r={0};
    double az=30.0*PI_SIM/180.0, pitch=88.0*PI_SIM/180.0;
    double qz[4]={cos(az/2),0,0,sin(az/2)};
    double qy[4]={cos(pitch/2),0,sin(pitch/2),0};
    producto(qz,qy,r.y+6);
    return r;
}

static void simulacion_generar_imu(const SimulacionReferencia *r,
                                 float aceleracion_g[3], float giro_dps[3])
{
    double C[3][3]; matriz(r->y+6,C);
    for (unsigned i=0; i<3; ++i) {
        double f=0.0;
        for (unsigned j=0; j<3; ++j)
            f+=C[j][i]*(r->a[j]-(j==2 ? (double)G_SIM : 0.0));
        aceleracion_g[i]=(float)((f+SESGO_ACC_REAL[i]+SIGMA_ACC*simulacion_normal())/G_SIM);
        giro_dps[i]=(float)((r->omega[i]+SESGO_GIRO_REAL[i]+SIGMA_GIRO*simulacion_normal())*180.0/PI_SIM);
    }
}
static FusionMedidaGnss simulacion_generar_gnss(
    const SimulacionReferencia *referencia)
{
    FusionMedidaGnss medida = {0};

    const float posicion_ned[3] = {
        (float)referencia->y[0],
        (float)referencia->y[1],
        (float)referencia->y[2]
    };

    const float velocidad_ned[3] = {
        (float)referencia->y[3],
        (float)referencia->y[4],
        (float)referencia->y[5]
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
            (float)(-referencia->y[2])
            + SIGMA_BARO * simulacion_normal();

        medidas.varianza_m2[i] =
            SIGMA_BARO * SIGMA_BARO;

        medidas.usar[i] = 1U;
    }

    return medidas;
}

static void simulacion_generar_magnetometro(
    const SimulacionReferencia *referencia,
    float campo_medido[3],
    float varianza[3])
{
    double C[3][3]; matriz(referencia->y+6,C);
    double campo_ideal[3]={0};
    for (unsigned i=0; i<3; ++i)
        for (unsigned j=0; j<3; ++j)
            campo_ideal[i]+=C[j][i]*CAMPO_NED_SIM[j];

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
    SimulacionReferencia inicial=referencia_inicial();
    float q_inicial[4];
    for (unsigned i=0; i<4; ++i) q_inicial[i]=(float)inicial.y[i+6];

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
            referencia, campo_medido, varianza);

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

/* Errores con el convenio del ESKF: real - estimado.
   Orientacion: log(conjugado(q_est) * q_ref), en el cuerpo estimado.
   Para errores pequenos, estas componentes se comparan con P[6..8]. */
static void errores(const FusionEstado *e, const SimulacionReferencia *r,
                    double error[15], double *angulo)
{
    for (unsigned i=0; i<3; ++i) {
        error[i]=r->y[i]-e->posicion_ned_m[i];
        error[i+3]=r->y[i+3]-e->velocidad_ned_m_s[i];
        error[i+9]=SESGO_ACC_REAL[i]-e->sesgo_acc_m_s2[i];
        error[i+12]=SESGO_GIRO_REAL[i]-e->sesgo_giro_rad_s[i];
    }
    double qc[4]={e->q_cuerpo_a_ned[0],-e->q_cuerpo_a_ned[1],
                  -e->q_cuerpo_a_ned[2],-e->q_cuerpo_a_ned[3]}, dq[4];
    producto(qc,r->y+6,dq);
    if (dq[0]<0) for (unsigned i=0; i<4; ++i) dq[i]=-dq[i];
    double s=hypot(hypot(dq[1],dq[2]),dq[3]);
    *angulo=2.0*atan2(s,dq[0]);
    double factor=s>1e-12 ? *angulo/s : 2.0;
    for (unsigned i=0; i<3; ++i) error[i+6]=factor*dq[i+1];
}

static const char *NOMBRES_ERROR[15]={
    "p_n_m","p_e_m","p_d_m","v_n_m_s","v_e_m_s","v_d_m_s",
    "theta_x_rad","theta_y_rad","theta_z_rad",
    "ba_x_m_s2","ba_y_m_s2","ba_z_m_s2",
    "bg_x_rad_s","bg_y_rad_s","bg_z_rad_s"};

static int cabecera(FILE *f)
{
    fputs("tiempo_s,altura_ref_m,altura_est_m,velocidad_ref_m_s,velocidad_est_m_s,"
          "error_altura_m,error_velocidad_m_s",f);
    const char *ejes[3]={"n","e","d"};
    for (unsigned i=0; i<3; ++i)
        fprintf(f,",p_%s_ref_m,p_%s_est_m,v_%s_ref_m_s,v_%s_est_m_s",
                ejes[i],ejes[i],ejes[i],ejes[i]);
    for (unsigned i=0; i<4; ++i) fprintf(f,",q%u_ref,q%u_est",i,i);
    fputs(",error_orientacion_deg",f);
    const char *cuerpo[3]={"x","y","z"};
    for (unsigned i=0; i<3; ++i)
        fprintf(f,",sesgo_acc_%s_m_s2,sesgo_acc_%s_real_m_s2,"
                  "sesgo_giro_%s_rad_s,sesgo_giro_%s_real_rad_s",
                  cuerpo[i],cuerpo[i],cuerpo[i],cuerpo[i]);
    for (unsigned i=0; i<15; ++i)
        fprintf(f,",error_%s,sigma_%s",NOMBRES_ERROR[i],NOMBRES_ERROR[i]);
    fputs(",masa_kg,empuje_n,rapidez_ref_m_s,inclinacion_ref_deg\n",f);
    return !ferror(f);
}

static int guardar(FILE *f, const FusionEstimador *est,
                   const SimulacionReferencia *r)
{
    const FusionEstado *e=&est->estado;
    double err[15],angulo,C[3][3];
    errores(e,r,err,&angulo); matriz(r->y+6,C);
    fprintf(f,"%.6f,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g",
            est->tiempo_s,-r->y[2],-(double)e->posicion_ned_m[2],
            -r->y[5],-(double)e->velocidad_ned_m_s[2],-err[2],-err[5]);
    for (unsigned i=0; i<3; ++i)
        fprintf(f,",%.12g,%.12g,%.12g,%.12g",r->y[i],
                (double)e->posicion_ned_m[i],r->y[i+3],(double)e->velocidad_ned_m_s[i]);
    for (unsigned i=0; i<4; ++i)
        fprintf(f,",%.12g,%.12g",r->y[i+6],(double)e->q_cuerpo_a_ned[i]);
    fprintf(f,",%.12g",angulo*180.0/PI_SIM);
    for (unsigned i=0; i<3; ++i)
        fprintf(f,",%.12g,%.12g,%.12g,%.12g",(double)e->sesgo_acc_m_s2[i],
                (double)SESGO_ACC_REAL[i],(double)e->sesgo_giro_rad_s[i],
                (double)SESGO_GIRO_REAL[i]);
    for (unsigned i=0; i<15; ++i)
        fprintf(f,",%.12g,%.12g",err[i],sqrt((double)e->P[i][i]));
    double t=est->tiempo_s;
    double T=(t>=DESPEGUE_S && t<DESPEGUE_S+COMBUSTION_S) ? IMPULSO_NS/COMBUSTION_S : 0;
    fprintf(f,",%.12g,%.12g,%.12g,%.12g\n",masa(t),T,
            hypot(hypot(r->y[3],r->y[4]),r->y[5]),
            acos(fmax(-1.0,fmin(1.0,-C[2][0])))*180.0/PI_SIM);
    return !ferror(f);
}

static int estado_valido(const FusionEstado *e)
{
    for (unsigned i=0; i<3; ++i)
        if (!isfinite(e->posicion_ned_m[i]) || !isfinite(e->velocidad_ned_m_s[i]) ||
            !isfinite(e->sesgo_acc_m_s2[i]) || !isfinite(e->sesgo_giro_rad_s[i])) return 0;
    double n=0;
    for (unsigned i=0; i<4; ++i) n+=(double)e->q_cuerpo_a_ned[i]*e->q_cuerpo_a_ned[i];
    if (!isfinite(n) || fabs(n-1.0)>1e-5) return 0;
    for (unsigned i=0; i<15; ++i) {
        if (e->P[i][i]<0) return 0;
        for (unsigned j=0; j<15; ++j) if (!isfinite(e->P[i][j])) return 0;
    }
    return 1;
}

int main(void)
{
    FusionEstimador est;
    FusionRuido ruido;
    srand(1U);
    if (!simulacion_inicializar_estimador(&est,&ruido)) {
        fputs("Error al inicializar el estimador.\n",stderr); return EXIT_FAILURE;
    }
    FILE *f=fopen("resultados_simulacion_3d.csv","w");
    if (!f) { perror("No se pudo abrir el CSV"); return EXIT_FAILURE; }
    SimulacionReferencia ref=referencia_inicial();
    int ok=cabecera(f) && guardar(f,&est,&ref), apogeo=0;
    double sum_p=0,sum_v=0,sum_ang=0,max_ang=0,max_v=0;
    unsigned muestras=0;
    /* Limite de seguridad de 180 s. Se termina antes del cambio de signo
       de vD; el ultimo registro es el ultimo instante IMU de ascenso. */
    for (unsigned paso=1; ok && paso<=18000U; ++paso) {
        double t=(double)paso*DT_SIM, t0=(double)(paso-1U)*DT_SIM;
        double tm=(t+t0)/2.0;
        int fase=tm<DESPEGUE_S ? 0 : (tm<DESPEGUE_S+COMBUSTION_S ? 1 : 2);
        SimulacionReferencia siguiente=ref, mitad=ref;
        double ds=DT_SIM/(double)SUBPASOS;
        for (unsigned j=0; j<SUBPASOS; ++j) {
            paso_referencia(&siguiente,t0+(double)j*ds,ds,fase);
            if (j+1U==SUBPASOS/2U) mitad=siguiente;
        }
        if (fase==2 && siguiente.y[5]>=0.0) { apogeo=1; break; }
        actualizar_auxiliares(&mitad,tm,fase);
        float acc[3],giro[3]; simulacion_generar_imu(&mitad,acc,giro);
        if (!fusion_predecir_hasta(&est,acc,giro,t,(float)(1.5*DT_SIM),G_SIM,&ruido) ||
            !simulacion_aplicar_correcciones(&est,&siguiente,paso) ||
            !estado_valido(&est.estado)) {
            fprintf(stderr,"Error en prediccion/correccion en t=%.3f s.\n",t);
            ok=0; break;
        }
        ref=siguiente;
        ok=guardar(f,&est,&ref);
        double err[15],angulo; errores(&est.estado,&ref,err,&angulo);
        for (unsigned i=0; i<3; ++i) { sum_p+=err[i]*err[i]; sum_v+=err[i+3]*err[i+3]; }
        sum_ang+=angulo*angulo; max_ang=fmax(max_ang,angulo);
        max_v=fmax(max_v,hypot(hypot(ref.y[3],ref.y[4]),ref.y[5]));
        ++muestras;
    }
    if (fclose(f)==EOF) ok=0;
    if (!ok || !apogeo || muestras==0) {
        fputs("Simulacion incompleta: error de calculo/escritura o apogeo no alcanzado.\n",stderr);
        return EXIT_FAILURE;
    }
    printf("Resultados guardados en resultados_simulacion_3d.csv\n");
    printf("Ultimo instante antes del apogeo: %.3f s\n",est.tiempo_s);
    printf("Altura: referencia = %.3f m, estimada = %.3f m\n",-ref.y[2],-(double)est.estado.posicion_ned_m[2]);
    printf("Velocidad vertical: referencia = %.3f m/s, estimada = %.3f m/s\n",-ref.y[5],-(double)est.estado.velocidad_ned_m_s[2]);
    printf("Rapidez maxima: %.3f m/s\n",max_v);
    printf("RMSE posicion 3D: %.4f m; velocidad 3D: %.4f m/s\n",sqrt(sum_p/muestras),sqrt(sum_v/muestras));
    printf("Error angular: RMS = %.4f grados; maximo = %.4f grados\n",sqrt(sum_ang/muestras)*180.0/PI_SIM,max_ang*180.0/PI_SIM);
    return EXIT_SUCCESS;
}
