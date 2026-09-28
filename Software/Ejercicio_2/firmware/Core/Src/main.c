/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile uint8_t baro_5c_detectado = 0U;
static volatile uint8_t baro_5d_detectado = 0U;
// Indican si hemos podido escribir la configuración de cada barómetro.
static volatile uint8_t baro_5c_configurado = 0U;
static volatile uint8_t baro_5d_configurado = 0U;
// Última presión leída del sensor 0x5C, en hPa.
static volatile float baro_5c_presion_hpa = 0.0f;
// Última presión leída del segundo sensor, en hPa.
static volatile float baro_5d_presion_hpa = 0.0f;
// Vale 1 solo cuando este ciclo ha producido una lectura nueva y correcta.
static volatile uint8_t baro_5d_presion_valida = 0U;
// Vale 1 solo en el ciclo en que obtenemos una lectura nueva y correcta.
static volatile uint8_t baro_5c_presion_valida = 0U;
// Última presión obtenida con las muestras nuevas disponibles, en hPa.
static volatile float presion_combinada_hpa = 0.0f;
// 0: sin muestra nueva; 1: solo 0x5C; 2: solo 0x5D; 3: ambos.
static volatile uint8_t barometros_usados = 0U;
static volatile uint8_t imu_detectada = 0U;
static volatile uint8_t imu_id_leido = 0U;
static volatile uint8_t imu_interfaz_configurada = 0U;
static volatile uint8_t imu_lowg_configurado = 0U;
static volatile float imu_ax_g = 0.0f;
static volatile float imu_ay_g = 0.0f;
static volatile float imu_az_g = 0.0f;
static volatile uint8_t imu_lowg_muestra_valida = 0U;
static volatile uint8_t imu_highg_configurado = 0U;
static volatile uint8_t imu_giro_configurado = 0U;
static volatile float imu_gx_dps = 0.0f;
static volatile float imu_gy_dps = 0.0f;
static volatile float imu_gz_dps = 0.0f;
static volatile uint8_t imu_giro_muestra_valida = 0U;
static uint8_t gnss_byte_rx = 0U;
static volatile uint8_t gnss_buffer[256] = {0U};
static volatile uint16_t gnss_indice = 0U;
static volatile uint32_t gnss_bytes_recibidos = 0U;
static volatile uint8_t gnss_rx_activa = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static float FiltrarAltura(float altura_nueva, float altura_anterior);
static float AlturaDesdePresion(float presion_pa, float presion_inicial_pa);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Aquí se guardará el identificador leído de cada sensor.
  // WHO_AM_I: 0x0F. El bit 7 indica lectura SPI.
  gnss_rx_activa =
    (HAL_UART_Receive_IT(&huart1, &gnss_byte_rx, 1U) == HAL_OK);
  uint8_t imu_tx[2] = {0x8FU, 0x00U};
  uint8_t imu_rx[2] = {0U, 0U};

  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);

  HAL_StatusTypeDef resultado_imu = HAL_SPI_TransmitReceive(
    &hspi1, imu_tx, imu_rx, 2U, 100U
  );

  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
  // El segundo contiene el valor del registro.
  if (resultado_imu == HAL_OK)
  {
    imu_id_leido = imu_rx[1];
    imu_detectada = (imu_id_leido == 0x73U);
  }
  if (imu_detectada)
{
    // CTRL3 (0x12): BDU = 1 e IF_INC = 1.
    uint8_t imu_config_tx[2] = {0x12U, 0x44U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef escritura_imu = HAL_SPI_Transmit(
        &hspi1, imu_config_tx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (escritura_imu == HAL_OK)
    {
        // Leemos CTRL3 de vuelta: 0x12 | 0x80 = 0x92.
        uint8_t imu_verif_tx[2] = {0x92U, 0x00U};
        uint8_t imu_verif_rx[2] = {0U, 0U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef lectura_imu = HAL_SPI_TransmitReceive(
            &hspi1, imu_verif_tx, imu_verif_rx, 2U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        imu_interfaz_configurada =
            (lectura_imu == HAL_OK &&
             (imu_verif_rx[1] & 0x44U) == 0x44U);
    }
}
if (imu_interfaz_configurada)
{
    // CTRL8: rango del acelerómetro low-g de +/-16 g.
    uint8_t config_rango[2] = {0x17U, 0x03U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef escritura_rango = HAL_SPI_Transmit(
        &hspi1, config_rango, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (escritura_rango == HAL_OK)
    {
        // CTRL1: modo de alto rendimiento y ODR de 60 Hz.
        uint8_t config_odr[2] = {0x10U, 0x05U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef escritura_odr = HAL_SPI_Transmit(
            &hspi1, config_odr, 2U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        imu_lowg_configurado = (escritura_odr == HAL_OK);
    }
}
if (imu_lowg_configurado)
{
    // Lectura de CTRL8 (0x17): 0x17 | 0x80 = 0x97.
    uint8_t tx_rango[2] = {0x97U, 0x00U};
    uint8_t rx_rango[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_rango = HAL_SPI_TransmitReceive(
        &hspi1, tx_rango, rx_rango, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    // Lectura de CTRL1 (0x10): 0x10 | 0x80 = 0x90.
    uint8_t tx_odr[2] = {0x90U, 0x00U};
    uint8_t rx_odr[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_odr = HAL_SPI_TransmitReceive(
        &hspi1, tx_odr, rx_odr, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    imu_lowg_configurado =
        (lectura_rango == HAL_OK && rx_rango[1] == 0x03U &&
         lectura_odr == HAL_OK && rx_odr[1] == 0x05U);
}
if (imu_lowg_configurado)
{
    /* CTRL1_XL_HG (0x4E):
       bit 7 = 1: habilita los registros de salida high-g
       bits 5:3 = 011: 480 Hz
       bits 2:0 = 100: rango ±320 g
       Resultado: 0x9C. */
    uint8_t highg_config_tx[2] = {0x4EU, 0x9CU};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef escritura_highg = HAL_SPI_Transmit(
        &hspi1, highg_config_tx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (escritura_highg == HAL_OK)
    {
        /* Lectura de comprobación: 0x4E | 0x80 = 0xCE. */
        uint8_t highg_verif_tx[2] = {0xCEU, 0x00U};
        uint8_t highg_verif_rx[2] = {0U, 0U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef lectura_highg = HAL_SPI_TransmitReceive(
            &hspi1, highg_verif_tx, highg_verif_rx, 2U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        imu_highg_configurado =
            (lectura_highg == HAL_OK &&
             highg_verif_rx[1] == 0x9CU);
    }
}
if (imu_lowg_configurado)
{
    /* CTRL6 (0x15): rango del giróscopo ±2000 °/s. */
    uint8_t giro_rango_tx[2] = {0x15U, 0x04U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef escritura_rango_giro = HAL_SPI_Transmit(
        &hspi1, giro_rango_tx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (escritura_rango_giro == HAL_OK)
    {
        /* CTRL2 (0x11): alto rendimiento y 60 Hz. */
        uint8_t giro_odr_tx[2] = {0x11U, 0x05U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef escritura_odr_giro = HAL_SPI_Transmit(
            &hspi1, giro_odr_tx, 2U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        if (escritura_odr_giro == HAL_OK)
{
    /* CTRL6: comprobamos el rango ±2000 °/s. */
    uint8_t rango_verif_tx[2] = {0x95U, 0x00U};
    uint8_t rango_verif_rx[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_rango_giro = HAL_SPI_TransmitReceive(
        &hspi1, rango_verif_tx, rango_verif_rx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    /* CTRL2: comprobamos la frecuencia de 60 Hz. */
    uint8_t odr_verif_tx[2] = {0x91U, 0x00U};
    uint8_t odr_verif_rx[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_odr_giro = HAL_SPI_TransmitReceive(
        &hspi1, odr_verif_tx, odr_verif_rx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    imu_giro_configurado =
        (lectura_rango_giro == HAL_OK &&
         rango_verif_rx[1] == 0x04U &&
         lectura_odr_giro == HAL_OK &&
         odr_verif_rx[1] == 0x05U);
}
    }
}

  uint8_t id_5c = 0U;
  uint8_t id_5d = 0U;
// Leemos el registro WHO_AM_I (0x0F) del barómetro con dirección 0x5C.
// HAL espera la dirección de 7 bits desplazada una posición a la izquierda.
HAL_StatusTypeDef lectura_5c = HAL_I2C_Mem_Read(
    &hi2c1,                 // Bus I²C configurado en CubeMX.
    0x5CU << 1,             // Dirección del primer barómetro.
    0x0FU,                  // Registro WHO_AM_I.
    I2C_MEMADD_SIZE_8BIT,   // La dirección del registro ocupa 8 bits.
    &id_5c,                // Variable donde se guardará el dato recibido.
    1U,                    // Leemos un byte.
    100U                   // Tiempo máximo de espera, en milisegundos.
);
// Repetimos la lectura para el barómetro con dirección 0x5D.
HAL_StatusTypeDef lectura_5d = HAL_I2C_Mem_Read(
    &hi2c1,
    0x5DU << 1,
    0x0FU,
    I2C_MEMADD_SIZE_8BIT,
    &id_5d,
    1U,
    100U
);
// 0xB4 es el valor esperado de WHO_AM_I para el LPS22DF.
// Exigimos tanto una comunicación correcta como el identificador esperado.
baro_5c_detectado = (lectura_5c == HAL_OK && id_5c == 0xB4U);
baro_5d_detectado = (lectura_5d == HAL_OK && id_5d == 0xB4U);
// CTRL_REG1: ODR = 100 Hz (0111) y AVG = 32 (011).
uint8_t configuracion = 0x3BU;
// Solo intentamos configurar el sensor que se haya identificado.
if (baro_5c_detectado)
{
    baro_5c_configurado = (HAL_I2C_Mem_Write(
        &hi2c1,                 // Bus I²C.
        0x5CU << 1,             // Dirección del primer barómetro.
        0x10U,                  // Registro CTRL_REG1.
        I2C_MEMADD_SIZE_8BIT,   // Dirección de registro de 8 bits.
        &configuracion,         // Valor que queremos escribir.
        1U,                    // Escribimos un byte.
        100U                   // Tiempo máximo de espera, en ms.
    ) == HAL_OK);
}
if (baro_5d_detectado)
{
    baro_5d_configurado = (HAL_I2C_Mem_Write(
        &hi2c1,
        0x5DU << 1,             // Dirección del segundo barómetro.
        0x10U,
        I2C_MEMADD_SIZE_8BIT,
        &configuracion,         // Ambos reciben la misma configuración.
        1U,
        100U
    ) == HAL_OK);
}// Variables donde guardaremos el contenido leído de CTRL_REG1.
uint8_t ctrl_5c = 0U;
uint8_t ctrl_5d = 0U;
if (baro_5c_configurado)
{
    // Leemos de nuevo CTRL_REG1 en el primer barómetro.
    HAL_StatusTypeDef resultado_5c = HAL_I2C_Mem_Read(
        &hi2c1,
        0x5CU << 1,
        0x10U,                  // Registro que acabamos de escribir.
        I2C_MEMADD_SIZE_8BIT,
        &ctrl_5c,               // Aquí quedará el valor leído.
        1U,
        100U
    );
    // Solo lo damos por configurado si la lectura funciona
    // y el registro contiene el valor previsto.
    baro_5c_configurado =
        (resultado_5c == HAL_OK && ctrl_5c == configuracion);
}
if (baro_5d_configurado)
{
    // Hacemos la misma comprobación en el segundo barómetro.
    HAL_StatusTypeDef resultado_5d = HAL_I2C_Mem_Read(
        &hi2c1,
        0x5DU << 1,
        0x10U,
        I2C_MEMADD_SIZE_8BIT,
        &ctrl_5d,
        1U,
        100U
    );
    baro_5d_configurado =
        (resultado_5d == HAL_OK && ctrl_5d == configuracion);
}
// En CTRL_REG2, el bit 3 activa BDU: 0000 1000 = 0x08.
uint8_t configuracion_bdu = 0x08U;
if (baro_5c_configurado)
{
    // Activamos BDU en el primer barómetro.
    HAL_StatusTypeDef escritura = HAL_I2C_Mem_Write(
        &hi2c1, 0x5CU << 1, 0x11U, I2C_MEMADD_SIZE_8BIT,
        &configuracion_bdu, 1U, 100U
    );
    // Si falla esta escritura, el sensor ya no se considera
    // completamente configurado.
    baro_5c_configurado = (escritura == HAL_OK);
}
if (baro_5d_configurado)
{
    // Aplicamos la misma configuración al segundo barómetro.
    HAL_StatusTypeDef escritura = HAL_I2C_Mem_Write(
        &hi2c1, 0x5DU << 1, 0x11U, I2C_MEMADD_SIZE_8BIT,
        &configuracion_bdu, 1U, 100U
    );
    baro_5d_configurado = (escritura == HAL_OK);
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t instante_anterior = 0U;
    static uint8_t primera_medida = 1U;
    static uint8_t referencia_pendiente = 1U;
    static float presion_inicio_pa = 0.0f;
    static float altura_filtrada = 0.0f;
    static volatile float resultado_prueba = 0.0f;
    static float historial_alturas[20] = {0};
    static uint8_t indice_historial = 0U;
    static uint8_t muestras_guardadas = 0U;
    static volatile float cambio_altura_02s = 0.0f;
    static float altura_maxima = 0.0f;
    static uint8_t confirmaciones_descenso = 0U;
    static uint8_t fallos_consecutivos = 0U;
    static volatile uint8_t descenso_detectado = 0U;
    uint32_t instante_actual = HAL_GetTick();
    if ((uint32_t)(instante_actual - instante_anterior) >= 10U)
    {
      instante_anterior = instante_actual;
      /* Se reinicia en cada ciclo: los valores numéricos conservan
   la última lectura, aunque no haya una muestra nueva. */
imu_lowg_muestra_valida = 0U;

if (imu_lowg_configurado)
{
    /* STATUS_REG (0x1E), bit 0: XLDA indica datos nuevos
       del acelerómetro low-g. */
    uint8_t estado_tx[2] = {0x9EU, 0x00U};
    uint8_t estado_rx[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_estado = HAL_SPI_TransmitReceive(
        &hspi1, estado_tx, estado_rx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (lectura_estado == HAL_OK && (estado_rx[1] & 0x01U) != 0U)
    {
        /* Lectura continua de 0x28 a 0x2D:
           X bajo/alto, Y bajo/alto, Z bajo/alto. */
        uint8_t datos_tx[7] = {0xA8U, 0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t datos_rx[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef lectura_datos = HAL_SPI_TransmitReceive(
            &hspi1, datos_tx, datos_rx, 7U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        if (lectura_datos == HAL_OK)
        {
            int16_t ax_bruta = (int16_t)(
                (uint16_t)datos_rx[1] |
                ((uint16_t)datos_rx[2] << 8)
            );
            int16_t ay_bruta = (int16_t)(
                (uint16_t)datos_rx[3] |
                ((uint16_t)datos_rx[4] << 8)
            );
            int16_t az_bruta = (int16_t)(
                (uint16_t)datos_rx[5] |
                ((uint16_t)datos_rx[6] << 8)
            );

            /* Para el rango configurado de ±16 g:
               0,488 mg por unidad = 0,000488 g por unidad. */
            imu_ax_g = (float)ax_bruta * 0.000488f;
            imu_ay_g = (float)ay_bruta * 0.000488f;
            imu_az_g = (float)az_bruta * 0.000488f;
            imu_lowg_muestra_valida = 1U;
        }
    }
}
imu_giro_muestra_valida = 0U;

if (imu_giro_configurado)
{
    /* STATUS_REG (0x1E), bit 1: datos nuevos del giróscopo. */
    uint8_t giro_estado_tx[2] = {0x9EU, 0x00U};
    uint8_t giro_estado_rx[2] = {0U, 0U};

    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef lectura_estado_giro = HAL_SPI_TransmitReceive(
        &hspi1, giro_estado_tx, giro_estado_rx, 2U, 100U
    );
    HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

    if (lectura_estado_giro == HAL_OK &&
        (giro_estado_rx[1] & 0x02U) != 0U)
    {
        /* Registros 0x22 a 0x27: X, Y y Z, byte bajo y alto. */
        uint8_t giro_datos_tx[7] = {0xA2U, 0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t giro_datos_rx[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};

        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef lectura_datos_giro = HAL_SPI_TransmitReceive(
            &hspi1, giro_datos_tx, giro_datos_rx, 7U, 100U
        );
        HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

        if (lectura_datos_giro == HAL_OK)
        {
            int16_t gx_bruta = (int16_t)(
                (uint16_t)giro_datos_rx[1] |
                ((uint16_t)giro_datos_rx[2] << 8)
            );
            int16_t gy_bruta = (int16_t)(
                (uint16_t)giro_datos_rx[3] |
                ((uint16_t)giro_datos_rx[4] << 8)
            );
            int16_t gz_bruta = (int16_t)(
                (uint16_t)giro_datos_rx[5] |
                ((uint16_t)giro_datos_rx[6] << 8)
            );

            /* ±2000 °/s: 70 mdps/LSB = 0,070 °/s por unidad. */
            imu_gx_dps = (float)gx_bruta * 0.070f;
            imu_gy_dps = (float)gy_bruta * 0.070f;
            imu_gz_dps = (float)gz_bruta * 0.070f;
            imu_giro_muestra_valida = 1U;
        }
    }
}
      uint8_t nueva_presion_5c = 0U;
      if (baro_5c_configurado)
      {
        uint8_t estado_5c = 0U;
        HAL_StatusTypeDef resultado = HAL_I2C_Mem_Read(
            &hi2c1,
            0x5CU << 1,
            0x27U,                  // Registro STATUS.
            I2C_MEMADD_SIZE_8BIT,
            &estado_5c,
            1U,
            100U
        );
        // P_DA es el bit 0: indica presión nueva disponible.
        nueva_presion_5c =
            (resultado == HAL_OK && (estado_5c & 0x01U) != 0U);
      }
            // Reiniciamos la validez en cada ciclo.
      // Una presión guardada de antes no se convierte en muestra nueva.
      baro_5c_presion_valida = 0U;
      if (nueva_presion_5c)
      {
        uint8_t bytes_presion[3] = {0U, 0U, 0U};
        // Leemos consecutivamente 0x28, 0x29 y 0x2A.
        // El incremento automático de dirección viene activado por defecto.
        HAL_StatusTypeDef lectura_presion = HAL_I2C_Mem_Read(
            &hi2c1,
            0x5CU << 1,
            0x28U,                  // Primer registro: PRESS_OUT_XL.
            I2C_MEMADD_SIZE_8BIT,
            bytes_presion,          // Recibirá los tres bytes.
            3U,
            100U
        );
        if (lectura_presion == HAL_OK)
        {
          // Reconstruimos el número de 24 bits:
          // byte bajo + byte central + byte alto.
          uint32_t bruto_24 =
              (uint32_t)bytes_presion[0]
              | ((uint32_t)bytes_presion[1] << 8)
              | ((uint32_t)bytes_presion[2] << 16);
          // La ficha define el dato como un entero con signo de 24 bits.
          // Si su bit de signo está activo, extendemos ese signo a int32_t.
          int32_t bruto_con_signo = (int32_t)bruto_24;
          if ((bruto_24 & 0x800000U) != 0U)
          {
            bruto_con_signo -= 0x1000000;
          }
          // Sensibilidad del LPS22DF: 4096 unidades por hPa.
          baro_5c_presion_hpa = (float)bruto_con_signo / 4096.0f;
          baro_5c_presion_valida = 1U;
        }
      }
          // Igual que con 0x5C, primero preguntamos si hay una muestra nueva.
      uint8_t nueva_presion_5d = 0U;
      baro_5d_presion_valida = 0U;
      if (baro_5d_configurado)
      {
        uint8_t estado_5d = 0U;
        HAL_StatusTypeDef resultado = HAL_I2C_Mem_Read(
            &hi2c1,
            0x5DU << 1,             // Dirección del segundo barómetro.
            0x27U,                  // Registro STATUS.
            I2C_MEMADD_SIZE_8BIT,
            &estado_5d,
            1U,
            100U
        );
        // P_DA, bit 0: hay una presión nueva disponible.
        nueva_presion_5d =
            (resultado == HAL_OK && (estado_5d & 0x01U) != 0U);
      }
      if (nueva_presion_5d)
      {
        uint8_t bytes_presion[3] = {0U, 0U, 0U};
        // Leemos los tres bytes en orden: XL, L y H.
        HAL_StatusTypeDef lectura_presion = HAL_I2C_Mem_Read(
            &hi2c1,
            0x5DU << 1,
            0x28U,                  // Primer registro de presión.
            I2C_MEMADD_SIZE_8BIT,
            bytes_presion,
            3U,
            100U
        );
        if (lectura_presion == HAL_OK)
        {
          // Unimos los tres bytes en un entero de 24 bits.
          uint32_t bruto_24 =
              (uint32_t)bytes_presion[0]
              | ((uint32_t)bytes_presion[1] << 8)
              | ((uint32_t)bytes_presion[2] << 16);
          // Extendemos el signo si el bit 23 está activo.
          int32_t bruto_con_signo = (int32_t)bruto_24;
          if ((bruto_24 & 0x800000U) != 0U)
          {
            bruto_con_signo -= 0x1000000;
          }
          // Convertimos el dato del sensor a hPa.
          baro_5d_presion_hpa = (float)bruto_con_signo / 4096.0f;
          baro_5d_presion_valida = 1U;
        }
      }
      // Se decide de nuevo en cada ciclo qué muestras se han recibido.
      barometros_usados = 0U;
      if (baro_5c_presion_valida && baro_5d_presion_valida)
      {
        // Si ambos entregaron una muestra nueva, calculamos su media.
        presion_combinada_hpa =
            0.5f * (baro_5c_presion_hpa + baro_5d_presion_hpa);
        barometros_usados = 3U;
      }
      else if (baro_5c_presion_valida)
      {
        // Solo hay una muestra nueva del sensor 0x5C.
        presion_combinada_hpa = baro_5c_presion_hpa;
        barometros_usados = 1U;
      }
      else if (baro_5d_presion_valida)
      {
        // Solo hay una muestra nueva del sensor 0x5D.
        presion_combinada_hpa = baro_5d_presion_hpa;
        barometros_usados = 2U;
      }
      // Una muestra nueva permite actualizar la altura. En el primer ciclo
      // válido, la presión medida fija el cero de altura relativo al arranque.
      if (barometros_usados != 0U && presion_combinada_hpa > 0.0f)
      {
        float presion_actual_pa = presion_combinada_hpa * 100.0f;
        if (referencia_pendiente)
        {
          presion_inicio_pa = presion_actual_pa;
          referencia_pendiente = 0U;
        }
        float altura_nueva =
            AlturaDesdePresion(presion_actual_pa, presion_inicio_pa);
        if (primera_medida)
        {
          altura_filtrada = altura_nueva;
          primera_medida = 0U;
        }
        else
        {
          altura_filtrada = FiltrarAltura(altura_nueva, altura_filtrada);
        }
        if (altura_filtrada > altura_maxima)
        {
          altura_maxima = altura_filtrada;
        }
        if (muestras_guardadas == 20U)
        {
          cambio_altura_02s =
              altura_filtrada - historial_alturas[indice_historial];
          if (descenso_detectado == 0U)
          {
            const float caida_minima_desde_maximo_m = 2.0f;
            const float descenso_minimo_02s_m = 1.0f;
            if ((altura_maxima - altura_filtrada >= caida_minima_desde_maximo_m) &&
                (cambio_altura_02s <= -descenso_minimo_02s_m))
            {
              fallos_consecutivos = 0U;
              confirmaciones_descenso++;
              if (confirmaciones_descenso >= 50U)
              {
                descenso_detectado = 1U;
              }
            }
            else
            {
              fallos_consecutivos++;
              if (fallos_consecutivos >= 3U)
              {
                confirmaciones_descenso = 0U;
                fallos_consecutivos = 0U;
              }
            }
          }
        }
        else
        {
          muestras_guardadas++;
        }
        historial_alturas[indice_historial] = altura_filtrada;
        indice_historial = (uint8_t)((indice_historial + 1U) % 20U);
        resultado_prueba = altura_filtrada;
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_MEM_GPIO_Port, CS_MEM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_IMU_Pin */
  GPIO_InitStruct.Pin = CS_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_IMU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_MEM_Pin */
  GPIO_InitStruct.Pin = CS_MEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_MEM_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        gnss_buffer[gnss_indice] = gnss_byte_rx;
        gnss_indice = (uint16_t)((gnss_indice + 1U) & 0xFFU);
        gnss_bytes_recibidos++;

        /* Preparar la recepción del siguiente byte. */
        gnss_rx_activa =
            (HAL_UART_Receive_IT(&huart1, &gnss_byte_rx, 1U) == HAL_OK);
    }
}
static float FiltrarAltura(float altura_nueva, float altura_anterior)
{
  const float alpha = 0.25f;  /* Valor de prueba, todavía no validado */
  return altura_anterior + alpha * (altura_nueva - altura_anterior);
}
static float AlturaDesdePresion(float presion_pa, float presion_inicial_pa)
{
  const float R = 287.05f;     /* J/(kg·K), aire seco */
  const float T = 288.15f;     /* K, valor supuesto para esta prueba */
  const float g = 9.80665f;    /* m/s² */
  return (R * T / g) * logf(presion_inicial_pa / presion_pa);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
