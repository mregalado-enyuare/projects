/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HC-SR04 Ultrasonic Distance Sensor — Roll Level Detector
  *                   STM32H723ZG NUCLEO-H723ZG
  *
  *  TRIG  →  PA10  (GPIO_Output)
  *  ECHO  →  PA4   (GPIO_Input)
  *  Timer →  TIM1  (prescaler=95, ARR=65535 → 1 µs tick @ 96 MHz)
  *  UART  →  USART3 115200 baud (VCP via USB on NUCLEO)
  *
  *  Roll diameter reference (sensor distance from surface when full = 3.0 cm):
  *    100%  →  6.350 cm diam  →  sensor reads ~3.00 cm
  *     70%  →  4.445 cm diam  →  sensor reads ~3.95 cm
  *     50%  →  (interpolated) →  sensor reads ~4.60 cm
  *     30%  →  1.905 cm diam  →  sensor reads ~5.22 cm
  *    DONE  →  roll gone      →  sensor reads  >5.50 cm
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ── Roll level thresholds (sensor distance in cm) ──────────────────────────
 * Adjust these if your sensor mounting distance changes.
 * All thresholds are the distance the sensor reads at that roll state.       */
#define THRESH_FULL      3.50f   /* below this  → FULL  (100%)              */
#define THRESH_HIGH      4.20f   /* below this  → HIGH  (~70%)              */
#define THRESH_HALF      4.90f   /* below this  → HALFWAY (~50%)            */
#define THRESH_LOW       5.40f   /* below this  → LOW   (~30%)              */
                                 /* at or above → DONE / REPLACE ROLL       */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
float       HCSR04_Read(void);
void        UART_Print(const char *msg);
const char *GetRollStatus(float distance_cm);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * @brief  Send a string directly over USART3 (no printf dependency).
 */
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief  Triggers HC-SR04, returns distance in cm.
 *         Returns -1.0f on timeout (no echo / out of range).
 */
float HCSR04_Read(void)
{
    /* Trigger: 10 µs HIGH pulse */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    /* Wait for ECHO HIGH */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        if (__HAL_TIM_GET_COUNTER(&htim1) > 30000) return -1.0f;
    }

    /* Measure ECHO pulse width */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        if (__HAL_TIM_GET_COUNTER(&htim1) > 30000) return -1.0f;
    }

    uint32_t pulse_us = __HAL_TIM_GET_COUNTER(&htim1);
    return (float)pulse_us * 0.01715f;   /* 0.0343 cm/µs ÷ 2 */
}

/**
 * @brief  Maps a sensor distance reading to a roll status string.
 *
 *  Distance increases as roll depletes (sensor→surface gap grows).
 *
 *  < THRESH_FULL  →  "FULL"
 *  < THRESH_HIGH  →  "HIGH  (~70%)"
 *  < THRESH_HALF  →  "HALFWAY (~50%)"
 *  < THRESH_LOW   →  "LOW   (~30%) - order soon"
 *  >= THRESH_LOW  →  "DONE  - replace roll!"
 */
const char *GetRollStatus(float distance_cm)
{
    if (distance_cm < THRESH_FULL) return "FULL";
    if (distance_cm < THRESH_HIGH) return "HIGH  (~70%)";
    if (distance_cm < THRESH_HALF) return "HALFWAY (~50%)";
    if (distance_cm < THRESH_LOW)  return "LOW   (~30%) - order soon";
    return                                "DONE  - replace roll!";
}

/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */
    setvbuf(stdout, NULL, _IONBF, 0);
    HAL_TIM_Base_Start(&htim1);

    HAL_UART_Transmit(&huart3, (uint8_t *)"BOOT\r\n", 6, 1000);
    UART_Print("\r\n================================\r\n");
    UART_Print("  Roll Level Detector\r\n");
    UART_Print("  NUCLEO-H723ZG / HC-SR04\r\n");
    UART_Print("================================\r\n\r\n");
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    char buf[80];

    while (1)
    {
        float dist = HCSR04_Read();

        if (dist < 0.0f)
        {
            UART_Print("Sensor: NO READING (check wiring)\r\n");
        }
        else if (dist < 2.0f)
        {
            UART_Print("Sensor: TOO CLOSE\r\n");
        }
        else
        {
            const char *status = GetRollStatus(dist);

            /* Print distance as integer + fractional parts (nano.specs safe) */
            int d_int  = (int)dist;
            int d_frac = (int)((dist - d_int) * 100);

            int len = snprintf(buf, sizeof(buf),
                               "Distance: %d.%02d cm  |  Roll: %s\r\n",
                               d_int, d_frac, status);
            HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, HAL_MAX_DELAY);
        }

        HAL_Delay(500);   /* read every 500 ms */
    }
    /* USER CODE END WHILE */
}

/**
  * @brief  System Clock — SYSCLK 192 MHz, AHB 96 MHz, APBx 48 MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 12;
    RCC_OscInitStruct.PLL.PLLP            = 1;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE         = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL      = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN       = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                     | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  TIM1 — 1 µs tick (prescaler=95, ARR=65535)
  */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 95;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 65535;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  USART3 — 115200 8N1
  */
static void MX_USART3_UART_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  GPIO — PA10 TRIG out, PA4 ECHO in, PD8/9 USART3 AF
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* PA10 — TRIG, start LOW */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA4 — ECHO, no pull */
    GPIO_InitStruct.Pin  = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PD8/PD9 — USART3 TX/RX */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
