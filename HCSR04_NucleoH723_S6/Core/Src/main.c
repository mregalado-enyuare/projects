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
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "delay_us.h"
#include "hcsr04.h"
#include "cup_detector.h"
#include <stdio.h>
#include <string.h>

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
char uartMsg[256];  // buffer available for formatted UART messages if needed
/* USER CODE END PV */

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    /* set up memory protection before anything else runs */
    MPU_Config();

    /* reset all peripherals and start the HAL tick timer (1ms) */
    HAL_Init();

    /* configure system clock using HSI + PLL */
    SystemClock_Config();

    /* set up TRIG pins as outputs and ECHO pins as inputs */
    MX_GPIO_Init();

    /* enable the DWT cycle counter used for microsecond delays */
    DWT_Delay_Init();

    /* assign GPIO pins to each sensor and zero out their distances */
    HCSR04_Init();

    /* LED_GREEN is commented out because PB0 is used for sensor 0 echo pin
       initializing it would reconfigure PB0 as output and break sensor 0 */
    // BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);

    /* set up the blue user button as an external interrupt */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* set up UART over ST-Link VCP (COM1 = USART3, shows as COM21 on PC)
       used to print sensor readings to PuTTY/plink at 115200 baud */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();  // hang if UART fails to init
    }

    while (1)
    {
        /* trigger all 5 sensors one at a time and store averaged distances */
        HCSR04_ScanAll();

        /* classify each sensor reading into EMPTY, CUP, TRASH, etc. */
        CupDetector_Update();

        /* print each sensor's raw distance and current slot status */
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            const char* statusStr;
            switch(slotStatus[i])
            {
                case SLOT_CUP_PRESENT: statusStr = "CUP";         break;
                case SLOT_TRASH:       statusStr = "TRASH";       break;
                case SLOT_EMPTY:       statusStr = "EMPTY";       break;
                case SLOT_OBSTRUCTION: statusStr = "OBSTRUCTION"; break;
                default:               statusStr = "ERROR";       break;
            }
            /* print integer and decimal parts separately to avoid %f
               since float printing is disabled by default on this toolchain */
            printf("Sensor %d: %d.%d cm | Status: %s\r\n",
                   i,
                   (int)sensors[i].distance_cm,
                   (int)(sensors[i].distance_cm * 10) % 10,
                   statusStr);
        }

        /* print total cup count using neighbor deduplication (max 3) */
        printf("Cups detected: %d\r\n", GetCupCount());

        /* print overall system state based on all active sensors */
        SystemStatus_t consensus = GetSystemConsensus();
        switch(consensus)
        {
            case SYSTEM_CUP_DETECTED:   printf("CONSENSUS: CUP IN SYSTEM\r\n");   break;
            case SYSTEM_TRASH_DETECTED: printf("CONSENSUS: TRASH IN SYSTEM\r\n"); break;
            case SYSTEM_ALL_EMPTY:      printf("CONSENSUS: ALL SLOTS EMPTY\r\n"); break;
            case SYSTEM_MIXED:          printf("CONSENSUS: MIXED OBJECTS\r\n");   break;
        }
        printf("---\r\n");

        /* wait before next scan — controls how often the output updates */
        HAL_Delay(50);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* use internal HSI oscillator as the PLL input source */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 12;
    RCC_OscInitStruct.PLL.PLLP            = 1;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    /* configure CPU, AHB, and APB bus clock dividers */
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
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* enable clocks for all GPIO ports being used */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* make sure all TRIG pins start LOW before configuring them */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                           | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    /* configure PA0-PA5 as push-pull outputs for sensor TRIG signals */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                          | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* configure PB0, PB1, PB2, PB10, PB11, PB12 as inputs for sensor ECHO signals */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                         | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA0 and PA1 have internal analog switches that connect to the ADC
       setting them to OPEN disconnects the ADC path so the pins work
       cleanly as digital outputs for the TRIG signal */
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_OPEN);
}

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    /* block access to the entire 4GB address space by default
       this catches null pointer dereferences and bad memory accesses */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
    /* disable all interrupts and loop forever
       used to catch any hardware init failures at startup */
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* called automatically if an HAL assert fails
       can add a printf here to print the file and line number */
}
#endif
