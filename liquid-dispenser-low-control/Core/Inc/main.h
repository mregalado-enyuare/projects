/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LID_TRANSLATE_1_DIR_Pin GPIO_PIN_2
#define LID_TRANSLATE_1_DIR_GPIO_Port GPIOE
#define LID_TRANSLATE_2_DIR_Pin GPIO_PIN_3
#define LID_TRANSLATE_2_DIR_GPIO_Port GPIOE
#define LID_ROTATE_1_DIR_Pin GPIO_PIN_4
#define LID_ROTATE_1_DIR_GPIO_Port GPIOE
#define SYRINGE_PWM_Pin GPIO_PIN_5
#define SYRINGE_PWM_GPIO_Port GPIOE
#define LID_ROTATE_2_DIR_Pin GPIO_PIN_6
#define LID_ROTATE_2_DIR_GPIO_Port GPIOE
#define PEBBLE_ICE_TRIG_Pin GPIO_PIN_13
#define PEBBLE_ICE_TRIG_GPIO_Port GPIOC
#define GANTRY_X_PWM_Pin GPIO_PIN_0
#define GANTRY_X_PWM_GPIO_Port GPIOF
#define CUBE_ICE_DONE_Pin GPIO_PIN_1
#define CUBE_ICE_DONE_GPIO_Port GPIOF
#define CUBE_ICE_DONE_EXTI_IRQn EXTI1_IRQn
#define GANTRY_X2_PWM_Pin GPIO_PIN_2
#define GANTRY_X2_PWM_GPIO_Port GPIOF
#define LID_SEAL_DIR_Pin GPIO_PIN_3
#define LID_SEAL_DIR_GPIO_Port GPIOF
#define LID_TRANSLATE_2_FAULT_Pin GPIO_PIN_4
#define LID_TRANSLATE_2_FAULT_GPIO_Port GPIOF
#define LID_SEAL_FAULT_Pin GPIO_PIN_5
#define LID_SEAL_FAULT_GPIO_Port GPIOF
#define LID_SINGULATOR_1_RING_1_FET_Pin GPIO_PIN_6
#define LID_SINGULATOR_1_RING_1_FET_GPIO_Port GPIOF
#define GANTRY_X2_DIR_Pin GPIO_PIN_7
#define GANTRY_X2_DIR_GPIO_Port GPIOF
#define LID_SINGULATOR_1_RING_2_FET_Pin GPIO_PIN_8
#define LID_SINGULATOR_1_RING_2_FET_GPIO_Port GPIOF
#define ENABLE_24V_Pin GPIO_PIN_9
#define ENABLE_24V_GPIO_Port GPIOF
#define LID_SINGULATOR_2_RING_1_FET_Pin GPIO_PIN_10
#define LID_SINGULATOR_2_RING_1_FET_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LID_SINGULATOR_2_RING_2_FET_Pin GPIO_PIN_0
#define LID_SINGULATOR_2_RING_2_FET_GPIO_Port GPIOC
#define APDS9960_INT_Pin GPIO_PIN_2
#define APDS9960_INT_GPIO_Port GPIOC
#define APDS9960_INT_EXTI_IRQn EXTI2_IRQn
#define GANTRY_Y_PWM_Pin GPIO_PIN_0
#define GANTRY_Y_PWM_GPIO_Port GPIOA
#define CUBE_ICE_TRIG_Pin GPIO_PIN_5
#define CUBE_ICE_TRIG_GPIO_Port GPIOA
#define LID_TRANSLATE_1_PWM_Pin GPIO_PIN_6
#define LID_TRANSLATE_1_PWM_GPIO_Port GPIOA
#define LID_ROTATE_2_FAULT_Pin GPIO_PIN_1
#define LID_ROTATE_2_FAULT_GPIO_Port GPIOB
#define LID_ROTATE_1_PWM_Pin GPIO_PIN_11
#define LID_ROTATE_1_PWM_GPIO_Port GPIOF
#define LID_ROTATE_2_PWM_Pin GPIO_PIN_12
#define LID_ROTATE_2_PWM_GPIO_Port GPIOF
#define CUP_1_IN1_Pin GPIO_PIN_13
#define CUP_1_IN1_GPIO_Port GPIOF
#define CUP_1_IN2_Pin GPIO_PIN_14
#define CUP_1_IN2_GPIO_Port GPIOF
#define KEG2_FET_Pin GPIO_PIN_15
#define KEG2_FET_GPIO_Port GPIOF
#define KEG3_FET_Pin GPIO_PIN_0
#define KEG3_FET_GPIO_Port GPIOG
#define KEG4_FET_Pin GPIO_PIN_1
#define KEG4_FET_GPIO_Port GPIOG
#define GANTRY_X_LIMIT_Pin GPIO_PIN_7
#define GANTRY_X_LIMIT_GPIO_Port GPIOE
#define GANTRY_Y_LIMIT_Pin GPIO_PIN_8
#define GANTRY_Y_LIMIT_GPIO_Port GPIOE
#define SYRINGE_ENC_A_Pin GPIO_PIN_9
#define SYRINGE_ENC_A_GPIO_Port GPIOE
#define GANTRY_Y_FAULT_Pin GPIO_PIN_10
#define GANTRY_Y_FAULT_GPIO_Port GPIOE
#define SYRINGE_ENC_B_Pin GPIO_PIN_11
#define SYRINGE_ENC_B_GPIO_Port GPIOE
#define LID_TRANSLATE_1_FAULT_Pin GPIO_PIN_12
#define LID_TRANSLATE_1_FAULT_GPIO_Port GPIOE
#define KEG5_FET_Pin GPIO_PIN_13
#define KEG5_FET_GPIO_Port GPIOE
#define GANTRY_X2_LIMIT_Pin GPIO_PIN_14
#define GANTRY_X2_LIMIT_GPIO_Port GPIOE
#define CUP_2_IN1_Pin GPIO_PIN_15
#define CUP_2_IN1_GPIO_Port GPIOE
#define CUP_2_IN2_Pin GPIO_PIN_10
#define CUP_2_IN2_GPIO_Port GPIOB
#define PEBBLE_ICE_DONE_Pin GPIO_PIN_11
#define PEBBLE_ICE_DONE_GPIO_Port GPIOB
#define PEBBLE_ICE_DONE_EXTI_IRQn EXTI15_10_IRQn
#define KEG6_FET_Pin GPIO_PIN_12
#define KEG6_FET_GPIO_Port GPIOB
#define CUP_1_ECHO_Pin GPIO_PIN_14
#define CUP_1_ECHO_GPIO_Port GPIOB
#define CUP_2_ECHO_Pin GPIO_PIN_15
#define CUP_2_ECHO_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define APDS9960_2_SCL_Pin GPIO_PIN_11
#define APDS9960_2_SCL_GPIO_Port GPIOD
#define LID_TRANSLATE_2_PWM_Pin GPIO_PIN_12
#define LID_TRANSLATE_2_PWM_GPIO_Port GPIOD
#define DIP_1_Pin GPIO_PIN_13
#define DIP_1_GPIO_Port GPIOD
#define GANTRY_Y_DIR_Pin GPIO_PIN_14
#define GANTRY_Y_DIR_GPIO_Port GPIOD
#define APDS9960_2_SDA_Pin GPIO_PIN_15
#define APDS9960_2_SDA_GPIO_Port GPIOD
#define SYRINGE_SEL0_Pin GPIO_PIN_2
#define SYRINGE_SEL0_GPIO_Port GPIOG
#define SYRINGE_SEL1_Pin GPIO_PIN_3
#define SYRINGE_SEL1_GPIO_Port GPIOG
#define SYRINGE_SEL2_Pin GPIO_PIN_4
#define SYRINGE_SEL2_GPIO_Port GPIOG
#define SYRINGE_SEL3_Pin GPIO_PIN_5
#define SYRINGE_SEL3_GPIO_Port GPIOG
#define APDS9960_2_INT_Pin GPIO_PIN_6
#define APDS9960_2_INT_GPIO_Port GPIOG
#define APDS9960_2_INT_EXTI_IRQn EXTI9_5_IRQn
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_OVCR_EXTI_IRQn EXTI9_5_IRQn
#define LID_ROTATE_1_FAULT_Pin GPIO_PIN_8
#define LID_ROTATE_1_FAULT_GPIO_Port GPIOG
#define CUP_1_PWM_Pin GPIO_PIN_6
#define CUP_1_PWM_GPIO_Port GPIOC
#define CUP_2_PWM_Pin GPIO_PIN_7
#define CUP_2_PWM_GPIO_Port GPIOC
#define DIP_2_Pin GPIO_PIN_8
#define DIP_2_GPIO_Port GPIOC
#define APDS9960_SDA_Pin GPIO_PIN_9
#define APDS9960_SDA_GPIO_Port GPIOC
#define APDS9960_SCL_Pin GPIO_PIN_8
#define APDS9960_SCL_GPIO_Port GPIOA
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LID_SEAL_PWM_Pin GPIO_PIN_15
#define LID_SEAL_PWM_GPIO_Port GPIOA
#define GANTRY_X_FAULT_Pin GPIO_PIN_10
#define GANTRY_X_FAULT_GPIO_Port GPIOC
#define GANTRY_X2_FAULT_Pin GPIO_PIN_11
#define GANTRY_X2_FAULT_GPIO_Port GPIOC
#define LID_SEAL_SENSOR_Pin GPIO_PIN_12
#define LID_SEAL_SENSOR_GPIO_Port GPIOC
#define LID_SUCTION_FET_Pin GPIO_PIN_0
#define LID_SUCTION_FET_GPIO_Port GPIOD
#define CUP_1_AIR_1_FET_Pin GPIO_PIN_1
#define CUP_1_AIR_1_FET_GPIO_Port GPIOD
#define CUP_1_AIR_2_FET_Pin GPIO_PIN_2
#define CUP_1_AIR_2_FET_GPIO_Port GPIOD
#define CUP_2_AIR_1_FET_Pin GPIO_PIN_3
#define CUP_2_AIR_1_FET_GPIO_Port GPIOD
#define CUP_2_AIR_2_FET_Pin GPIO_PIN_4
#define CUP_2_AIR_2_FET_GPIO_Port GPIOD
#define LID_SINGULATOR_1_SENSOR_Pin GPIO_PIN_5
#define LID_SINGULATOR_1_SENSOR_GPIO_Port GPIOD
#define GANTRY_X_DIR_Pin GPIO_PIN_6
#define GANTRY_X_DIR_GPIO_Port GPIOD
#define LID_SINGULATOR_2_SENSOR_Pin GPIO_PIN_7
#define LID_SINGULATOR_2_SENSOR_GPIO_Port GPIOD
#define LID_TRANSLATE_1_LIMIT_Pin GPIO_PIN_9
#define LID_TRANSLATE_1_LIMIT_GPIO_Port GPIOG
#define LID_TRANSLATE_2_LIMIT_Pin GPIO_PIN_10
#define LID_TRANSLATE_2_LIMIT_GPIO_Port GPIOG
#define KEG1_FET_Pin GPIO_PIN_12
#define KEG1_FET_GPIO_Port GPIOG
#define LID_SEAL_LIMIT_Pin GPIO_PIN_14
#define LID_SEAL_LIMIT_GPIO_Port GPIOG
#define CUP_1_SWITCH_Pin GPIO_PIN_15
#define CUP_1_SWITCH_GPIO_Port GPIOG
#define CUP_1_TRIG_Pin GPIO_PIN_5
#define CUP_1_TRIG_GPIO_Port GPIOB
#define SYRINGE_IN1_Pin GPIO_PIN_6
#define SYRINGE_IN1_GPIO_Port GPIOB
#define SYRINGE_IN2_Pin GPIO_PIN_7
#define SYRINGE_IN2_GPIO_Port GPIOB
#define CUP_2_SWITCH_Pin GPIO_PIN_8
#define CUP_2_SWITCH_GPIO_Port GPIOB
#define CUP_2_TRIG_Pin GPIO_PIN_9
#define CUP_2_TRIG_GPIO_Port GPIOB
#define DIP_3_Pin GPIO_PIN_0
#define DIP_3_GPIO_Port GPIOE
#define DIP_4_Pin GPIO_PIN_1
#define DIP_4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
