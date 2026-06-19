#ifndef VL53L4CD_H
#define VL53L4CD_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define VL53L4CD_ADDR  0x52U   /* 8-bit I2C address (7-bit 0x29 << 1) */

typedef uint8_t VL53L4CD_Error;
#define VL53L4CD_OK          ((VL53L4CD_Error)0U)
#define VL53L4CD_ERR_TIMEOUT ((VL53L4CD_Error)255U)
#define VL53L4CD_ERR_INVALID ((VL53L4CD_Error)254U)

typedef struct {
    uint8_t  range_status;          /* 0 = valid measurement */
    uint16_t distance_mm;
    uint16_t ambient_rate_kcps;
    uint16_t ambient_per_spad_kcps;
    uint16_t signal_rate_kcps;
    uint16_t signal_per_spad_kcps;
    uint16_t number_of_spad;
    uint16_t sigma_mm;
} VL53L4CD_Result_t;

VL53L4CD_Error VL53L4CD_Init(I2C_HandleTypeDef *hi2c);
VL53L4CD_Error VL53L4CD_SetRangeTiming(I2C_HandleTypeDef *hi2c, uint32_t timing_budget_ms, uint32_t inter_measurement_ms);
VL53L4CD_Error VL53L4CD_StartRanging(I2C_HandleTypeDef *hi2c);
VL53L4CD_Error VL53L4CD_StopRanging(I2C_HandleTypeDef *hi2c);
VL53L4CD_Error VL53L4CD_CheckForDataReady(I2C_HandleTypeDef *hi2c, uint8_t *is_ready);
VL53L4CD_Error VL53L4CD_GetResult(I2C_HandleTypeDef *hi2c, VL53L4CD_Result_t *result);
VL53L4CD_Error VL53L4CD_ClearInterrupt(I2C_HandleTypeDef *hi2c);

#endif /* VL53L4CD_H */
