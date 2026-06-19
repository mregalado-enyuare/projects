#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "commands.h"   // cmd_status_t
#include "stm32h7xx_hal.h"
#include "async_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

cmd_status_t HomingX_MoveIndependentSteps_Async(bool motor_left,
                                                int32_t steps_signed,
                                                int speed_k,
                                                int accel_k,
                                                uint8_t async_slot);

cmd_status_t HomingX_MoveIndependentUntilLimit_Async(bool motor_left,
                                                     int speed_k,
                                                     uint8_t async_slot);

cmd_status_t HomingX_HomeDualUntilLimit_Async(int speed_k,
                                              uint8_t async_slot);

cmd_status_t HomingX_MoveDualUntilPin_Async(GPIO_TypeDef *port,
                                           uint16_t pin,
                                           bool active_low,
                                           int speed_k,
                                           uint8_t async_slot);

void HomingX_Pump(void);

void HomingX_Abort(void);
bool HomingX_IsActive(void);
async_result_t HomingX_LastResult(void);


#ifdef __cplusplus
}
#endif
