/*
 * motor_until.h
 *
 *  Created on: Nov 11, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_MOTOR_UNTIL_H_
#define INC_MOTOR_UNTIL_H_

#include "stm32h7xx_hal.h"
#include "dc_motor.h"
#include "async_bus.h"
#include <stdbool.h>
#include <stdint.h>

// Start a non-blocking "run motor until sensor active (debounced)".
// steps_mag: large positive magnitude to run; forward=true => +steps, false => -steps
bool motor_until_sensor_start(dc_motor_t *motor,
                              bool forward,
                              uint32_t steps_mag,
                              GPIO_TypeDef *sensor_port, uint16_t sensor_pin, bool sensor_active_low,
                              uint16_t debounce_samples,   // e.g. 3..10
                              uint32_t sample_ms,          // e.g. 1..5
                              uint32_t timeout_ms,         // e.g. 4000
							  const char *dbg_tag,
							  uint8_t async_slot);

void motor_until_sensor_abort(void);


#endif /* INC_MOTOR_UNTIL_H_ */
