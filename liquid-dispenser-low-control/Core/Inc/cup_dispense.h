/*
 * cup_dispense.h
 *
 *  Created on: Nov 3, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_CUP_DISPENSE_H_
#define INC_CUP_DISPENSE_H_

#include "lwip/timeouts.h"
#include "async_bus.h"

typedef struct {
  uint8_t active:1;
  uint8_t sensor_active_low:1;
  GPIO_TypeDef *fet_port;
  uint16_t fet_pin;
  GPIO_TypeDef *sensor_port;
  uint16_t sensor_pin;
  uint16_t need_samples;
  uint16_t have_samples;
  uint32_t sample_ms;
  uint32_t t0_ms;
  uint32_t timeout_ms;
  uint8_t  slot;
} fet_until_ctx_t;

// Start a non-blocking "set FET -> wait for sensor (debounced) -> clear FET" cycle.
// Returns false if a run is already active or args invalid.
// On completion: pushes ASYNC_RES_OK (sensor tripped) or ASYNC_RES_STALLED (timeout) to async queue.
bool fet_until_sensor_start(GPIO_TypeDef *fet_port, uint16_t fet_pin,
                            GPIO_TypeDef *sensor_port, uint16_t sensor_pin,
                            bool sensor_active_low,
                            uint16_t debounce_samples,   // e.g. 100 samples
                            uint32_t sample_ms,          // e.g. 1 ms
                            uint32_t timeout_ms,         // e.g. 4000 ms
                            uint8_t async_slot);

// Optional abort (turns FET off and signals ABORTED)
void fet_until_sensor_abort(void);

#endif /* INC_CUP_DISPENSE_H_ */
