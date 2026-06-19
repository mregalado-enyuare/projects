/*
 * keg_dispense.h
 *
 *  Created on: Nov 4, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_KEG_DISPENSE_H_
#define INC_KEG_DISPENSE_H_

#include "stm32h7xx_hal.h"
#include "lwip/timeouts.h"
#include "async_bus.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t  active;
  GPIO_TypeDef *fet_port;
  uint16_t fet_pin;
  uint32_t t0_ms;
  uint32_t duration_ms;
  uint8_t  slot;
} keg_dispense_ctx_t;

// Start a non-blocking "FET ON for duration → FET OFF → DONE" cycle.
// Returns false if already active or bad args.
// On completion: pushes ASYNC_RES_OK to the async queue.
bool keg_dispense_start(GPIO_TypeDef *fet_port, uint16_t fet_pin,
                        uint32_t duration_ms, uint8_t async_slot);

// Optional abort (turns FET off, cancels timer, signals ABORTED).
void keg_dispense_abort(void);

// Optional: check if an operation is in progress.
bool keg_dispense_busy(void);

// keg_dispense.h
void keg_dispense_on_timer_irq(void);   // call from HAL_TIM_PeriodElapsedCallback

#endif /* INC_KEG_DISPENSE_H_ */
