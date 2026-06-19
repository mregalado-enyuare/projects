// Core/Inc/ice_dispenser.h
#ifndef ICE_DISPENSER_H
#define ICE_DISPENSER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

// Each hardware ice machine with its own TRIG/DONE pair
typedef enum {
  ICE_MACHINE_CUBE = 0,
  ICE_MACHINE_PEBBLE,
  ICE_MACHINE_COUNT
} ice_machine_t;

// Initialize internal state and make trigger lines idle HIGH
void ice_dispenser_init(void);

// Start a dispense on a given machine, associate with async_slot
// Returns false if that machine is already active.
bool ice_dispenser_start(ice_machine_t id, uint8_t async_slot);

// True while that machine is still waiting for DONE
bool ice_dispenser_busy(ice_machine_t id);

// Abort a dispense on that machine (no DONE event pushed)
void ice_dispenser_abort(ice_machine_t id);

// Called from EXTI callback with the GPIO pin that fired
void ice_dispenser_on_done_exti(uint16_t gpio_pin);

#endif // ICE_DISPENSER_H
