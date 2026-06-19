// Core/Src/ice_dispenser.c
#include "ice_dispenser.h"
#include "main.h"          // CUBE_ICE_* / PEBBLE_ICE_* pin macros
#include "async_bus.h"     // async_evt_push_isr(...)
#include "lwip/timeouts.h" // sys_timeout(...)
#include <stdio.h>
#include <string.h>

#ifndef ICE_PULSE_MS
#define ICE_PULSE_MS  50u   // length of LOW pulse to "tap" the Arduino
#endif

// Convenience aliases to the pins defined in CubeMX (main.h)
#define CUBE_ICE_TRIG_PORT   CUBE_ICE_TRIG_GPIO_Port
#define CUBE_ICE_TRIG_PIN    CUBE_ICE_TRIG_Pin
#define CUBE_ICE_DONE_PIN    CUBE_ICE_DONE_Pin

#define PEBBLE_ICE_TRIG_PORT PEBBLE_ICE_TRIG_GPIO_Port
#define PEBBLE_ICE_TRIG_PIN  PEBBLE_ICE_TRIG_Pin
#define PEBBLE_ICE_DONE_PIN  PEBBLE_ICE_DONE_Pin

typedef struct {
  volatile uint8_t  active;
  uint8_t           slot;
  uint32_t          last_irq_ms;

  GPIO_TypeDef     *trig_port;
  uint16_t          trig_pin;
  uint16_t          done_pin;
} ice_ctx_t;

static ice_ctx_t g_ice[ICE_MACHINE_COUNT];

static inline uint32_t tick_now(void)
{
  return HAL_GetTick();
}

static void ice_release_pulse_cb(void *arg)
{
  ice_ctx_t *ctx = (ice_ctx_t *)arg;
  // Release the open-drain line: let it float HIGH via pull-up
  HAL_GPIO_WritePin(ctx->trig_port, ctx->trig_pin, GPIO_PIN_SET);
}

/* -------------------- public API -------------------- */

void ice_dispenser_init(void)
{
  memset(g_ice, 0, sizeof(g_ice));

  // CUBE machine wiring
  g_ice[ICE_MACHINE_CUBE].trig_port = CUBE_ICE_TRIG_PORT;
  g_ice[ICE_MACHINE_CUBE].trig_pin  = CUBE_ICE_TRIG_PIN;
  g_ice[ICE_MACHINE_CUBE].done_pin  = CUBE_ICE_DONE_PIN;

  // PEBBLE machine wiring
  g_ice[ICE_MACHINE_PEBBLE].trig_port = PEBBLE_ICE_TRIG_PORT;
  g_ice[ICE_MACHINE_PEBBLE].trig_pin  = PEBBLE_ICE_TRIG_PIN;
  g_ice[ICE_MACHINE_PEBBLE].done_pin  = PEBBLE_ICE_DONE_PIN;

  // Ensure all trigger lines idle HIGH (open-drain released).
  for (int i = 0; i < ICE_MACHINE_COUNT; ++i) {
    HAL_GPIO_WritePin(g_ice[i].trig_port, g_ice[i].trig_pin, GPIO_PIN_SET);
  }
}

bool ice_dispenser_start(ice_machine_t id, uint8_t async_slot)
{
  if (id >= ICE_MACHINE_COUNT) return false;
  ice_ctx_t *ctx = &g_ice[id];

  if (ctx->active) return false;

  ctx->active      = 1;
  ctx->slot        = async_slot;
  ctx->last_irq_ms = 0;

  printf("Ice dispenser %d: START\n", id);

  // Assert a LOW pulse on the open-drain output to simulate a button press.
  HAL_GPIO_WritePin(ctx->trig_port, ctx->trig_pin, GPIO_PIN_RESET);

  // Auto-release after ICE_PULSE_MS
  sys_timeout(ICE_PULSE_MS, ice_release_pulse_cb, ctx);

  return true;
}

bool ice_dispenser_busy(ice_machine_t id)
{
  if (id >= ICE_MACHINE_COUNT) return false;
  return g_ice[id].active != 0;
}

void ice_dispenser_abort(ice_machine_t id)
{
  if (id >= ICE_MACHINE_COUNT) return;
  ice_ctx_t *ctx = &g_ice[id];

  if (!ctx->active) return;

  // Make sure trigger line is released high
  HAL_GPIO_WritePin(ctx->trig_port, ctx->trig_pin, GPIO_PIN_SET);
  ctx->active = 0;
}

// Called from EXTI handler with the pin that fired
void ice_dispenser_on_done_exti(uint16_t gpio_pin)
{
  uint32_t now_ms = tick_now();

  for (int i = 0; i < ICE_MACHINE_COUNT; ++i) {
    ice_ctx_t *ctx = &g_ice[i];

    if (ctx->done_pin != gpio_pin) {
      continue;
    }

    // Debounce: ignore if within 100 ms of last IRQ
    if (now_ms - ctx->last_irq_ms < 100u) {
      return;
    }
    ctx->last_irq_ms = now_ms;

    if (!ctx->active) {
      // spurious DONE?
      return;
    }

    ctx->active = 0;

    printf("Ice dispenser %d: DONE\n", i);

    (void)async_evt_push_isr(ctx->slot, ASYNC_RES_OK);
    return; // handled
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Route ice-done lines into the dispenser logic
    ice_dispenser_on_done_exti(GPIO_Pin);

    // If you have other EXTI users, dispatch them too, e.g.:
    // cup_dispense_on_exti(GPIO_Pin);
    // some_other_module_on_exti(GPIO_Pin);
}
