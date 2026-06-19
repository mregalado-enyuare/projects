/*
 * cup_dispense.c
 *
 *  Created on: Nov 3, 2025
 *      Author: eostroff_enyuare
 */

#include "cup_dispense.h"

static fet_until_ctx_t g_fet_until = {0};

/* Print statement helpers for debounce pass/fail */

static inline void fet_debounce_fail(GPIO_TypeDef *port, uint16_t pin_mask,
                                     uint16_t got, uint16_t req)
{
  if (got > 0 && got < req) {
    printf("[CUP SENSOR] debounce fail (%u/%u)\r\n", (unsigned)got, (unsigned)req);
  }
}

static inline void fet_debounce_pass(GPIO_TypeDef *port, uint16_t pin_mask,
                                     uint16_t req)
{
	printf("[CUP SENSOR] debounce pass (%u/%u)\r\n", (unsigned)req, (unsigned)req);
}

/* FET until sensor active implementation */


static inline bool sensor_is_active(GPIO_TypeDef *port, uint16_t pin, bool active_low) {
  GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
  bool high = (s == GPIO_PIN_SET);
  return active_low ? !high : high;
}

static void fet_until_tick(void *arg) {
  (void)arg;
  if (!g_fet_until.active) return;

  // Debounce sample
  bool act = sensor_is_active(g_fet_until.sensor_port,
                              g_fet_until.sensor_pin,
                              g_fet_until.sensor_active_low);
  if (act) {
      if (g_fet_until.have_samples < g_fet_until.need_samples) {
        g_fet_until.have_samples++;
        if (g_fet_until.have_samples == g_fet_until.need_samples) {
          fet_debounce_pass(g_fet_until.sensor_port,
                            g_fet_until.sensor_pin,
                            g_fet_until.need_samples);
        }
      }
    } else {
      fet_debounce_fail(g_fet_until.sensor_port,
                        g_fet_until.sensor_pin,
                        g_fet_until.have_samples,
                        g_fet_until.need_samples);
      g_fet_until.have_samples = 0;
    }

  // Success path: debounced active
  if (g_fet_until.have_samples >= g_fet_until.need_samples) {
    // turn FET off
    HAL_GPIO_WritePin(g_fet_until.fet_port, g_fet_until.fet_pin, GPIO_PIN_RESET);
    g_fet_until.active = 0;
    // notify
    (void)async_evt_push_isr(g_fet_until.slot, ASYNC_RES_OK);
    return;
  }

  // Timeout?
  uint32_t now = HAL_GetTick();
  if ((now - g_fet_until.t0_ms) >= g_fet_until.timeout_ms) {
    HAL_GPIO_WritePin(g_fet_until.fet_port, g_fet_until.fet_pin, GPIO_PIN_RESET);
    g_fet_until.active = 0;
    (void)async_evt_push_isr(g_fet_until.slot, ASYNC_RES_STALLED);
    return;
  }

  // Keep polling
  sys_timeout(g_fet_until.sample_ms, fet_until_tick, NULL);
}

bool fet_until_sensor_start(GPIO_TypeDef *fet_port, uint16_t fet_pin,
                            GPIO_TypeDef *sensor_port, uint16_t sensor_pin,
                            bool sensor_active_low,
                            uint16_t debounce_samples,
                            uint32_t sample_ms,
                            uint32_t timeout_ms,
                            uint8_t async_slot)
{
  if (!fet_port || !sensor_port || debounce_samples == 0 || sample_ms == 0 || timeout_ms == 0)
    return false;
  if (g_fet_until.active) return false; // single op at a time (can be extended to multi)

  g_fet_until.active            = 1;
  g_fet_until.sensor_active_low = sensor_active_low ? 1 : 0;
  g_fet_until.fet_port          = fet_port;
  g_fet_until.fet_pin           = fet_pin;
  g_fet_until.sensor_port       = sensor_port;
  g_fet_until.sensor_pin        = sensor_pin;
  g_fet_until.need_samples      = debounce_samples;
  g_fet_until.have_samples      = 0;
  g_fet_until.sample_ms         = sample_ms;
  g_fet_until.timeout_ms        = timeout_ms;
  g_fet_until.t0_ms             = HAL_GetTick();
  g_fet_until.slot              = async_slot;

  // Turn FET ON immediately
  HAL_GPIO_WritePin(fet_port, fet_pin, GPIO_PIN_SET);

  // Kick first poll
  sys_timeout(g_fet_until.sample_ms, fet_until_tick, NULL);
  return true;
}

void fet_until_sensor_abort(void) {
  if (!g_fet_until.active) return;
  g_fet_until.active = 0;
  HAL_GPIO_WritePin(g_fet_until.fet_port, g_fet_until.fet_pin, GPIO_PIN_RESET);
  (void)async_evt_push_isr(g_fet_until.slot, ASYNC_RES_ABORTED);
}
