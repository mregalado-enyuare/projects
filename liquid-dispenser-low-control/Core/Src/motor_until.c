/*
 * motor_until.c
 *
 *  Created on: Nov 11, 2025
 *      Author: eostroff_enyuare
 */

// motor_until.c
#include <stdio.h>
#include "motor_until.h"
#include "lwip/timeouts.h"
#include "async_bus.h"     // <-- you were using async_evt_push_isr
#include "dc_motor.h"      // <-- for dc_motor_* API

typedef struct {
  uint8_t      active:1;
  uint8_t      sensor_active_low:1;
  uint8_t      require_edge:1;      // optional: only accept after seeing opposite state once
  uint8_t      armed:1;             // internal: have we seen the opposite state?
  uint8_t      pass_printed:1;
  dc_motor_t  *motor;
  GPIO_TypeDef *sensor_port;
  uint16_t     sensor_pin;
  uint16_t     need_samples;
  uint16_t     have_samples;
  uint32_t     sample_ms;
  uint32_t     timeout_ms;
  uint32_t     t0_ms;
  uint8_t      slot;
  const char *dbg_tag;
} motor_until_ctx_t;

static motor_until_ctx_t g = {0};

static inline void motor_debounce_fail(const char *tag, uint16_t got, uint16_t req)
{
  if (got > 0 && got < req) {
    printf("[%s SENSOR] debounce fail (%u/%u)\r\n",
           tag ? tag : "MOTOR", (unsigned)got, (unsigned)req);
  }
}

static inline void motor_debounce_pass(const char *tag, uint16_t req)
{
  printf("[%s SENSOR] debounce pass (%u/%u)\r\n",
         tag ? tag : "MOTOR", (unsigned)req, (unsigned)req);
}

static inline bool sensor_is_active(GPIO_TypeDef *port, uint16_t pin, bool active_low) {
  GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
  bool high = (s == GPIO_PIN_SET);
  return active_low ? !high : high;
}

static void tick(void *arg) {
  (void)arg;
  if (!g.active) return;

  bool act = sensor_is_active(g.sensor_port, g.sensor_pin, g.sensor_active_low);

  // Optional edge-arming (disabled by default; behaves like cup_dispense when false)
  if (g.require_edge) {
    // arm when we see opposite of "active"
    if (!g.armed) {
      if (!act) g.armed = 1;  // require seeing inactive once before accepting active
    }
  } else {
    g.armed = 1; // behave like cup_dispense: accept immediately
  }

  // --- debounce sample ---
  if (g.armed && act) {
    if (g.have_samples < g.need_samples) {
      g.have_samples++;
      if (g.have_samples == g.need_samples && !g.pass_printed) {
        motor_debounce_pass(g.dbg_tag, g.need_samples);
        g.pass_printed = 1;
      }
    }
  } else {
    motor_debounce_fail(g.dbg_tag, g.have_samples, g.need_samples);
    g.have_samples = 0;
    g.pass_printed = 0; // allow pass print next time we achieve it again
  }

  // success: debounced trip
  if (g.have_samples >= g.need_samples) {
    dc_motor_stop(g.motor);        // stop cleanly
    g.active = 0;
    (void)async_evt_push_isr(g.slot, ASYNC_RES_OK);
    return;
  }

  // monitor motor outcome
  dc_motor_state_t st = dc_motor_status(g.motor);
  if (st == dc_motor_state_done_stalled) {
    g.active = 0;
    (void)async_evt_push_isr(g.slot, ASYNC_RES_STALLED);
    return;
  } else if (st == dc_motor_state_done_ok || st == dc_motor_state_aborted) {
    // motor finished before sensor tripped → failure
    g.active = 0;
    (void)async_evt_push_isr(g.slot, ASYNC_RES_STALLED);
    return;
  }

  // timeout
  uint32_t now = HAL_GetTick();
  if ((now - g.t0_ms) >= g.timeout_ms) {
    dc_motor_abort(g.motor);
    g.active = 0;
    (void)async_evt_push_isr(g.slot, ASYNC_RES_STALLED);
    return;
  }

  // keep polling
  sys_timeout(g.sample_ms, tick, NULL);
}

bool motor_until_sensor_start(dc_motor_t *motor,
                              bool forward,
                              uint32_t steps_mag,
                              GPIO_TypeDef *sensor_port, uint16_t sensor_pin, bool sensor_active_low,
                              uint16_t debounce_samples,
                              uint32_t sample_ms,
                              uint32_t timeout_ms,
							  const char *dbg_tag,
                              uint8_t async_slot)
{
  if (!motor || !sensor_port || sensor_pin == 0) return false;
  if (debounce_samples == 0 || sample_ms == 0 || timeout_ms == 0) return false;
  if (g.active) return false;
  if (dc_motor_is_busy(motor)) return false;

  if (steps_mag == 0) steps_mag = 100000; // big default run; we’ll abort on sensor/timeout

  g.active            = 1;
  g.sensor_active_low = sensor_active_low ? 1 : 0;
  g.require_edge      = 0;                 // set to 1 if you want "must see inactive first"
  g.armed             = 0;
  g.pass_printed      = 0;
  g.motor             = motor;
  g.sensor_port       = sensor_port;
  g.sensor_pin        = sensor_pin;
  g.need_samples      = debounce_samples;
  g.have_samples      = 0;
  g.sample_ms         = sample_ms;
  g.timeout_ms        = timeout_ms;
  g.t0_ms             = HAL_GetTick();
  g.slot              = async_slot;
  g.dbg_tag 		  = dbg_tag ? dbg_tag : "MOTOR";

  // kick the motor (large run; we’ll abort on sensor/timeout)
  int32_t steps = forward ? (int32_t)steps_mag : -(int32_t)steps_mag;
  if (!dc_motor_start_move(motor, steps)) {
    g.active = 0;
    return false;
  }

  sys_timeout(g.sample_ms, tick, NULL);
  return true;
}

void motor_until_sensor_abort(void) {
  if (!g.active) return;
  g.active = 0;
  dc_motor_stop(g.motor);
  (void)async_evt_push_isr(g.slot, ASYNC_RES_ABORTED);
}
