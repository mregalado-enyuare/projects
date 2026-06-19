/*
 * dc_motor.c
 *
 *  Created on: Oct 28, 2025
 *      Author: eostroff_enyuare
 */

#include "dc_motor.h"
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
extern UART_HandleTypeDef huart3;

/*
// 50% Duty PWM
  __HAL_TIM_SET_COMPARE(htim_pwm, pwm_channel, arr / 2.0);
  HAL_TIM_PWM_Start(htim_pwm, pwm_channel);

  // Count encoder magnitude with wrap-safe arithmetic
  uint32_t prev = __HAL_TIM_GET_COUNTER(htim_enc);
  int32_t total = 0;
  int32_t target = (steps > 0) ? steps : -steps;

  const uint32_t stallMs           = 1000;     // couple milliseconds of no encoder updates
  const int32_t  toleranceSteps    = 400;    // how close to target counts as successful move

  // timing
  uint32_t start_ms      = HAL_GetTick();
  uint32_t last_move_ms  = start_ms;

  bool success = false;
  bool stalled_near_goal = false;www

  while (total < target) {
    uint32_t now_cnt = __HAL_TIM_GET_COUNTER(htim_enc);
    int32_t diff = (int32_t) (now_cnt - prev); // handles wrap

    if (diff != 0) {
      if (diff < 0) diff = -diff; // magnitude only
      total += diff;
      prev = now_cnt;
      last_move_ms = HAL_GetTick(); // motion seen, reset stall timer
    }

    // check for stall near goal
    uint32_t now_ms = HAL_GetTick();

    // If we've seen no encoder change for stallMs, evaluate proximity to target
    if ((now_ms - last_move_ms) >= stallMs) {
      int32_t remaining = target - total;
      if (remaining <= toleranceSteps) {
        success = true;           // close enough—call it good
        stalled_near_goal = true; // will brake the motor to keep pressure
      }
      break;                     // either way, stop driving if we're stalled
    }
  }
  */

// --- Local helpers -----------------------------------------------------------
static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) {
    return lo;
  }

  if (v > hi) {
    return hi;
  }

  return v;
}

static inline uint32_t enc_get_raw(dc_motor_t *m) {
  return __HAL_TIM_GET_COUNTER(m->htim_enc);
}

static inline int32_t enc_delta_mag(dc_motor_t *m, uint32_t now_raw, uint32_t prev_raw) {
  if (m->enc_is_16bit) {
    int16_t diff = (int16_t)((uint16_t)now_raw - (uint16_t)prev_raw);
    return (diff >= 0) ? diff : -diff;
  } else {
    int32_t diff = (int32_t)((uint32_t)now_raw - (uint32_t)prev_raw);
    return (diff >= 0) ? diff : -diff;
  }
}

static inline void pwm_apply(dc_motor_t *m, bool enable) {
  if (!enable) {
    HAL_TIM_PWM_Stop(m->htim_pwm, m->pwm_channel);
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    return;
  }

  uint32_t arr = m->htim_pwm->Instance->ARR;
  uint32_t ccr = (arr * clamp_u32(m->pwm_duty_pct, 0, 100)) / 100U;
  if (ccr == 0) { ccr = 1; }              // avoid 0% corner case if needed
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, ccr);
  HAL_TIM_PWM_Start(m->htim_pwm, m->pwm_channel);

  // clear any pending update and enable its interrupt
  __HAL_TIM_CLEAR_FLAG(m->htim_pwm, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(m->htim_pwm);
}

static inline void set_dir(dc_motor_t *m, bool forward) {
  if (forward) {
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
  }
}

static inline void enc_start(dc_motor_t *m) {
  __HAL_TIM_SET_COUNTER(m->htim_enc, 0);
  HAL_TIM_Encoder_Start(m->htim_enc, TIM_CHANNEL_ALL);
}

static inline void enc_stop(dc_motor_t *m) {
  HAL_TIM_Encoder_Stop(m->htim_enc, TIM_CHANNEL_ALL);
}

void dc_motor_set_done_cb(dc_motor_t *m, dc_motor_done_cb_t cb, void *user) {
  if (!m) return;
  m->on_done = cb;
  m->user = user;
}

static void finish_move(dc_motor_t *m, dc_motor_state_t end_state) {
  pwm_apply(m, false);
  enc_stop(m);
  m->state = end_state;
  // ISR context: only signal
  if (m->on_done) m->on_done(m->user, end_state);
}

// --- Public API --------------------------------------------------------------

void dc_motor_init(dc_motor_t *m,
                   GPIO_TypeDef *in1_port, uint16_t in1_pin,
                   GPIO_TypeDef *in2_port, uint16_t in2_pin,
                   TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel,
                   TIM_HandleTypeDef *htim_enc, bool enc_is_16bit) {
  if (!m) return;
  m->in1_port = in1_port; m->in1_pin = in1_pin;
  m->in2_port = in2_port; m->in2_pin = in2_pin;
  m->htim_pwm = htim_pwm; m->pwm_channel = pwm_channel;
  m->htim_enc = htim_enc; m->enc_is_16bit = enc_is_16bit;

  m->pwm_duty_pct = DC_MOTOR_DEFAULT_PWM_DUTY_PCT;
  m->stall_window_ms = DC_MOTOR_STALL_WINDOW_MS;
  m->goal_tolerance_counts = DC_MOTOR_GOAL_TOLERANCE_COUNTS;

  m->state = dc_motor_state_idle;
  m->target_counts = 0;
  m->total_counts = 0;
  m->last_enc_raw = 0;
  m->last_motion_ms = 0;
  m->forward = true;
}

void dc_motor_set_duty(dc_motor_t *m, uint32_t duty_pct) {
  if (!m) return;
  m->pwm_duty_pct = clamp_u32(duty_pct, 0, 100);
}

void dc_motor_set_stall_params(dc_motor_t *m, uint32_t stall_window_ms,
                               int32_t goal_tolerance_counts) {
  if (!m) return;
  m->stall_window_ms = stall_window_ms;
  m->goal_tolerance_counts = goal_tolerance_counts;
}

bool dc_motor_start_move(dc_motor_t *m, int32_t steps) {
  if (!m || steps == 0) return false;
  if (m->state == dc_motor_state_moving_up || m->state == dc_motor_state_moving_down) {
    return false; // busy
  }

  m->forward = (steps > 0);
  m->target_counts = (steps > 0) ? steps : -steps;
  m->total_counts = 0;
  set_dir(m, m->forward);
  enc_start(m);

  m->last_enc_raw = enc_get_raw(m);
  m->last_motion_ms = HAL_GetTick();
  m->state = m->forward ? dc_motor_state_moving_up : dc_motor_state_moving_down;

  pwm_apply(m, true);
  return true;
}

void dc_motor_on_timer_irq(dc_motor_t *m) {
  if (!m) return;
  if (m->state != dc_motor_state_moving_up && m->state != dc_motor_state_moving_down) {
    return; // nothing to do
  }

  uint32_t now_raw = enc_get_raw(m);
  int32_t dmag = enc_delta_mag(m, now_raw, m->last_enc_raw);
  if (dmag > 0) {
    m->total_counts += dmag;
    m->last_enc_raw = now_raw;
    m->last_motion_ms = HAL_GetTick();

    char buf[48];
    int n = snprintf(buf, sizeof(buf), "enc: %ld / %ld\r\n",
                     m->total_counts, m->target_counts);
    if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
  }

  // Completion check
  if (m->total_counts >= m->target_counts) {
    finish_move(m, dc_motor_state_done_ok);
    printf("DC Motor move done: target %ld counts reached (total %ld)\r\n",
           m->target_counts, m->total_counts);
    return;
  }

  // Stall check
  uint32_t now_ms = HAL_GetTick();
  if ((now_ms - m->last_motion_ms) >= m->stall_window_ms) {
    // close enough is success, otherwise stall
    int32_t remaining = m->target_counts - m->total_counts;
    if (remaining <= m->goal_tolerance_counts) {
      finish_move(m, dc_motor_state_done_ok);
    } else {
      finish_move(m, dc_motor_state_done_stalled);
    }
  }
}

void dc_motor_abort(dc_motor_t *m) {
  if (!m) return;
  finish_move(m, dc_motor_state_aborted);
}

void dc_motor_stop(dc_motor_t *m) {
  if (!m) return;
  finish_move(m, dc_motor_state_done_ok);
}

bool dc_motor_is_busy(const dc_motor_t *m) {
  if (!m) return false;
  return (m->state == dc_motor_state_moving_up || m->state == dc_motor_state_moving_down);
}

dc_motor_state_t dc_motor_status(const dc_motor_t *m) {
  if (!m) return dc_motor_state_aborted;
  return m->state;
}
