/*
 * dc_motor.h
 *
 *  Created on: Oct 28, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"   // Adjust device family include as needed

// --- Configuration constants -------------------------------------------------
// Tune these to your mechanism. All time values are in milliseconds.
#define DC_MOTOR_DEFAULT_PWM_DUTY_PCT     50U     // percent (0..100)
#define DC_MOTOR_STALL_WINDOW_MS          1000U   // no encoder change for this long => stall
#define DC_MOTOR_GOAL_TOLERANCE_COUNTS    400     // within this of goal counts as success

// --- Types -------------------------------------------------------------------

typedef enum {
  dc_motor_state_idle = 0,
  dc_motor_state_moving_up,
  dc_motor_state_moving_down,
  dc_motor_state_done_ok,
  dc_motor_state_done_stalled,
  dc_motor_state_aborted,
} dc_motor_state_t;

typedef void (*dc_motor_done_cb_t)(void *user, dc_motor_state_t st);

// Hardware binding for a single motor/encoder/PWM set
typedef struct {
  // Direction H-bridge pins (set/reset per polarity)
  GPIO_TypeDef *in1_port;
  uint16_t      in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t      in2_pin;

  // PWM output timer + channel (e.g., TIM24_CHx)
  TIM_HandleTypeDef *htim_pwm;
  uint32_t           pwm_channel; // TIM_CHANNEL_x

  // Quadrature encoder timer (TIM in encoder mode)
  TIM_HandleTypeDef *htim_enc;
  bool               enc_is_16bit; // true if 16-bit counter

  // Runtime planning
  uint32_t pwm_duty_pct;          // 0..100
  uint32_t stall_window_ms;       // ms of no movement => stall
  int32_t  goal_tolerance_counts; // counts within goal -> success

  // Movement tracking
  volatile dc_motor_state_t state;
  volatile int32_t          target_counts;  // |steps| requested
  volatile int32_t          total_counts;   // magnitude accumulated
  volatile uint32_t         last_enc_raw;   // raw counter snapshot
  volatile uint32_t         last_motion_ms; // HAL_GetTick() when last moved
  volatile bool             forward;        // direction requested (>0)

  // For sending done asynchronously
  dc_motor_done_cb_t on_done; // called in ISR context
  void              *user;    // slot index
} dc_motor_t;

void dc_motor_set_done_cb(dc_motor_t *m, dc_motor_done_cb_t cb, void *user);

// --- API ---------------------------------------------------------------------

void dc_motor_init(dc_motor_t *m,
                   GPIO_TypeDef *in1_port, uint16_t in1_pin,
                   GPIO_TypeDef *in2_port, uint16_t in2_pin,
                   TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel,
                   TIM_HandleTypeDef *htim_enc, bool enc_is_16bit);

void dc_motor_set_duty(dc_motor_t *m, uint32_t duty_pct);
void dc_motor_set_stall_params(dc_motor_t *m, uint32_t stall_window_ms,
                               int32_t goal_tolerance_counts);

// Start a non-blocking move. Positive steps => "forward" polarity, negative => reverse.
// Returns false if already busy or steps == 0.
bool dc_motor_start_move(dc_motor_t *m, int32_t steps);

// Periodic service called from HAL_TIM_PeriodElapsedCallback for a *free* basic timer
// (e.g., TIM6/TIM7) at ~1 kHz. You can service multiple motors by calling this for each.
void dc_motor_on_timer_irq(dc_motor_t *m);

// Control / status helpers
void dc_motor_abort(dc_motor_t *m);
void dc_motor_stop(dc_motor_t *m);
bool dc_motor_is_busy(const dc_motor_t *m);
dc_motor_state_t dc_motor_status(const dc_motor_t *m);

#endif // DC_MOTOR_H_
