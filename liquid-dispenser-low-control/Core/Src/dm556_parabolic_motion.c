/*
 * dm556_parabolic_motion.c
 *
 *  Created on: Sep 22, 2025
 *      Author: eostroff_enyuare
 *
 *  UPDATED (Option C):
 *    - X2 step output moved to TIM23_CH3 (same timer as X1 TIM23_CH1)
 *    - Locked motion pulses CH1+CH3 from the SAME counter/timeline
 *    - TIM13 is no longer used for X2 step generation
 *
 *  Key idea:
 *    - TIM23 is the ONLY motion timeline interrupt.
 *    - On each tick, we decide whether to pulse X and/or Y using DDA accumulators.
 *    - X2 mirrors X pulses by using another channel on the same timer (TIM23_CH3).
 */

#include "main.h"
#include "dm556_parabolic_motion.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "lwip.h"
#include "async_bus.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

// NOTE: ASYNC_SLOT_NONE sentinel used to suppress internal DONE events.
// This comment is intentionally harmless and forces a file reparse in the IDE.

/* ========================= GLOBALS / CONSTANTS ========================= */

gantry_async_t g_gantry = {0};
goto_seq_t g_goto_seq   = {0};

volatile uint8_t g_circle_slot_valid = 0;
volatile uint8_t g_circle_slot       = 0;

#define DUTY_NUMERATOR            (1)
#define DUTY_DENOMINATOR          (2)
#define ARR_FLOOR                 (2U)
#define ARR_CEIL                  (0xFFFFFFFFU)

#define DM556_MAX_STEPS_DEFAULT   (31000U)
const uint32_t DM556_MAX_STEPS_PER_MOVE = DM556_MAX_STEPS_DEFAULT;

/* ========================= TIMER HANDLES ========================= */

// X1 = TIM23_CH1 (master tick)
// X2 = TIM23_CH3 (same timer, second channel)
// Y  = TIM5_CH1
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim5;

extern IWDG_HandleTypeDef hiwdg1;

/* ========================= PROFILE BUFFERS (match header externs) ========================= */

uint32_t x_profile[DM556_MAX_PROFILE_STEPS];
uint32_t y_profile[DM556_MAX_PROFILE_STEPS];

/* ========================= MOTOR OBJECTS (match header externs) ========================= */

ParabolicMotorControl_t motorX = {
  .htim        = &htim23,
  .timChannel  = TIM_CHANNEL_1,
  .dirPort     = GANTRY_X_DIR_GPIO_Port,
  .dirPin      = GANTRY_X_DIR_Pin,
  .state       = MOTOR_IDLE,
  .arrProfile  = NULL,
  .stepsPlanned= 0,
  .stepIndex   = 0
};

ParabolicMotorControl_t motorY = {
  .htim        = &htim5,
  .timChannel  = TIM_CHANNEL_1,
  .dirPort     = GANTRY_Y_DIR_GPIO_Port,
  .dirPin      = GANTRY_Y_DIR_Pin,
  .state       = MOTOR_IDLE,
  .arrProfile  = NULL,
  .stepsPlanned= 0,
  .stepIndex   = 0
};

// *** UPDATED: X2 now uses TIM23_CH3 ***
ParabolicMotorControl_t motorX2 = {
  .htim        = &htim23,
  .timChannel  = TIM_CHANNEL_3,
  .dirPort     = GANTRY_X2_DIR_GPIO_Port,
  .dirPin      = GANTRY_X2_DIR_Pin,
  .state       = MOTOR_IDLE,
  .arrProfile  = NULL,
  .stepsPlanned= 0,
  .stepIndex   = 0
};

/* ========================= X AXIS MODE ========================= */

volatile XAxisMode_t g_xmode = XMODE_LOCKED;

bool XAxis_SetMode(XAxisMode_t mode)
{
  if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorX2)) return false;
  g_xmode = mode;
  return true;
}

/* ========================= FORWARD DECLS ========================= */

static void Motor_Enable(ParabolicMotorControl_t *m);
static void Motor_Disable(ParabolicMotorControl_t *m);
static void Motor_SetDir(ParabolicMotorControl_t *m, bool positive);

static void gantry_maybe_publish_done_from_irq(void);

static inline bool pin_active(GPIO_TypeDef *port, uint16_t pin, bool active_low);

/* ========================= DIR HELPERS (preserve your polarity mapping) ========================= */

static inline void SetDirX_Positive(void) {
  Motor_SetDir(&motorX,  false);
  Motor_SetDir(&motorX2, false);
}
static inline void SetDirX_Negative(void) {
  Motor_SetDir(&motorX,  true);
  Motor_SetDir(&motorX2, true);
}
static inline void SetDirY_Positive(void) { Motor_SetDir(&motorY, false); }
static inline void SetDirY_Negative(void) { Motor_SetDir(&motorY, true ); }

/* ========================= TIMER CLOCK HELPERS ========================= */

uint32_t TimerInputClock_Hz(TIM_HandleTypeDef *htim)
{
  RCC_ClkInitTypeDef clkcfg;
  uint32_t lat;
  HAL_RCC_GetClockConfig(&clkcfg, &lat);

  bool apb2 = (htim->Instance == TIM1) || (htim->Instance == TIM8);

  uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

  uint32_t mul  = apb2
    ? ((clkcfg.APB2CLKDivider == RCC_HCLK_DIV1) ? 1U : 2U)
    : ((clkcfg.APB1CLKDivider == RCC_HCLK_DIV1) ? 1U : 2U);

  return pclk * mul;
}

uint32_t Freq_To_ARR(uint32_t timerHz, double freq)
{
  if (freq < 1.0) freq = 1.0;

  double arrd = ((double)timerHz / freq) - 1.0;

  if (arrd < (double)ARR_FLOOR) arrd = (double)ARR_FLOOR;
  if (arrd > (double)ARR_CEIL)  arrd = (double)ARR_CEIL;

  return (uint32_t)(arrd + 0.5);
}

// With X2 on TIM23, the old TIM13 16-bit limit is irrelevant.
// Keep symbol for compatibility; now it's a no-op clamp.
float Clamp_XAxis_SpeedHz(float requestedHz)
{
  return requestedHz;
}

/* ========================= PARABOLIC ARR PROFILE ========================= */

void BuildParabolicARRProfile(uint32_t steps, uint32_t f_min, uint32_t f_peak,
                              uint32_t timerHz, uint32_t *outArr)
{
  if (!outArr || steps == 0) return;

  if (steps == 1) {
    uint32_t arr = (f_min > 0) ? (uint32_t)((double)timerHz / (double)f_min - 1.0) : ARR_CEIL;
    if (arr < ARR_FLOOR) arr = ARR_FLOOR;
    outArr[0] = arr;
    return;
  }

  double fmin  = (double)f_min;
  double fpeak = (double)((f_peak >= f_min) ? f_peak : f_min);

  for (uint32_t k = 0; k < steps; ++k) {
    double u  = (double)k / (double)(steps - 1);
    double s  = 1.0 - (2.0*u - 1.0)*(2.0*u - 1.0);
    double fv = fmin + (fpeak - fmin) * s;

    if (fv < 1.0) fv = 1.0;

    double arrd = ((double)timerHz / fv) - 1.0;

    if (arrd < (double)ARR_FLOOR) arrd = (double)ARR_FLOOR;
    if (arrd > (double)ARR_CEIL)  arrd = (double)ARR_CEIL;

    outArr[k] = (uint32_t)(arrd + 0.5);
  }
}

/* ========================= MOTOR LOW-LEVEL ========================= */

bool Motor_IsBusy(const ParabolicMotorControl_t* m)
{
  return m->state != MOTOR_IDLE;
}

static void Motor_SetDir(ParabolicMotorControl_t *m, bool positive)
{
  HAL_GPIO_WritePin(m->dirPort, m->dirPin,
                    positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_SetDir_Public(ParabolicMotorControl_t *m, bool dirPos)
{
  if (!m) return;
  if (dirPos) Motor_SetDir(m, false);
  else        Motor_SetDir(m, true);
}

static void Motor_Enable(ParabolicMotorControl_t *m)
{
  HAL_TIM_PWM_Start(m->htim, m->timChannel);

  // IMPORTANT:
  // Base IT is only needed for master-tick moves and legacy profiled moves.
  // It is safe to start it multiple times; HAL handles it.
  HAL_TIM_Base_Start_IT(m->htim);
}

static void Motor_Disable(ParabolicMotorControl_t *m)
{
  HAL_TIM_PWM_Stop(m->htim, m->timChannel);

  // IMPORTANT: TIM23 is shared by motorX (CH1) and motorX2 (CH3).
  // Do NOT stop the TIM23 base interrupt here, because the other X channel
  // (or a DDA engine) may still be using it.
  //
  // TIM5 (Y) is not shared with X, so it's safe to stop its base interrupt.
  if (m->htim != motorX.htim) {
    HAL_TIM_Base_Stop_IT(m->htim);
  }
}


/* ========================= EXTRA MOTOR REGISTRY ========================= */

typedef struct {
  ParabolicMotorControl_t *m;
  void (*on_done_isr)(void *ctx);
  void *ctx;
} ExtraMotor_t;

static ExtraMotor_t s_extras[DM556_EXTRA_MOTORS_MAX];
static int          s_nextra = 0;

void Motor_RegisterExtra(ParabolicMotorControl_t *m,
                         void (*on_done_isr)(void *ctx),
                         void *ctx)
{
  if (s_nextra < DM556_EXTRA_MOTORS_MAX) {
    s_extras[s_nextra].m           = m;
    s_extras[s_nextra].on_done_isr = on_done_isr;
    s_extras[s_nextra].ctx         = ctx;
    s_nextra++;
  }
}

/* ========================= GENERIC MOVE-UNTIL-LIMIT ========================= */

bool Motor_MoveUntilLimit(ParabolicMotorControl_t *m,
                          bool toward_positive, float speed_hz,
                          GPIO_TypeDef *port, uint16_t pin, bool active_low,
                          bool want_pressed,
                          uint32_t samples, uint32_t sample_ms)
{
  if (!m || !port) return false;
  if (Motor_IsBusy(m)) return false;

  Motor_RunConstant_Public(m, toward_positive, speed_hz);

  uint32_t stable = 0;
  for (;;) {
    stable = (pin_active(port, pin, active_low) == want_pressed) ? (stable + 1U) : 0U;
    if (stable >= samples) break;
    HAL_Delay(sample_ms);
    MX_LWIP_Process();
    HAL_IWDG_Refresh(&hiwdg1);
  }

  Motor_StopNow(m);
  return true;
}

void Motor_StopNow(ParabolicMotorControl_t *m)
{
  if (!m) return;

  __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, 0);
  HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);

  HAL_TIM_PWM_Stop(m->htim, m->timChannel);

  // Shared timer rule: don't kill TIM23 base IT if the other X channel is still running
  if (m->htim != motorX.htim) {
    HAL_TIM_Base_Stop_IT(m->htim);
  } else {
    bool other_busy = false;
    if (m == &motorX)  other_busy = (motorX2.state != MOTOR_IDLE);
    if (m == &motorX2) other_busy = (motorX.state  != MOTOR_IDLE);
    if (!other_busy) {
      HAL_TIM_Base_Stop_IT(m->htim);
    }
  }

  m->state = MOTOR_IDLE;
  __DMB();
}

/* ========================= MOTOR PROFILED MOVE (legacy; used for homing/tests) ========================= */

void Motor_StartMove(ParabolicMotorControl_t *m, uint32_t *arrBuf, uint32_t steps)
{
  if (!m || !arrBuf || steps == 0) return;

  m->arrProfile   = arrBuf;
  m->stepsPlanned = steps;
  m->stepIndex    = 0;
  m->state        = MOTOR_MOVING;
  __DMB();

  __HAL_TIM_DISABLE(m->htim);
  __HAL_TIM_SET_AUTORELOAD(m->htim, m->arrProfile[0]);
  __HAL_TIM_SET_COMPARE(m->htim, m->timChannel,
                        (m->arrProfile[0] * DUTY_NUMERATOR) / DUTY_DENOMINATOR);
  __HAL_TIM_SET_COUNTER(m->htim, 0);

  __HAL_TIM_URS_ENABLE(m->htim);
  HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);
  __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);
  __HAL_TIM_URS_DISABLE(m->htim);

  Motor_Enable(m);
}

bool Motor_MoveParabolic(ParabolicMotorControl_t *m,
                         uint32_t stepsAbs, bool dirPos,
                         uint32_t f_min, uint32_t f_peak,
                         uint32_t *scratchProfileBuf, uint32_t scratchLen)
{
  if (!m || stepsAbs == 0 || !scratchProfileBuf) return false;
  if (scratchLen < stepsAbs) return false;

  if (dirPos) Motor_SetDir(m, false);
  else        Motor_SetDir(m, true);

  uint32_t timerHz = TimerInputClock_Hz(m->htim);
  BuildParabolicARRProfile(stepsAbs, f_min, f_peak, timerHz, scratchProfileBuf);
  Motor_StartMove(m, scratchProfileBuf, stepsAbs);
  return true;
}

/* ========================= CONSTANT SPEED RUN/STOP ========================= */

static void Motor_RunConstant(ParabolicMotorControl_t *m, bool dirPos, float steps_per_sec)
{
  const uint32_t timerHz = TimerInputClock_Hz(m->htim);
  uint32_t arr = Freq_To_ARR(timerHz, (double)steps_per_sec);
  if (arr < ARR_FLOOR) arr = ARR_FLOOR;

  if (dirPos) Motor_SetDir(m, false);
  else        Motor_SetDir(m, true);

  // OPTION C: X and X2 share TIM23. Do NOT disable/reset the timer when starting
  // the second channel — it can cause slight pulse misalignment.
  if (m->htim == motorX.htim) {
    // Update shared ARR once (safe if same request); do not reset CNT.
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, arr / 2U);
    HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);

    // Just enable this PWM channel
    HAL_TIM_PWM_Start(m->htim, m->timChannel);
    return;
  }

  // Non-shared timers (ex: TIM5 for Y): legacy behavior ok
  __HAL_TIM_DISABLE(m->htim);
  __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
  __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, arr / 2U);
  __HAL_TIM_SET_COUNTER(m->htim, 0);

  HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_PWM_Start(m->htim, m->timChannel);
}


static void Motor_StopConstant(ParabolicMotorControl_t *m)
{
  __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, 0);
  HAL_TIM_GenerateEvent(m->htim, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_PWM_Stop(m->htim, m->timChannel);
}


void Motor_RunConstant_Public(ParabolicMotorControl_t *m,
                              bool toward_positive,
                              float steps_per_sec)
{
  Motor_RunConstant(m, toward_positive, steps_per_sec);
}

/* ========================= SPEED PLANNING FOR SEQUENCES ========================= */

float max_speed_for_move(float speed_cap, float accel, int32_t dx, int32_t dy)
{
  float D = sqrtf((float)dx * (float)dx + (float)dy * (float)dy);
  if (D < 1e-6f) return 0.0f;

  float steps_to_max = (speed_cap - MIN_SPEED) / accel;

  if (D >= steps_to_max) return speed_cap;
  return (accel * D + MIN_SPEED);
}

/* =======================================================================
 *                          CIRCLE DDA (updated for TIM23_CH3 X2)
 * ======================================================================= */

typedef enum {
  CPH_IDLE = 0,
  CPH_TO_RIM,
  CPH_ARC,
  CPH_BACK_Y,
  CPH_BACK_X
} CirclePhase_t;

typedef struct {
  volatile bool    active;
  volatile uint8_t stopping;
  CirclePhase_t    phase;
  uint32_t         rim_steps;
  uint32_t         ticks_left;
  float            xf, yf;
  int32_t          xi, yi;
  float            cs, sn;
  float            ex, ey;
  float            mixAcc;
  uint32_t         baseARR;
  bool             dirXpos, dirYpos;
  uint32_t         back_y_steps;
  uint32_t         back_x_steps;
  uint32_t         nominalARR;
} CircleDDA_t;

static CircleDDA_t g_circle = {0};

static inline void Pulse_X_only(void) {
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  g_circle.baseARR / 2U);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, g_circle.baseARR / 2U);
  __HAL_TIM_SET_COMPARE(motorY.htim,  motorY.timChannel,  0);
}
static inline void Pulse_Y_only(void) {
  __HAL_TIM_SET_COMPARE(motorY.htim,  motorY.timChannel,  g_circle.baseARR / 2U);
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);
}
static inline void Pulse_None(void) {
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);
  __HAL_TIM_SET_COMPARE(motorY.htim,  motorY.timChannel,  0);
}

bool Circle_IsActive(void) { return g_circle.active; }

static inline void Circle_StopArm(void)
{
  Pulse_None();
  g_circle.stopping = 1;
}

static inline void Circle_ApplySpeedRatio(float ratio)
{
  if (!(ratio > 0.0f && ratio <= 1.0f)) return;

  uint32_t scaledARR = (uint32_t)fmaxf(
    (float)ARR_FLOOR,
    floorf(((float)g_circle.nominalARR + 1.0f) / ratio - 1.0f)
  );

  if (g_circle.baseARR != scaledARR) {
    g_circle.baseARR = scaledARR;
    __HAL_TIM_SET_AUTORELOAD(motorX.htim, scaledARR);
    __HAL_TIM_SET_AUTORELOAD(motorY.htim, scaledARR);
  }
}

static inline void Circle_ApplyNominalSpeed(void)
{
  if (g_circle.baseARR != g_circle.nominalARR) {
    g_circle.baseARR = g_circle.nominalARR;
    __HAL_TIM_SET_AUTORELOAD(motorX.htim, g_circle.nominalARR);
    __HAL_TIM_SET_AUTORELOAD(motorY.htim, g_circle.nominalARR);
  }
}

void Circle_Stop(void)
{
  if (!g_circle.active) return;
  Circle_StopArm();
}

cmd_status_t Circle_Start_Async(uint32_t count, uint32_t radius,
                                float speedHz, bool clockwise,
                                uint8_t async_slot)
{
  if (count == 0 || radius == 0 || speedHz <= 0.0f) return CMD_ERR_BADARGS;
  if (g_circle.active) return CMD_ERR_BUSY;
  if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorY) || Motor_IsBusy(&motorX2)) return CMD_ERR_BUSY;
  if (g_gantry.active) return CMD_ERR_BUSY;

  g_circle_slot       = async_slot;
  g_circle_slot_valid = 1;

  Circle_Start(count, radius, speedHz, clockwise);
  return CMD_OK_STARTED;
}

void Circle_Abort_Async(void)
{
  if (!g_circle_slot_valid) return;
  uint8_t slot = g_circle_slot;
  g_circle_slot_valid = 0;
  Circle_Stop();
  if (slot != ASYNC_SLOT_NONE) {
    (void)async_evt_push_isr(slot, ASYNC_RES_ABORTED);
  }
}

void Circle_Start(uint32_t count, uint32_t radius, float speedHz, bool clockwise)
{
  if (count == 0U || radius == 0U || speedHz <= 0.0f) return;

  const uint32_t ticks_per_circle = 8u * radius;
  if (ticks_per_circle == 0U) return;

  const uint32_t timerHz = TimerInputClock_Hz(motorX.htim);
  const uint32_t arr = Freq_To_ARR(timerHz, speedHz);

  float dtheta = 1.0f / (float)radius;
  if (clockwise) dtheta = -dtheta;

  g_circle.active     = true;
  g_circle.phase      = CPH_TO_RIM;
  g_circle.rim_steps  = radius;
  g_circle.ticks_left = ticks_per_circle * count;

  g_circle.cs = cosf(dtheta);
  g_circle.sn = sinf(dtheta);

  g_circle.xf = (float)radius;
  g_circle.yf = 0.0f;
  g_circle.xi = 0;
  g_circle.yi = 0;

  g_circle.ex = 0.0f;
  g_circle.ey = 0.0f;
  g_circle.mixAcc = 0.0f;

  g_circle.baseARR    = arr;
  g_circle.nominalARR = arr;

  g_circle.dirXpos = true;
  g_circle.dirYpos = !clockwise;
  if (g_circle.dirXpos) SetDirX_Positive(); else SetDirX_Negative();
  if (g_circle.dirYpos) SetDirY_Positive(); else SetDirY_Negative();

  __HAL_TIM_SET_AUTORELOAD(motorX.htim,  arr);
  __HAL_TIM_SET_AUTORELOAD(motorY.htim,  arr);

  Pulse_None();

  // Start BOTH X channels on TIM23 (CH1 handled by Motor_Enable; start CH3 explicitly)
  HAL_TIM_PWM_Start(motorX2.htim, motorX2.timChannel);

  Motor_Enable(&motorX);   // starts TIM23 base IT + CH1 PWM
  Motor_Enable(&motorY);   // ok to keep as before (TIM5 base IT used in your existing logic)
}

/* ... Circle_Tick() UNCHANGED except it no longer stops TIM13 ... */
static inline void Circle_Tick(void)
{
  if (!g_circle.active) return;

  if (g_circle.stopping) {
    g_circle.stopping = 0;

    HAL_TIM_PWM_Stop(motorX.htim,  motorX.timChannel);
    HAL_TIM_PWM_Stop(motorX2.htim, motorX2.timChannel);
    HAL_TIM_PWM_Stop(motorY.htim,  motorY.timChannel);

    HAL_TIM_Base_Stop_IT(motorX.htim);
    HAL_TIM_Base_Stop_IT(motorY.htim);

    g_circle.active = false;
    __DMB();

    if (g_circle_slot_valid) {
        uint8_t slot = g_circle_slot;
        g_circle_slot_valid = 0;
        if (slot != ASYNC_SLOT_NONE) {
            (void)async_evt_push_isr(slot, ASYNC_RES_OK);
        }
    }
    return;
  }

  /* keep your existing circle algorithm exactly as-is */
  /* (omitted here for brevity – paste your existing ARC code below unchanged) */

  /* >>> START: paste your existing Circle_Tick body from your current file <<< */

  // Use the same reduced speed for center->rim and the return-to-center phases.
  const float RIM_SPEED_RATIO = 0.35f;

  if (g_circle.phase == CPH_TO_RIM) {
    Circle_ApplySpeedRatio(RIM_SPEED_RATIO);

    if (g_circle.rim_steps == 0U) {
      Circle_ApplyNominalSpeed();
      g_circle.phase = CPH_ARC;
      Pulse_None();
      return;
    }

    if (!g_circle.dirXpos) { g_circle.dirXpos = true; SetDirX_Positive(); }
    Pulse_X_only();
    g_circle.xi += 1;
    g_circle.rim_steps--;
    return;
  }

  if (g_circle.phase == CPH_ARC) {
    if (g_circle.ticks_left == 0U) {
      const int32_t yi = g_circle.yi;
      const int32_t xi = g_circle.xi;
      g_circle.back_y_steps = (uint32_t)((yi >= 0) ? yi : -yi);
      g_circle.back_x_steps = (uint32_t)((xi >= 0) ? xi : -xi);
      Circle_ApplySpeedRatio(RIM_SPEED_RATIO);

      if (g_circle.back_y_steps > 0U) {
        g_circle.phase = CPH_BACK_Y;
        const bool step_pos = (yi < 0);
        if (step_pos != g_circle.dirYpos) {
          g_circle.dirYpos = step_pos;
          if (step_pos) SetDirY_Positive(); else SetDirY_Negative();
        }
        Pulse_Y_only();
        g_circle.yi += (g_circle.dirYpos ? +1 : -1);
        g_circle.back_y_steps--;
      } else if (g_circle.back_x_steps > 0U) {
        g_circle.phase = CPH_BACK_X;
        const bool step_pos = (xi < 0);
        if (step_pos != g_circle.dirXpos) {
          g_circle.dirXpos = step_pos;
          if (step_pos) SetDirX_Positive(); else SetDirX_Negative();
        }
        Pulse_X_only();
        g_circle.xi += (g_circle.dirXpos ? +1 : -1);
        g_circle.back_x_steps--;
      } else {
        Circle_Stop();
      }
      return;
    }

    const float xn =  g_circle.xf * g_circle.cs - g_circle.yf * g_circle.sn;
    const float yn =  g_circle.xf * g_circle.sn + g_circle.yf * g_circle.cs;
    g_circle.xf = xn; g_circle.yf = yn;

    const float tx = xn - (float)g_circle.xi;
    const float ty = yn - (float)g_circle.yi;

    const float r  = fmaxf(1.0f, hypotf(g_circle.xf, g_circle.yf));
    const float vx = -g_circle.yf / r;
    const float vy =  g_circle.xf / r;

    const float ax = fabsf(vx), ay = fabsf(vy);
    const float denom = fmaxf(1e-6f, ax + ay);
    const float rho = ax / denom;

    g_circle.mixAcc += rho;

    if (g_circle.mixAcc >= 1.0f) {
      g_circle.mixAcc -= 1.0f;
      const bool step_pos = (vx >= 0.0f);
      if (step_pos != g_circle.dirXpos) {
        g_circle.dirXpos = step_pos;
        if (step_pos) SetDirX_Positive(); else SetDirX_Negative();
      }
      Pulse_X_only();
      g_circle.xi += (step_pos ? +1 : -1);
      g_circle.ex += (tx - (step_pos ? +1.0f : -1.0f));
    } else {
      const bool step_pos = (vy >= 0.0f);
      if (step_pos != g_circle.dirYpos) {
        g_circle.dirYpos = step_pos;
        if (step_pos) SetDirY_Positive(); else SetDirY_Negative();
      }
      Pulse_Y_only();
      g_circle.yi += (step_pos ? +1 : -1);
      g_circle.ey += (ty - (step_pos ? +1.0f : -1.0f));
    }

    g_circle.ticks_left--;
    return;
  }

  if (g_circle.phase == CPH_BACK_Y) {
    Circle_ApplySpeedRatio(RIM_SPEED_RATIO);

    if (g_circle.back_y_steps == 0U) {
      if (g_circle.back_x_steps > 0U) {
        g_circle.phase = CPH_BACK_X;
        const bool step_pos = (g_circle.xi < 0);
        if (step_pos != g_circle.dirXpos) {
          g_circle.dirXpos = step_pos;
          if (step_pos) SetDirX_Positive(); else SetDirX_Negative();
        }
        Pulse_X_only();
        g_circle.xi += (g_circle.dirXpos ? +1 : -1);
        g_circle.back_x_steps--;
      } else {
        Circle_ApplyNominalSpeed();
        Circle_Stop();
      }
      return;
    }
    Pulse_Y_only();
    g_circle.yi += (g_circle.dirYpos ? +1 : -1);
    g_circle.back_y_steps--;
    return;
  }

  if (g_circle.phase == CPH_BACK_X) {
    Circle_ApplySpeedRatio(RIM_SPEED_RATIO);

    if (g_circle.back_x_steps == 0U) {
      Circle_ApplyNominalSpeed();
      Circle_Stop();
      return;
    }
    Pulse_X_only();
    g_circle.xi += (g_circle.dirXpos ? +1 : -1);
    g_circle.back_x_steps--;
    return;
  }

  /* >>> END: paste your existing Circle_Tick body <<< */
}

/* =======================================================================
 *                      MASTER-TICK LINE DDA ENGINE (updated for TIM23_CH3)
 * ======================================================================= */

typedef struct {
  volatile bool active;
  uint32_t ticks_total;
  uint32_t tick_idx;
  uint32_t ax, ay;
  uint32_t accX, accY;
  bool x_pos, y_pos;
  uint32_t *arr;    // points to x_profile (reused)
} LineDDA_t;

static LineDDA_t g_line = {0};

static inline void Line_Pulse_NoneAll(void)
{
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);
  __HAL_TIM_SET_COMPARE(motorY.htim,  motorY.timChannel,  0);
}

static inline void Line_Pulse_X(bool on, uint32_t arr)
{
  const uint32_t ccr = on ? (arr/2U) : 0U;
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  ccr);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, ccr);
}

static inline void Line_Pulse_Y(bool on, uint32_t arr)
{
  __HAL_TIM_SET_COMPARE(motorY.htim, motorY.timChannel, on ? (arr/2) : 0U);
}

static void Line_StartTimers(uint32_t arr0)
{
  if (arr0 < ARR_FLOOR) arr0 = ARR_FLOOR;

  __HAL_TIM_DISABLE(motorX.htim);
  __HAL_TIM_DISABLE(motorY.htim);

  __HAL_TIM_SET_AUTORELOAD(motorX.htim,  arr0);
  __HAL_TIM_SET_AUTORELOAD(motorY.htim,  arr0);

  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);
  __HAL_TIM_SET_COMPARE(motorY.htim,  motorY.timChannel,  0);

  __HAL_TIM_SET_COUNTER(motorX.htim,  0);
  __HAL_TIM_SET_COUNTER(motorY.htim,  0);

  HAL_TIM_GenerateEvent(motorX.htim,  TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_GenerateEvent(motorY.htim,  TIM_EVENTSOURCE_UPDATE);

  // Start X2 PWM channel on the SAME timer as X
  HAL_TIM_PWM_Start(motorX2.htim, motorX2.timChannel);

  // Start Y PWM (no base IT needed for Y here; you kept it that way)
  HAL_TIM_PWM_Start(motorY.htim, motorY.timChannel);
  HAL_TIM_Base_Start(motorY.htim);

  // Start master tick timeline (TIM23 base IT + CH1 PWM)
  Motor_Enable(&motorX);
}

static void Line_StopTimers(void)
{
  Line_Pulse_NoneAll();

  HAL_TIM_GenerateEvent(motorX.htim,  TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_GenerateEvent(motorY.htim,  TIM_EVENTSOURCE_UPDATE);

  // Stop TIM23
  HAL_TIM_PWM_Stop(motorX.htim,  motorX.timChannel);
  HAL_TIM_PWM_Stop(motorX2.htim, motorX2.timChannel);
  HAL_TIM_Base_Stop_IT(motorX.htim);

  // Stop TIM5
  HAL_TIM_Base_Stop(motorY.htim);
  HAL_TIM_PWM_Stop(motorY.htim, motorY.timChannel);

  motorX.state  = MOTOR_IDLE;
  motorX2.state = MOTOR_IDLE;
  motorY.state  = MOTOR_IDLE;
  __DMB();
}

static bool Line_Start(int32_t deltaX, int32_t deltaY,
                       float startSpeed, float peakSpeed)
{
  if (deltaX == 0 && deltaY == 0) return false;
  if (g_line.active) return false;
  if (Circle_IsActive()) return false;
  if (g_xmode != XMODE_LOCKED) return false;
  if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorY) || Motor_IsBusy(&motorX2)) return false;

  uint32_t ax = (deltaX >= 0) ? (uint32_t)deltaX : (uint32_t)(-deltaX);
  uint32_t ay = (deltaY >= 0) ? (uint32_t)deltaY : (uint32_t)(-deltaY);

  uint32_t ticks = (ax > ay) ? ax : ay;
  if (ticks == 0) return false;
  if (ticks > DM556_MAX_STEPS_PER_MOVE) return false;

  uint32_t timerHz = TimerInputClock_Hz(motorX.htim);
  uint32_t f_min   = (uint32_t)fmaxf(1.0f, startSpeed);
  uint32_t f_peak  = (uint32_t)fmaxf((float)f_min, peakSpeed);

  BuildParabolicARRProfile(ticks, f_min, f_peak, timerHz, x_profile);

  memset(&g_line, 0, sizeof(g_line));
  g_line.active      = true;
  g_line.ticks_total = ticks;
  g_line.tick_idx    = 0;
  g_line.ax          = ax;
  g_line.ay          = ay;
  g_line.accX        = 0;
  g_line.accY        = 0;
  g_line.x_pos       = (deltaX >= 0);
  g_line.y_pos       = (deltaY >= 0);
  g_line.arr         = x_profile;

  if (g_line.x_pos) SetDirX_Positive(); else SetDirX_Negative();
  if (g_line.y_pos) SetDirY_Positive(); else SetDirY_Negative();
  HAL_Delay(1);

  motorX.state  = MOTOR_MOVING;
  motorX2.state = MOTOR_MOVING;
  motorY.state  = MOTOR_MOVING;
  __DMB();

  uint32_t arr0 = x_profile[0];
  if (arr0 < ARR_FLOOR) arr0 = ARR_FLOOR;

  Line_StartTimers(arr0);
  return true;
}

static inline void Line_Tick(void)
{
  if (!g_line.active) return;

  uint32_t i = g_line.tick_idx;

  if (i >= g_line.ticks_total) {
    g_line.active = false;
    Line_StopTimers();
    __DMB();
    gantry_maybe_publish_done_from_irq();
    return;
  }

  uint32_t arr = g_line.arr[i];
  if (arr < ARR_FLOOR) arr = ARR_FLOOR;

  __HAL_TIM_SET_AUTORELOAD(motorX.htim,  arr);
  __HAL_TIM_SET_AUTORELOAD(motorY.htim,  arr);

  bool stepX = false;
  bool stepY = false;

  g_line.accX += g_line.ax;
  g_line.accY += g_line.ay;

  if (g_line.accX >= g_line.ticks_total) {
    g_line.accX -= g_line.ticks_total;
    stepX = (g_line.ax != 0);
  }
  if (g_line.accY >= g_line.ticks_total) {
    g_line.accY -= g_line.ticks_total;
    stepY = (g_line.ay != 0);
  }

  Line_Pulse_X(stepX, arr);
  Line_Pulse_Y(stepY, arr);

  g_line.tick_idx++;
}

/* ========================= PUBLIC GANTRY API (unchanged) ========================= */

void Gantry_Goto_Parabolic(int32_t deltaX, int32_t deltaY,
                           float startSpeed, float endSpeed)
{
  if (deltaX == 0 && deltaY == 0) return;
  if (!Line_Start(deltaX, deltaY, startSpeed, endSpeed)) return;

  while (g_line.active) {
    HAL_Delay(5);
    MX_LWIP_Process();
    HAL_IWDG_Refresh(&hiwdg1);
  }
}

cmd_status_t Gantry_Goto_Parabolic_Async(int32_t deltaX, int32_t deltaY,
                                         float startSpeed, float endSpeed,
                                         uint8_t async_slot)
{
  if (deltaX == 0 && deltaY == 0) return CMD_OK_IMMEDIATE;

  if (g_gantry.active) return CMD_ERR_BUSY;
  if (g_line.active)   return CMD_ERR_BUSY;
  if (Circle_IsActive()) return CMD_ERR_BUSY;
  if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorY) || Motor_IsBusy(&motorX2))
    return CMD_ERR_BUSY;
  if (g_xmode != XMODE_LOCKED) return CMD_ERR_BUSY;

  g_gantry.slot   = async_slot;
  g_gantry.active = 1;

  if (!Line_Start(deltaX, deltaY, startSpeed, endSpeed)) {
    g_gantry.active = 0;
    return CMD_ERR_BADARGS;
  }

  return CMD_OK_STARTED;
}

/* ========================= DONE-PUBLISH FROM IRQ (unchanged) ========================= */

static inline void gantry_maybe_publish_done_from_irq(void)
{
  if (!Motor_IsBusy(&motorX) && !Motor_IsBusy(&motorX2) && !Motor_IsBusy(&motorY)) {

    if (g_goto_seq.active) {
      g_gantry.active = 0;
      g_goto_seq.kick = 1;
      return;
    }

    if (g_gantry.active) {
      uint8_t slot = g_gantry.slot;
      g_gantry.active = 0;
      if (slot != ASYNC_SLOT_NONE) {
        (void)async_evt_push_isr(slot, ASYNC_RES_OK);
      }
    }
  }
}

// ------------------------------------------------------------
// Helper: pin active check (limit switches etc.)
// ------------------------------------------------------------
static inline bool pin_active(GPIO_TypeDef *port, uint16_t pin, bool active_low)
{
  GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
  return active_low ? (s == GPIO_PIN_RESET) : (s == GPIO_PIN_SET);
}

/* ========================= LIMIT SWITCH HELPERS (exported) ========================= */

static inline bool pin_active_limit(GPIO_TypeDef *port, uint16_t pin, bool active_low)
{
  GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
  return active_low ? (s == GPIO_PIN_RESET) : (s == GPIO_PIN_SET);
}

bool X_MoveUntil_Limit(bool toward_positive,
                       float speed_k,
                       GPIO_TypeDef *port, uint16_t pin, bool active_low,
                       bool want_pressed,
                       uint32_t samples, uint32_t sample_ms)
{
  if (pin_active(port, pin, active_low) == want_pressed) return true;
  if (g_xmode != XMODE_LOCKED) return false;
  if (Circle_IsActive() || g_line.active) return false;

  // X direction helper sets BOTH dir pins
  if (toward_positive) SetDirX_Positive();
  else                 SetDirX_Negative();

  const uint32_t timerHz = TimerInputClock_Hz(motorX.htim); // TIM23
  uint32_t arr = Freq_To_ARR(timerHz, (double)speed_k);
  if (arr < ARR_FLOOR) arr = ARR_FLOOR;

  // Make sure TIM23 counter is not running while we arm channels
  __HAL_TIM_DISABLE(motorX.htim);

  // TIM23 shared ARR
  __HAL_TIM_SET_AUTORELOAD(motorX.htim, arr);

  // Arm both channels to "off" first to avoid a start glitch
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);

  __HAL_TIM_SET_COUNTER(motorX.htim, 0);
  HAL_TIM_GenerateEvent(motorX.htim, TIM_EVENTSOURCE_UPDATE);

  // Enable BOTH PWM channels BEFORE starting the counter
  HAL_TIM_PWM_Start(motorX.htim,  motorX.timChannel);   // CH1
  HAL_TIM_PWM_Start(motorX2.htim, motorX2.timChannel);  // CH3

  // Now start the base (counter) ONCE (TIM23)
  HAL_TIM_Base_Start(motorX.htim);

  // Run with a steady 50% duty while we wait (both channels same CCR)
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  arr / 2U);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, arr / 2U);

  uint32_t stable = 0;
  for (;;) {
    bool pressed = pin_active(port, pin, active_low);
    if (pressed == want_pressed) {
      if (++stable >= samples) break;
    } else {
      stable = 0;
    }

    HAL_Delay(sample_ms);
    MX_LWIP_Process();
    HAL_IWDG_Refresh(&hiwdg1);
  }

  // Stop both channels cleanly
  __HAL_TIM_SET_COMPARE(motorX.htim,  motorX.timChannel,  0);
  __HAL_TIM_SET_COMPARE(motorX2.htim, motorX2.timChannel, 0);
  HAL_TIM_GenerateEvent(motorX.htim, TIM_EVENTSOURCE_UPDATE);

  HAL_TIM_PWM_Stop(motorX.htim,  motorX.timChannel);
  HAL_TIM_PWM_Stop(motorX2.htim, motorX2.timChannel);
  HAL_TIM_Base_Stop(motorX.htim);

  return true;
}


bool Y_MoveUntil_Limit(bool toward_positive,
                       float speed_k,
                       GPIO_TypeDef *port, uint16_t pin, bool active_low,
                       bool want_pressed,
                       uint32_t samples, uint32_t sample_ms)
{
  if (pin_active_limit(port, pin, active_low) == want_pressed) return true;
  if (Circle_IsActive() || g_line.active) return false;
  if (Motor_IsBusy(&motorY)) return false;

  Motor_RunConstant_Public(&motorY, toward_positive, speed_k);

  uint32_t stable = 0;
  for (;;) {
    bool pressed = pin_active_limit(port, pin, active_low);
    stable = (pressed == want_pressed) ? (stable + 1U) : 0U;
    if (stable >= samples) break;

    HAL_Delay(sample_ms);
    MX_LWIP_Process();
    HAL_IWDG_Refresh(&hiwdg1);
  }

  Motor_StopNow(&motorY);
  return true;
}

/* ========================= GOTO SEQUENCE PUMP (exported) ========================= */

void GotoSeq_Pump(void)
{
  if (!g_goto_seq.active) return;

  if (g_goto_seq.kick &&
      !g_line.active &&
      !Motor_IsBusy(&motorX) &&
      !Motor_IsBusy(&motorX2) &&
      !Motor_IsBusy(&motorY))
  {
    g_goto_seq.kick = 0;

    if (g_goto_seq.idx < g_goto_seq.count) {
      int32_t dx = g_goto_seq.dx[g_goto_seq.idx];
      int32_t dy = g_goto_seq.dy[g_goto_seq.idx];

      if (dx == 0 && dy == 0) {
        g_goto_seq.idx++;
        g_goto_seq.kick = 1;
        return;
      }

      float v_peak = max_speed_for_move(
          (float)(g_goto_seq.speed_k * SPEED_SCALE),
          (float)g_goto_seq.accel_k,
          dx, dy
      );
      if (v_peak < g_goto_seq.v_min) v_peak = g_goto_seq.v_min;

      cmd_status_t st = Gantry_Goto_Parabolic_Async(dx, dy, g_goto_seq.v_min, v_peak, ASYNC_SLOT_NONE);
      if (st == CMD_ERR_BUSY) {
        g_goto_seq.kick = 1;
        return;
      }

      g_goto_seq.idx++;
      return;
    }

    uint8_t user_slot = g_goto_seq.slot;
    g_goto_seq.active = 0;
    (void)async_evt_push_isr(user_slot, ASYNC_RES_OK);
    printf("GOTO sequence done\n");
  }
}


/* ========================= MOTOR IRQ SERVICE HOOK ========================= */

void Motor_ServiceIRQ(TIM_HandleTypeDef *htim)
{
  if (!htim) return;

  if (htim == motorX.htim) {
    if (Circle_IsActive()) { Circle_Tick(); return; }
    if (g_line.active)     { Line_Tick();   return; }
  }

  // Legacy single-axis stepping for other modes if needed
  ParabolicMotorControl_t *motors[] = { &motorX, &motorX2, &motorY };
  const size_t N = sizeof(motors) / sizeof(motors[0]);

  for (size_t i = 0; i < N; ++i) {
    ParabolicMotorControl_t *m = motors[i];
    if (!m) continue;
    if (m->htim != htim) continue;
    if (m->state != MOTOR_MOVING) continue;
    if (!m->arrProfile) continue;

    uint32_t emitted = m->stepIndex + 1;

    if (emitted >= m->stepsPlanned) {
      __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, 0);
      HAL_TIM_PWM_Stop(m->htim, m->timChannel);

      __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);

      m->stepIndex = m->stepsPlanned;
      m->state     = MOTOR_IDLE;
      __DMB();

      // If this motor is on TIM23, stop base IT only when BOTH X channels are idle
      if (m->htim == motorX.htim) {
        if (motorX.state == MOTOR_IDLE && motorX2.state == MOTOR_IDLE) {
          HAL_TIM_Base_Stop_IT(m->htim);
        }
      } else {
        HAL_TIM_Base_Stop_IT(m->htim);
      }

      gantry_maybe_publish_done_from_irq();
      continue;
    }


    m->stepIndex = emitted;
    uint32_t nextARR = m->arrProfile[emitted];
    if (nextARR < ARR_FLOOR) nextARR = ARR_FLOOR;

    __HAL_TIM_SET_AUTORELOAD(m->htim, nextARR);
    __HAL_TIM_SET_COMPARE(m->htim, m->timChannel,
                          (nextARR * DUTY_NUMERATOR) / DUTY_DENOMINATOR);
  }

  // Step extra registered motors (e.g. lid steppers)
  for (int ei = 0; ei < s_nextra; ei++) {
    ParabolicMotorControl_t *m = s_extras[ei].m;
    if (!m || m->htim != htim) continue;
    if (m->state != MOTOR_MOVING || !m->arrProfile) continue;

    uint32_t emitted = m->stepIndex + 1;

    if (emitted >= m->stepsPlanned) {
      __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, 0);
      HAL_TIM_PWM_Stop(m->htim, m->timChannel);
      __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);
      m->stepIndex = m->stepsPlanned;
      m->state     = MOTOR_IDLE;
      __DMB();

      // Stop base IT only if no other extra motor on the same timer is still moving
      bool other_running = false;
      for (int ej = 0; ej < s_nextra; ej++) {
        if (ej == ei) continue;
        if (s_extras[ej].m && s_extras[ej].m->htim == htim &&
            s_extras[ej].m->state != MOTOR_IDLE) {
          other_running = true;
          break;
        }
      }
      if (!other_running) HAL_TIM_Base_Stop_IT(m->htim);

      if (s_extras[ei].on_done_isr) s_extras[ei].on_done_isr(s_extras[ei].ctx);
      continue;
    }

    m->stepIndex = emitted;
    uint32_t nextARR = m->arrProfile[emitted];
    if (nextARR < ARR_FLOOR) nextARR = ARR_FLOOR;
    __HAL_TIM_SET_AUTORELOAD(m->htim, nextARR);
    __HAL_TIM_SET_COMPARE(m->htim, m->timChannel,
                          (nextARR * DUTY_NUMERATOR) / DUTY_DENOMINATOR);
  }
}
