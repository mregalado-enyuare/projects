#include "homing_x.h"
#include "dm556_parabolic_motion.h"
#include "async_bus.h"
#include "lwip.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern ParabolicMotorControl_t motorX;   // Gantry X  = RIGHT motor (TIM23_CH1)
extern ParabolicMotorControl_t motorX2;  // Gantry X2 = LEFT  motor (TIM23_CH3)
extern ParabolicMotorControl_t motorY;

extern bool   Motor_IsBusy(const ParabolicMotorControl_t* m);
extern void   Motor_StopNow(ParabolicMotorControl_t *m);
extern void   Motor_RunConstant_Public(ParabolicMotorControl_t *m,
                                       bool toward_positive,
                                       float steps_per_sec);
extern bool   Motor_MoveParabolic(ParabolicMotorControl_t *m,
                                  uint32_t stepsAbs, bool dirPos,
                                  uint32_t f_min, uint32_t f_peak,
                                  uint32_t *scratchProfileBuf, uint32_t scratchLen);

extern void   Motor_SetDir_Public(ParabolicMotorControl_t *m, bool dirPos);

extern float  Clamp_XAxis_SpeedHz(float requestedHz);
extern uint32_t TimerInputClock_Hz(TIM_HandleTypeDef *htim);

extern cmd_status_t Gantry_Goto_Parabolic_Async(int32_t deltaX, int32_t deltaY,
                                               float startSpeed, float endSpeed,
                                               uint8_t async_slot);

#ifndef SPEED_SCALE
#define SPEED_SCALE 1000.0f
#endif
#ifndef MIN_SPEED
#define MIN_SPEED 4000.0f
#endif

#ifndef GANTRY_X_LIMIT_GPIO_Port
#error "Need GANTRY_X_LIMIT_GPIO_Port / GANTRY_X_LIMIT_Pin for RIGHT motor limit."
#endif
#ifndef GANTRY_X2_LIMIT_GPIO_Port
#error "Need GANTRY_X2_LIMIT_GPIO_Port / GANTRY_X2_LIMIT_Pin for LEFT motor limit."
#endif

static const bool     HX_LIMIT_ACTIVE_LOW      = false;
static const bool     HX_WANT_PRESSED          = true;

/* ---- Debounce + sampling + timeout ----
 * Separate debounce counts:
 *  - FAST seek has its own debounce (like you asked)
 *  - SLOW seek + UNTIL_LIMIT_ONE share the original HX_DEBOUNCE_SAMPLES
 *  - UNTIL_PIN has its own debounce count (like you asked)
 */
static const uint16_t HX_DEBOUNCE_SAMPLES           = 12;  // slow_seek + until_limit_one
static const uint16_t HX_FAST_DEBOUNCE_SAMPLES      = 5;   // fast_seek (tune me)
static const uint16_t HX_UNTIL_PIN_DEBOUNCE_SAMPLES = 30;  // dual_until_pin (tune me)

static const uint32_t HX_SAMPLE_MS             = 1;
static const uint32_t HX_TIMEOUT_MS            = 12000;

static const bool     HX_HOME_TOWARD_POSITIVE  = false;
static const bool     HX_BACKOFF_TOWARD_POSITIVE = true;

static const float    HX_FAST_SEEK_SPEED_HZ    = 5000.0f;
static const float    HX_SLOW_SEEK_SPEED_HZ    = 2000.0f;

static const uint32_t HX_BACKOFF_STEPS         = 1000;
static const float    HX_BACKOFF_VMIN_HZ       = 1250.0f;
static const float    HX_BACKOFF_VPEAK_HZ      = 2500.0f;

typedef enum {
    HX_NONE = 0,
    HX_MOVE_STEPS,
    HX_UNTIL_LIMIT_ONE,

    HX_HOME_DUAL_FAST_SEEK,
    HX_HOME_DUAL_BACKOFF,
    HX_HOME_DUAL_SLOW_SEEK,

    HX_DUAL_UNTIL_PIN
} hx_mode_t;

typedef struct {
    volatile uint8_t active;
    hx_mode_t mode;
    uint8_t slot;

    bool left_running;
    bool right_running;

    uint32_t start_ms;
    uint32_t next_sample_ms;

    uint16_t left_stable;
    uint16_t right_stable;

    GPIO_TypeDef *shared_port;
    uint16_t      shared_pin;
    bool          shared_active_low;
    uint16_t      shared_stable;
} homing_x_state_t;

static homing_x_state_t g_hx = {0};
static volatile async_result_t g_hx_last_result = ASYNC_RES_ABORTED;

extern uint32_t x_profile[];
extern const uint32_t DM556_MAX_STEPS_PER_MOVE;

/* print only on transitions:
 *  - when a streak breaks (fail)
 *  - when a streak reaches required count (pass)
 */
static inline void hx_debounce_fail(const char *tag, uint16_t got, uint16_t req)
{
    if (got > 0 && got < req) {
        printf("%s debounce fail (%u/%u)\r\n",
                      tag, (unsigned)got, (unsigned)req);
    }
}

static inline void hx_debounce_pass(const char *tag, uint16_t req)
{
    printf("%s debounce pass (%u/%u)\r\n",
                  tag, (unsigned)req, (unsigned)req);
}

static inline bool pin_active(GPIO_TypeDef *port, uint16_t pin, bool active_low)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
    return active_low ? (s == GPIO_PIN_RESET) : (s == GPIO_PIN_SET);
}

static void finish(async_result_t r)
{
    g_hx_last_result = r;
    uint8_t slot = g_hx.slot;
    memset(&g_hx, 0, sizeof(g_hx));
    (void)async_evt_push_isr(slot, r);
}

bool HomingX_IsActive(void) { return g_hx.active != 0; }

void HomingX_Abort(void)
{
    if (!g_hx.active) return;

    Motor_StopNow(&motorX);
    Motor_StopNow(&motorX2);

    XAxis_SetMode(XMODE_LOCKED);
    finish(ASYNC_RES_ABORTED);
}

async_result_t HomingX_LastResult(void)
{
    return g_hx_last_result;
}

/* ------------------------------------------------------------
 * MOVE_X_INDEPENDENT (single motor parabolic move)
 * ------------------------------------------------------------ */
cmd_status_t HomingX_MoveIndependentSteps_Async(bool left_motor,
                                               int32_t steps_signed,
                                               int speed_k,
                                               int accel_k,
                                               uint8_t async_slot)
{
    (void)accel_k;

    if (steps_signed == 0) return CMD_OK_IMMEDIATE;
    if (g_hx.active) return CMD_ERR_BUSY;

    bool toward_positive = (steps_signed > 0);
    uint32_t steps_abs = (steps_signed > 0) ? (uint32_t)steps_signed
                                            : (uint32_t)(-steps_signed);

    if (steps_abs == 0) return CMD_OK_IMMEDIATE;
    if (steps_abs > DM556_MAX_STEPS_PER_MOVE) return CMD_ERR_BADARGS;

    if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorX2) || Motor_IsBusy(&motorY))
        return CMD_ERR_BUSY;

    if (!XAxis_SetMode(XMODE_INDEPENDENT)) return CMD_ERR_BUSY;

    ParabolicMotorControl_t *m = left_motor ? &motorX2 : &motorX;

    // Ensure the OTHER motor is not running
    Motor_StopNow(left_motor ? &motorX : &motorX2);

    float v_min  = (float)MIN_SPEED;
    float v_peak = (float)(speed_k * SPEED_SCALE);

    v_min  = Clamp_XAxis_SpeedHz(v_min);
    v_peak = Clamp_XAxis_SpeedHz(v_peak);

    bool ok = Motor_MoveParabolic(m,
                                 steps_abs,
                                 toward_positive,
                                 (uint32_t)v_min,
                                 (uint32_t)v_peak,
                                 x_profile,
                                 DM556_MAX_PROFILE_STEPS);

    if (!ok) {
        XAxis_SetMode(XMODE_LOCKED);
        return CMD_ERR_BADARGS;
    }

    memset(&g_hx, 0, sizeof(g_hx));
    g_hx.active = 1;
    g_hx.mode   = HX_MOVE_STEPS;
    g_hx.slot   = async_slot;

    if (left_motor)  g_hx.left_running  = 1;
    else             g_hx.right_running = 1;

    return CMD_OK_STARTED;
}

/* ------------------------------------------------------------
 * MOVE_X_INDEPENDENT_UNTIL_LIMIT
 * ------------------------------------------------------------ */
cmd_status_t HomingX_MoveIndependentUntilLimit_Async(bool motor_left,
                                                     int speed_k,
                                                     uint8_t async_slot)
{
    if (g_hx.active) return CMD_ERR_BUSY;

    if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorX2) || Motor_IsBusy(&motorY))
        return CMD_ERR_BUSY;

    if (!XAxis_SetMode(XMODE_INDEPENDENT)) return CMD_ERR_BUSY;

    float speedHz = (float)(speed_k * SPEED_SCALE);
    speedHz = Clamp_XAxis_SpeedHz(speedHz);

    memset(&g_hx, 0, sizeof(g_hx));
    g_hx.active = 1;
    g_hx.mode   = HX_UNTIL_LIMIT_ONE;
    g_hx.slot   = async_slot;
    g_hx.start_ms = HAL_GetTick();
    g_hx.next_sample_ms = g_hx.start_ms;

    // reset debounce counters (important)
    g_hx.left_stable  = 0;
    g_hx.right_stable = 0;

    if (motor_left) {
        g_hx.left_running = 1;
        Motor_RunConstant_Public(&motorX2, HX_HOME_TOWARD_POSITIVE, speedHz);
    } else {
        g_hx.right_running = 1;
        Motor_RunConstant_Public(&motorX, HX_HOME_TOWARD_POSITIVE, speedHz);
    }

    return CMD_OK_STARTED;
}

/* ------------------------------------------------------------
 * HOME_X_DUAL
 * ------------------------------------------------------------ */
cmd_status_t HomingX_HomeDualUntilLimit_Async(int speed_k,
                                              uint8_t async_slot)
{
    (void)speed_k;

    if (g_hx.active) return CMD_ERR_BUSY;

    if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorX2) || Motor_IsBusy(&motorY))
        return CMD_ERR_BUSY;

    if (!XAxis_SetMode(XMODE_INDEPENDENT)) return CMD_ERR_BUSY;

    float fastHz = Clamp_XAxis_SpeedHz(HX_FAST_SEEK_SPEED_HZ);

    memset(&g_hx, 0, sizeof(g_hx));
    g_hx.active = 1;
    g_hx.mode   = HX_HOME_DUAL_FAST_SEEK;
    g_hx.slot   = async_slot;
    g_hx.start_ms = HAL_GetTick();
    g_hx.next_sample_ms = g_hx.start_ms;

    g_hx.left_running  = 1;
    g_hx.right_running = 1;

    // reset debounce counters for fast seek
    g_hx.left_stable  = 0;
    g_hx.right_stable = 0;

    Motor_RunConstant_Public(&motorX,  HX_HOME_TOWARD_POSITIVE, fastHz);
    Motor_RunConstant_Public(&motorX2, HX_HOME_TOWARD_POSITIVE, fastHz);

    return CMD_OK_STARTED;
}

cmd_status_t HomingX_MoveDualUntilPin_Async(GPIO_TypeDef *port,
                                           uint16_t pin,
                                           bool active_low,
                                           int speed_k,
                                           uint8_t async_slot)
{
    if (!port || pin == 0) return CMD_ERR_BADARGS;
    if (speed_k <= 0) return CMD_ERR_BADARGS;

    if (g_hx.active) return CMD_ERR_BUSY;

    if (Motor_IsBusy(&motorX) || Motor_IsBusy(&motorX2) || Motor_IsBusy(&motorY))
        return CMD_ERR_BUSY;

    if (!XAxis_SetMode(XMODE_INDEPENDENT)) return CMD_ERR_BUSY;

    float speedHz = (float)(speed_k * SPEED_SCALE);
    speedHz = Clamp_XAxis_SpeedHz(speedHz);

    memset(&g_hx, 0, sizeof(g_hx));
    g_hx.active = 1;
    g_hx.mode   = HX_DUAL_UNTIL_PIN;
    g_hx.slot   = async_slot;

    g_hx.start_ms = HAL_GetTick();
    g_hx.next_sample_ms = g_hx.start_ms;

    g_hx.left_running  = 1;
    g_hx.right_running = 1;

    g_hx.shared_port       = port;
    g_hx.shared_pin        = pin;
    g_hx.shared_active_low = active_low;
    g_hx.shared_stable     = 0;

    // Per your requirement: move both X axes in the *positive* direction
    const bool toward_positive = true;

    Motor_RunConstant_Public(&motorX,  toward_positive, speedHz);
    Motor_RunConstant_Public(&motorX2, toward_positive, speedHz);

    return CMD_OK_STARTED;
}


/* ------------------------------------------------------------
 * Backoff:
 * IMPORTANT: with X2 on TIM23, you cannot run two independent ARR profiles.
 * Backoff is a "locked move" anyway — do it with the locked Gantry move (dx only).
 * ------------------------------------------------------------ */
static void start_dual_backoff(void)
{
    Motor_StopNow(&motorX);
    Motor_StopNow(&motorX2);

    uint32_t steps = HX_BACKOFF_STEPS;
    if (steps == 0) {
        g_hx.mode = HX_HOME_DUAL_SLOW_SEEK;
        return;
    }

    // Backoff direction is AWAY from limits
    int32_t dx = (HX_BACKOFF_TOWARD_POSITIVE) ? (int32_t)steps : -(int32_t)steps;

    // Temporarily lock axis for a perfectly matched backoff distance.
    XAxis_SetMode(XMODE_LOCKED);

    cmd_status_t st = Gantry_Goto_Parabolic_Async(dx, 0,
                                                 HX_BACKOFF_VMIN_HZ,
                                                 HX_BACKOFF_VPEAK_HZ,
                                                 ASYNC_SLOT_NONE /*slot not used here*/);
    if (st != CMD_OK_STARTED) {
        // If it failed, fall back to stopping + fail
        XAxis_SetMode(XMODE_INDEPENDENT);
        g_hx.active = 0;
        return;
    }

    g_hx.left_running  = 1;
    g_hx.right_running = 1;
    g_hx.mode = HX_HOME_DUAL_BACKOFF;

    // reset counters for next phases
    g_hx.left_stable  = 0;
    g_hx.right_stable = 0;
}

static void start_dual_slow_seek(void)
{
    float slowHz = Clamp_XAxis_SpeedHz(HX_SLOW_SEEK_SPEED_HZ);

    // Return to independent mode for per-motor stop on limit
    XAxis_SetMode(XMODE_INDEPENDENT);

    g_hx.left_running  = 1;
    g_hx.right_running = 1;
    g_hx.left_stable = 0;
    g_hx.right_stable = 0;

    g_hx.mode = HX_HOME_DUAL_SLOW_SEEK;

    Motor_RunConstant_Public(&motorX,  HX_HOME_TOWARD_POSITIVE, slowHz);
    Motor_RunConstant_Public(&motorX2, HX_HOME_TOWARD_POSITIVE, slowHz);
}

void HomingX_Pump(void)
{
    if (!g_hx.active) return;

    if (g_hx.mode == HX_MOVE_STEPS) {
        if (g_hx.left_running && !Motor_IsBusy(&motorX2)) g_hx.left_running = 0;
        if (g_hx.right_running && !Motor_IsBusy(&motorX)) g_hx.right_running = 0;

        if (!g_hx.left_running && !g_hx.right_running) {
            XAxis_SetMode(XMODE_LOCKED);
            finish(ASYNC_RES_OK);
        }
        return;
    }

    if (HX_TIMEOUT_MS > 0) {
        uint32_t elapsed = HAL_GetTick() - g_hx.start_ms;
        if (elapsed >= HX_TIMEOUT_MS) {
            Motor_StopNow(&motorX);
            Motor_StopNow(&motorX2);
            XAxis_SetMode(XMODE_LOCKED);
            finish(ASYNC_RES_STALLED);
            return;
        }
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_hx.next_sample_ms) < 0) return;
    g_hx.next_sample_ms = now + HX_SAMPLE_MS;

    /* ---------------- FAST SEEK (now debounced + debug) ---------------- */
    if (g_hx.mode == HX_HOME_DUAL_FAST_SEEK) {

        // LEFT
        if (g_hx.left_running) {
            bool pressed = pin_active(GANTRY_X2_LIMIT_GPIO_Port,
                                      GANTRY_X2_LIMIT_Pin,
                                      HX_LIMIT_ACTIVE_LOW);

            if (pressed == HX_WANT_PRESSED) {
                if (g_hx.left_stable < HX_FAST_DEBOUNCE_SAMPLES) g_hx.left_stable++;
                if (g_hx.left_stable == HX_FAST_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[FAST_SEEK][LEFT]", HX_FAST_DEBOUNCE_SAMPLES);
                    start_dual_backoff();
                    return;
                }
            } else {
                hx_debounce_fail("[FAST_SEEK][LEFT]", g_hx.left_stable, HX_FAST_DEBOUNCE_SAMPLES);
                g_hx.left_stable = 0;
            }
        }

        // RIGHT
        if (g_hx.right_running) {
            bool pressed = pin_active(GANTRY_X_LIMIT_GPIO_Port,
                                      GANTRY_X_LIMIT_Pin,
                                      HX_LIMIT_ACTIVE_LOW);

            if (pressed == HX_WANT_PRESSED) {
                if (g_hx.right_stable < HX_FAST_DEBOUNCE_SAMPLES) g_hx.right_stable++;
                if (g_hx.right_stable == HX_FAST_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[FAST_SEEK][RIGHT]", HX_FAST_DEBOUNCE_SAMPLES);
                    start_dual_backoff();
                    return;
                }
            } else {
                hx_debounce_fail("[FAST_SEEK][RIGHT]", g_hx.right_stable, HX_FAST_DEBOUNCE_SAMPLES);
                g_hx.right_stable = 0;
            }
        }

        return;
    }

    if (g_hx.mode == HX_HOME_DUAL_BACKOFF) {
        // Wait for locked backoff move to finish (both X motors become idle)
        if (g_hx.left_running && !Motor_IsBusy(&motorX2)) g_hx.left_running = 0;
        if (g_hx.right_running && !Motor_IsBusy(&motorX)) g_hx.right_running = 0;

        if (!g_hx.left_running && !g_hx.right_running) {
            start_dual_slow_seek();
        }
        return;
    }

    /* ---------------- SLOW SEEK (debug on streak breaks + pass) ---------------- */
    if (g_hx.mode == HX_HOME_DUAL_SLOW_SEEK) {

        if (g_hx.left_running) {
            bool pressed = pin_active(GANTRY_X2_LIMIT_GPIO_Port, GANTRY_X2_LIMIT_Pin, HX_LIMIT_ACTIVE_LOW);
            if (pressed == HX_WANT_PRESSED) {
                if (++g_hx.left_stable >= HX_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[SLOW_SEEK][LEFT]", HX_DEBOUNCE_SAMPLES);
                    Motor_StopNow(&motorX2);
                    g_hx.left_running = 0;
                }
            } else {
                hx_debounce_fail("[SLOW_SEEK][LEFT]", g_hx.left_stable, HX_DEBOUNCE_SAMPLES);
                g_hx.left_stable = 0;
            }
        }

        if (g_hx.right_running) {
            bool pressed = pin_active(GANTRY_X_LIMIT_GPIO_Port, GANTRY_X_LIMIT_Pin, HX_LIMIT_ACTIVE_LOW);
            if (pressed == HX_WANT_PRESSED) {
                if (++g_hx.right_stable >= HX_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[SLOW_SEEK][RIGHT]", HX_DEBOUNCE_SAMPLES);
                    Motor_StopNow(&motorX);
                    g_hx.right_running = 0;
                }
            } else {
                hx_debounce_fail("[SLOW_SEEK][RIGHT]", g_hx.right_stable, HX_DEBOUNCE_SAMPLES);
                g_hx.right_stable = 0;
            }
        }

        if (!g_hx.left_running && !g_hx.right_running) {
            XAxis_SetMode(XMODE_LOCKED);
            finish(ASYNC_RES_OK);
        }
        return;
    }

    /* ---------------- UNTIL_LIMIT_ONE (debug on streak breaks + pass) ---------------- */
    if (g_hx.mode == HX_UNTIL_LIMIT_ONE) {

        if (g_hx.left_running) {
            bool pressed = pin_active(GANTRY_X2_LIMIT_GPIO_Port, GANTRY_X2_LIMIT_Pin, HX_LIMIT_ACTIVE_LOW);
            if (pressed == HX_WANT_PRESSED) {
                if (++g_hx.left_stable >= HX_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[UNTIL_LIMIT_ONE][LEFT]", HX_DEBOUNCE_SAMPLES);
                    Motor_StopNow(&motorX2);
                    g_hx.left_running = 0;
                }
            } else {
                hx_debounce_fail("[UNTIL_LIMIT_ONE][LEFT]", g_hx.left_stable, HX_DEBOUNCE_SAMPLES);
                g_hx.left_stable = 0;
            }
        }

        if (g_hx.right_running) {
            bool pressed = pin_active(GANTRY_X_LIMIT_GPIO_Port, GANTRY_X_LIMIT_Pin, HX_LIMIT_ACTIVE_LOW);
            if (pressed == HX_WANT_PRESSED) {
                if (++g_hx.right_stable >= HX_DEBOUNCE_SAMPLES) {
                    hx_debounce_pass("[UNTIL_LIMIT_ONE][RIGHT]", HX_DEBOUNCE_SAMPLES);
                    Motor_StopNow(&motorX);
                    g_hx.right_running = 0;
                }
            } else {
                hx_debounce_fail("[UNTIL_LIMIT_ONE][RIGHT]", g_hx.right_stable, HX_DEBOUNCE_SAMPLES);
                g_hx.right_stable = 0;
            }
        }

        if (!g_hx.left_running && !g_hx.right_running) {
            XAxis_SetMode(XMODE_LOCKED);
            finish(ASYNC_RES_OK);
        }
        return;
    }

    /* ---------------- DUAL_UNTIL_PIN (separate debounce + debug) ---------------- */
    if (g_hx.mode == HX_DUAL_UNTIL_PIN) {

        bool pressed = pin_active(g_hx.shared_port,
                                  g_hx.shared_pin,
                                  g_hx.shared_active_low);

        if (pressed == HX_WANT_PRESSED) {
            if (g_hx.shared_stable < HX_UNTIL_PIN_DEBOUNCE_SAMPLES) g_hx.shared_stable++;
            if (g_hx.shared_stable == HX_UNTIL_PIN_DEBOUNCE_SAMPLES) {
                hx_debounce_pass("[DUAL_UNTIL_PIN]", HX_UNTIL_PIN_DEBOUNCE_SAMPLES);

                Motor_StopNow(&motorX);
                Motor_StopNow(&motorX2);

                g_hx.left_running  = 0;
                g_hx.right_running = 0;

                XAxis_SetMode(XMODE_LOCKED);
                finish(ASYNC_RES_OK);
            }
        } else {
            hx_debounce_fail("[DUAL_UNTIL_PIN]", g_hx.shared_stable, HX_UNTIL_PIN_DEBOUNCE_SAMPLES);
            g_hx.shared_stable = 0;
        }
        return;
    }
}
