#include "homing.h"
#include <math.h>

// --------- local helpers (not exported) ---------
static inline bool pin_active(GPIO_TypeDef *port, uint16_t pin, bool active_low) {
    GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
    return active_low ? (s == GPIO_PIN_RESET) : (s == GPIO_PIN_SET);
}

static uint32_t timer_clock_apb1(void) {
    RCC_ClkInitTypeDef clkcfg;
    uint32_t flashLatency;
    HAL_RCC_GetClockConfig(&clkcfg, &flashLatency);

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return (clkcfg.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2U);
}

// Convert k-steps/s -> ARR for the axis timers you use on APB1.
// If you move an axis to an APB2 timer, mirror this with an APB2 variant.
static uint32_t ksteps_to_arr(float ksteps) {
    if (ksteps <= 0.0f) return 0xFFFFFFFFu;
    const double step_hz = (double)ksteps * 1000.0;
    const double tclk    = (double)timer_clock_apb1();
    double arrd = (tclk / step_hz) - 1.0;
    if (arrd < 1.0) arrd = 1.0;
    if (arrd > 4294967295.0) arrd = 4294967295.0;
    return (uint32_t)(arrd + 0.5);
}

// Start constant-speed PWM WITHOUT enabling base timer interrupts.
static void Motor_StartConstant(MotorControl_t *m, bool dir, float speed_k) {
    uint32_t arr = ksteps_to_arr(speed_k);
//    Motor_SetDirection(m, dir);
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->timChannel, arr / 2U);
    HAL_TIM_PWM_Start(m->htim, m->timChannel);   // no Base_Start_IT
    m->state = dir ? MOTOR_STATE_MOVING_UP : MOTOR_STATE_MOVING_DOWN;
}

static void Motor_StopConstant(MotorControl_t *m) {
    HAL_TIM_PWM_Stop(m->htim, m->timChannel);
    m->state = MOTOR_STATE_IDLE;
}


static bool WaitForLimitDebounced(GPIO_TypeDef *port, uint16_t pin, bool active_low,
                                  bool want_pressed,
                                  uint32_t needed_samples, uint32_t sample_ms)
{
    uint32_t stable = 0;
    for (;;) {
        bool pressed = pin_active(port, pin, active_low);
        stable = (pressed == want_pressed) ? (stable + 1) : 0;
        if (stable >= needed_samples) return true;
        // Homing_RefreshWatchdog();
        HAL_Delay(sample_ms);
    }
}

static bool MoveUntilLimit(MotorControl_t *m, bool dir, float speed_k,
                           GPIO_TypeDef *port, uint16_t pin, bool active_low,
                           bool want_pressed,
                           uint32_t needed_samples, uint32_t sample_ms)
{
    Motor_StartConstant(m, dir, speed_k);
    bool ok = WaitForLimitDebounced(port, pin, active_low, want_pressed,
                                    needed_samples, sample_ms);
    Motor_StopConstant(m);
    return ok;
}

// --------- exported API ---------
bool HomeAxis(const AxisHomeConfig *cfg, const HomeTuning *tune) {
    if (!cfg || !cfg->motor || !cfg->pull_off || !cfg->zero_axis || !tune) return false;

    // 0) If already pressed, just back off by a fixed number of steps (no wait-until-release).
        if (HAL_GPIO_ReadPin(cfg->limit_port, cfg->limit_pin) ==
            (cfg->limit_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET)) {

            // Pull off away from the switch using your motion layer (fixed distance).
            // Use a gentle speed (tune->slow_k) so you don't overshoot mechanically.
            int32_t steps = (tune->pull_off_steps > 0) ? tune->pull_off_steps : 200;
            cfg->pull_off(steps, tune->slow_k);

            // (Optional but harmless) short settle delay for mechanics/electrical
            HAL_Delay(tune->debounce_ms * tune->debounce_samples);
        }

    // 1) FAST approach TOWARD the switch until it trips (debounced).
    if (!MoveUntilLimit(cfg->motor, /*dir=*/cfg->dir_toward_switch, tune->fast_k,
                        cfg->limit_port, cfg->limit_pin, cfg->limit_active_low,
                        /*want_pressed=*/true,
                        tune->debounce_samples, tune->debounce_ms
                        /* no timeout */)) {
        return false;
    }

    // 2) Ensure RELEASE before precision pass: move away UNTIL it actually releases (debounced).
    //    This avoids the “backs off a bit but still pressed” edge case on sticky switches.
    if (!MoveUntilLimit(cfg->motor, /*dir=*/!cfg->dir_toward_switch, /*use a gentle speed*/ 5.0f,
                        cfg->limit_port, cfg->limit_pin, cfg->limit_active_low,
                        /*want_pressed=*/false,
                        tune->debounce_samples, tune->debounce_ms
                        /* no timeout */)) {
        return false;
    }

    // Optional: add a tiny extra pull-off by steps to guarantee mechanical clearance.
    // (This uses your motion layer so both axes behave consistently.)
    cfg->pull_off((tune->pull_off_steps > 0) ? tune->pull_off_steps : 200, 1.0f);

    // 3) SLOW precision approach TOWARD the switch until it trips again (debounced).
    if (!MoveUntilLimit(cfg->motor, /*dir=*/cfg->dir_toward_switch, tune->slow_k,
                        cfg->limit_port, cfg->limit_pin, cfg->limit_active_low,
                        /*want_pressed=*/true,
                        tune->debounce_samples, tune->debounce_ms
                        /* no timeout */)) {
        return false;
    }

    // 4) Zero software position and stop motor.
    cfg->zero_axis();
    Motor_StopConstant(cfg->motor);
    cfg->motor->stepCount = 0;
    cfg->motor->state     = MOTOR_STATE_IDLE;
    return true;
}

