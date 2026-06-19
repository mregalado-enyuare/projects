#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "main.h"    // GPIO_TypeDef, HAL_* (also exposes extern handles like hiwdg1)
#include "motors.h"  // MotorControl_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    MotorControl_t *motor;            // axis motor handle (your motors[0]/[1])
    GPIO_TypeDef   *limit_port;
    uint16_t        limit_pin;
    bool            limit_active_low; // true if pressed = RESET (pull-up wiring)
    bool            dir_toward_switch;// arg passed to Motor_SetDirection()
    // Motion-layer adapters (you wire these up per-axis in commands.c):
    void (*pull_off)(int32_t steps, float speed_k); // move away from switch by +steps
    void (*zero_axis)(void);         // zero your software position for this axis
} AxisHomeConfig;

typedef struct {
    float     fast_k;
    float     slow_k;
    int32_t   pull_off_steps;
    uint32_t  debounce_samples;
    uint32_t  debounce_ms;
} HomeTuning;

/**
 * Homes a single axis: (release if pre-pressed) -> fast seek -> pull-off -> slow seek.
 * Returns true on success, false on timeout or switch never seen.
 */
bool HomeAxis(const AxisHomeConfig *cfg, const HomeTuning *tune);

#ifdef __cplusplus
}
#endif
