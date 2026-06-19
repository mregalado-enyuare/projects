/*
 * dm556_parabolic_motion.h
 *
 *  Created on: Sep 22, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_DM556_PARABOLIC_MOTION_H_
#define INC_DM556_PARABOLIC_MOTION_H_

#include "main.h"
#include "async_bus.h"
#include "commands.h"
#include <stdbool.h>
#include <stdint.h>

// min speed used for moves (auto select vmax based on this and acceleration)
#define MIN_SPEED 4000.0f

// steps and speed scales that anya wants
#define STEPS_SCALE  10
#define SPEED_SCALE  1000

// max segments in a GOTO sequence
#define GOTO_SEQ_MAX  32

// Motor state machine
typedef enum {
  MOTOR_IDLE = 0,
  MOTOR_MOVING
} ParabolicMotorState_t;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t           timChannel;
  GPIO_TypeDef      *dirPort;
  uint16_t           dirPin;
  volatile ParabolicMotorState_t state;
  uint32_t          *arrProfile;
  uint32_t           stepsPlanned;
  volatile uint32_t  stepIndex;
} ParabolicMotorControl_t;

typedef struct {
  volatile uint8_t  active;     // 0/1: a Gantry_Goto_Parabolic move is in flight
  volatile uint8_t  slot;       // async slot to signal on completion
} gantry_async_t;

typedef struct {
  volatile uint8_t active;     // sequence running
  volatile uint8_t kick;       // "start next segment" requested (set in ISR)
  uint8_t          slot;       // user's async slot for the overall GOTO
  float            v_min;      // MIN_SPEED
  int              speed_k;    // from JSON ("speed")
  int              accel_k;    // from JSON ("accel")
  int              count;      // number of segments
  int              idx;        // next segment to launch
  int32_t          dx[GOTO_SEQ_MAX];
  int32_t          dy[GOTO_SEQ_MAX];
} goto_seq_t;

// Expose X axis mode type for independent driving
typedef enum {
  XMODE_LOCKED = 0,
  XMODE_INDEPENDENT
} XAxisMode_t;

// Allow other modules to switch modes
bool XAxis_SetMode(XAxisMode_t mode);

// Export max profile steps as a macro usable for array sizing
#define DM556_MAX_PROFILE_STEPS   (31000U)

// Capacity constant visible to other files
extern const uint32_t DM556_MAX_STEPS_PER_MOVE;

// Expose motors for tests in main
extern ParabolicMotorControl_t motorX;
extern ParabolicMotorControl_t motorY;

// Scratch buffer for motor profiles
extern uint32_t x_profile[];
extern uint32_t y_profile[];

extern ParabolicMotorControl_t motorX2;

float max_speed_for_move(float speed_cap, float accel, int32_t dx, int32_t dy);

/* Public API */
bool Motor_MoveParabolic(ParabolicMotorControl_t *m, uint32_t stepsAbs, bool dirPos,
                         uint32_t f_min, uint32_t f_peak,
                         uint32_t *scratchProfileBuf, uint32_t scratchLen);

void Gantry_Goto_Parabolic(int32_t deltaX, int32_t deltaY,
                           float startSpeed, float endSpeed);

cmd_status_t Gantry_Goto_Parabolic_Async(int32_t deltaX, int32_t deltaY,
                                         float startSpeed, float endSpeed,
                                         uint8_t async_slot);

bool Motor_IsBusy(const ParabolicMotorControl_t* m);

// dm556_parabolic_motion.h  (add near the bottom, before #endif)

// Smooth continuous circle (no Gantry_Goto_Parabolic, no PWM start/stop per segment)
// - count: number of full circles (>=1)
// - radius: in your gantry units (before your ×10 scaling; same units you used for circles)
// - speedHz: total steps/second along the path (one step per timer tick overall)
// - clockwise: true = CW, false = CCW
void Circle_Start(uint32_t count, uint32_t radius, float speedHz, bool clockwise);
bool Circle_IsActive(void);
void Circle_Stop(void);

// dm556_parabolic_motion.h
cmd_status_t Circle_Start_Async(uint32_t count, uint32_t radius,
                                float speedHz, bool clockwise,
                                uint8_t async_slot);
void Circle_Abort_Async(void);

bool X_MoveUntil_Limit(bool toward_positive,
                        float speed_k,
                        GPIO_TypeDef *port, uint16_t pin, bool active_low,
                        bool want_pressed,
                        uint32_t samples, uint32_t sample_ms);

bool Y_MoveUntil_Limit(bool toward_positive,
                        float speed_k,
                        GPIO_TypeDef *port, uint16_t pin, bool active_low,
                        bool want_pressed,
                        uint32_t samples, uint32_t sample_ms);

void GotoSeq_Pump(void);

void Motor_ServiceIRQ(TIM_HandleTypeDef *htim);

extern gantry_async_t g_gantry;
extern goto_seq_t g_goto_seq;


// ---- Exposed helpers for other modules (homing_x.c) ----
uint32_t TimerInputClock_Hz(TIM_HandleTypeDef *htim);
uint32_t Freq_To_ARR(uint32_t timerHz, double freq);
void BuildParabolicARRProfile(uint32_t steps, uint32_t f_min, uint32_t f_peak,
                              uint32_t timerHz, uint32_t *outArr);

void Motor_StartMove(ParabolicMotorControl_t *m, uint32_t *arrBuf, uint32_t steps);

// Set motor direction in gantry "dirPos" terms.
// dirPos=true  => move toward +X in machine coordinates
// dirPos=false => move toward -X
void Motor_SetDir_Public(ParabolicMotorControl_t *m, bool dirPos);

// Run/stop constant-speed pulses (public wrapper)
void Motor_RunConstant_Public(ParabolicMotorControl_t *m,
                              bool toward_positive,
                              float steps_per_sec);

void Motor_StopNow(ParabolicMotorControl_t *m);

float Clamp_XAxis_SpeedHz(float requestedHz);

// --- Extra-motor registration (for modules outside dm556, e.g. lid_motors) ---
// Register a motor not in the built-in motors[] array so Motor_ServiceIRQ steps
// it through its ARR profile and calls on_done_isr(ctx) from the ISR on completion.
#define DM556_EXTRA_MOTORS_MAX 8
void Motor_RegisterExtra(ParabolicMotorControl_t *m,
                         void (*on_done_isr)(void *ctx),
                         void *ctx);

// Generic blocking "move until limit" for any parabolic motor.
// Mirrors Y_MoveUntil_Limit but accepts any motor pointer.
bool Motor_MoveUntilLimit(ParabolicMotorControl_t *m,
                          bool toward_positive, float speed_hz,
                          GPIO_TypeDef *port, uint16_t pin, bool active_low,
                          bool want_pressed,
                          uint32_t samples, uint32_t sample_ms);

#endif /* INC_DM556_PARABOLIC_MOTION_H_ */
