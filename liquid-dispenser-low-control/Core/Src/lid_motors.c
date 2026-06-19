#include "lid_motors.h"
#include "main.h"
#include "async_bus.h"
#include "dm556_parabolic_motion.h"
#include "lwip.h"
#include <stddef.h>

extern TIM_HandleTypeDef htim2;
extern IWDG_HandleTypeDef hiwdg1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim24;

/* ========================= SCRATCH PROFILE BUFFERS ========================= */

static uint32_t lid_translate1_profile[LID_MOTOR_MAX_STEPS];
static uint32_t lid_translate2_profile[LID_MOTOR_MAX_STEPS];
static uint32_t lid_rotate1_profile[LID_MOTOR_MAX_STEPS];
static uint32_t lid_rotate2_profile[LID_MOTOR_MAX_STEPS];
static uint32_t lid_seal_profile[LID_MOTOR_MAX_STEPS];

/* ========================= MOTOR INSTANCES ========================= */

ParabolicMotorControl_t motorLidTranslate1 = {
  .htim       = NULL,  // set in lid_motors_init
  .timChannel = TIM_CHANNEL_1,
  .dirPort    = LID_TRANSLATE_1_DIR_GPIO_Port,
  .dirPin     = LID_TRANSLATE_1_DIR_Pin,
  .state      = MOTOR_IDLE,
  .arrProfile = NULL,
  .stepsPlanned = 0,
  .stepIndex  = 0,
};

ParabolicMotorControl_t motorLidTranslate2 = {
  .htim       = NULL,
  .timChannel = TIM_CHANNEL_1,
  .dirPort    = LID_TRANSLATE_2_DIR_GPIO_Port,
  .dirPin     = LID_TRANSLATE_2_DIR_Pin,
  .state      = MOTOR_IDLE,
  .arrProfile = NULL,
  .stepsPlanned = 0,
  .stepIndex  = 0,
};

ParabolicMotorControl_t motorLidRotate1 = {
  .htim       = NULL,
  .timChannel = TIM_CHANNEL_1,
  .dirPort    = LID_ROTATE_1_DIR_GPIO_Port,
  .dirPin     = LID_ROTATE_1_DIR_Pin,
  .state      = MOTOR_IDLE,
  .arrProfile = NULL,
  .stepsPlanned = 0,
  .stepIndex  = 0,
};

ParabolicMotorControl_t motorLidRotate2 = {
  .htim       = NULL,
  .timChannel = TIM_CHANNEL_2,
  .dirPort    = LID_ROTATE_2_DIR_GPIO_Port,
  .dirPin     = LID_ROTATE_2_DIR_Pin,
  .state      = MOTOR_IDLE,
  .arrProfile = NULL,
  .stepsPlanned = 0,
  .stepIndex  = 0,
};

ParabolicMotorControl_t motorLidSeal = {
  .htim       = NULL,
  .timChannel = TIM_CHANNEL_1,
  .dirPort    = LID_SEAL_DIR_GPIO_Port,
  .dirPin     = LID_SEAL_DIR_Pin,
  .state      = MOTOR_IDLE,
  .arrProfile = NULL,
  .stepsPlanned = 0,
  .stepIndex  = 0,
};

/* ========================= ASYNC SLOT TRACKING ========================= */

typedef struct { uint8_t slot; volatile uint8_t active; } LidSlot_t;

static LidSlot_t g_slot_translate1;
static LidSlot_t g_slot_translate2;
static LidSlot_t g_slot_rotate1;
static LidSlot_t g_slot_rotate2;
static LidSlot_t g_slot_seal;

static void on_translate1_done(void *ctx) {
  (void)ctx;
  if (g_slot_translate1.active) {
    g_slot_translate1.active = 0;
    async_evt_push_isr(g_slot_translate1.slot, ASYNC_RES_OK);
  }
}

static void on_translate2_done(void *ctx) {
  (void)ctx;
  if (g_slot_translate2.active) {
    g_slot_translate2.active = 0;
    async_evt_push_isr(g_slot_translate2.slot, ASYNC_RES_OK);
  }
}

static void on_rotate1_done(void *ctx) {
  (void)ctx;
  if (g_slot_rotate1.active) {
    g_slot_rotate1.active = 0;
    async_evt_push_isr(g_slot_rotate1.slot, ASYNC_RES_OK);
  }
}

static void on_rotate2_done(void *ctx) {
  (void)ctx;
  if (g_slot_rotate2.active) {
    g_slot_rotate2.active = 0;
    async_evt_push_isr(g_slot_rotate2.slot, ASYNC_RES_OK);
  }
}

static void on_seal_done(void *ctx) {
  (void)ctx;
  if (g_slot_seal.active) {
    g_slot_seal.active = 0;
    async_evt_push_isr(g_slot_seal.slot, ASYNC_RES_OK);
  }
}

/* ========================= INIT ========================= */

void lid_motors_init(void)
{
  motorLidTranslate1.htim = &htim3;
  motorLidTranslate2.htim = &htim4;
  motorLidRotate1.htim    = &htim24;
  motorLidRotate2.htim    = &htim24;
  motorLidSeal.htim       = &htim2;

  Motor_RegisterExtra(&motorLidTranslate1, on_translate1_done, NULL);
  Motor_RegisterExtra(&motorLidTranslate2, on_translate2_done, NULL);
  Motor_RegisterExtra(&motorLidRotate1,   on_rotate1_done,    NULL);
  Motor_RegisterExtra(&motorLidRotate2,   on_rotate2_done,    NULL);
  Motor_RegisterExtra(&motorLidSeal,      on_seal_done,       NULL);

  /* Limit switch pins are NO (normally open), driven HIGH by switch at limit.
   * Pull-down keeps pin LOW when switch is open (not at limit). */
  GPIO_InitTypeDef cfg = {0};
  cfg.Mode = GPIO_MODE_INPUT;
  cfg.Pull = GPIO_PULLDOWN;

  cfg.Pin = LID_TRANSLATE_1_LIMIT_Pin;
  HAL_GPIO_Init(LID_TRANSLATE_1_LIMIT_GPIO_Port, &cfg);

  cfg.Pin = LID_TRANSLATE_2_LIMIT_Pin;
  HAL_GPIO_Init(LID_TRANSLATE_2_LIMIT_GPIO_Port, &cfg);

  cfg.Pin = LID_SEAL_LIMIT_Pin;
  HAL_GPIO_Init(LID_SEAL_LIMIT_GPIO_Port, &cfg);
}

/* ========================= ASYNC MOVE HELPERS ========================= */

static cmd_status_t start_move(ParabolicMotorControl_t *m,
                               uint32_t *profile_buf,
                               LidSlot_t *slot_state,
                               int32_t steps,
                               uint32_t speed_hz, uint32_t accel_hz,
                               uint8_t async_slot)
{
  if (Motor_IsBusy(m)) return CMD_ERR_BUSY;
  if (steps == 0)      return CMD_OK_IMMEDIATE;

  bool dir_pos = (steps > 0);
  uint32_t steps_abs = (uint32_t)(steps < 0 ? -steps : steps);
  if (steps_abs > LID_MOTOR_MAX_STEPS) return CMD_ERR_BADARGS;

  slot_state->slot   = async_slot;
  slot_state->active = 1;

  if (!Motor_MoveParabolic(m, steps_abs, dir_pos,
                           MIN_SPEED, (float)speed_hz,
                           profile_buf, LID_MOTOR_MAX_STEPS)) {
    slot_state->active = 0;
    return CMD_ERR_BUSY;
  }

  return CMD_OK_STARTED;
}

cmd_status_t LidTranslate_StartMove(int motor_id, int32_t steps,
                                    uint32_t speed_hz, uint32_t accel_hz,
                                    uint8_t async_slot)
{
  switch (motor_id) {
    case 1:
      return start_move(&motorLidTranslate1, lid_translate1_profile,
                        &g_slot_translate1, steps, speed_hz, accel_hz, async_slot);
    case 2:
      return start_move(&motorLidTranslate2, lid_translate2_profile,
                        &g_slot_translate2, steps, speed_hz, accel_hz, async_slot);
    default:
      return CMD_ERR_BADARGS;
  }
}

cmd_status_t LidRotate_StartMove(int motor_id, int32_t steps,
                                 uint32_t speed_hz, uint32_t accel_hz,
                                 uint8_t async_slot)
{
  // TIM24 is shared by both rotate motors; only one can run at a time
  if (Motor_IsBusy(&motorLidRotate1) || Motor_IsBusy(&motorLidRotate2))
    return CMD_ERR_BUSY;

  switch (motor_id) {
    case 1:
      return start_move(&motorLidRotate1, lid_rotate1_profile,
                        &g_slot_rotate1, steps, speed_hz, accel_hz, async_slot);
    case 2:
      return start_move(&motorLidRotate2, lid_rotate2_profile,
                        &g_slot_rotate2, steps, speed_hz, accel_hz, async_slot);
    default:
      return CMD_ERR_BADARGS;
  }
}

cmd_status_t LidSeal_StartMove(int32_t steps, uint32_t speed_hz,
                               uint32_t accel_hz, uint8_t async_slot)
{
  return start_move(&motorLidSeal, lid_seal_profile,
                    &g_slot_seal, steps, speed_hz, accel_hz, async_slot);
}

/* ========================= ASYNC CALIBRATE STATE MACHINE ========================= */

#define LID_CAL_DEBOUNCE  7   /* consecutive HIGH reads before declaring home */

typedef enum {
    LCAL_IDLE = 0,
    LCAL_SEEK,
} LidCalPhase;

typedef struct {
    LidCalPhase              phase;
    ParabolicMotorControl_t *motor;
    GPIO_TypeDef            *limit_port;
    uint16_t                 limit_pin;
    uint32_t                 stable;
    uint8_t                  async_slot;
} LidCalState;

static LidCalState g_lid_cal;

static cmd_status_t lidcal_start(ParabolicMotorControl_t *m,
                                 GPIO_TypeDef *port, uint16_t pin,
                                 float speed_hz, bool toward_positive,
                                 uint8_t async_slot)
{
    if (speed_hz <= 0.0f)             return CMD_ERR_BADARGS;
    if (g_lid_cal.phase != LCAL_IDLE) return CMD_ERR_BUSY;
    if (Motor_IsBusy(m))              return CMD_ERR_BUSY;

    g_lid_cal.motor      = m;
    g_lid_cal.limit_port = port;
    g_lid_cal.limit_pin  = pin;
    g_lid_cal.async_slot = async_slot;
    g_lid_cal.stable     = 0;
    g_lid_cal.phase      = LCAL_SEEK;

    Motor_RunConstant_Public(m, toward_positive, speed_hz);
    return CMD_OK_STARTED;
}

cmd_status_t LidTranslate_StartCalibrate(int motor_id, uint32_t speed_hz, uint8_t async_slot)
{
    switch (motor_id) {
        case 1: return lidcal_start(&motorLidTranslate1,
                                    LID_TRANSLATE_1_LIMIT_GPIO_Port, LID_TRANSLATE_1_LIMIT_Pin,
                                    (float)speed_hz, false, async_slot);
        case 2: return lidcal_start(&motorLidTranslate2,
                                    LID_TRANSLATE_2_LIMIT_GPIO_Port, LID_TRANSLATE_2_LIMIT_Pin,
                                    (float)speed_hz, false, async_slot);
        default: return CMD_ERR_BADARGS;
    }
}

cmd_status_t LidSeal_StartCalibrate(uint32_t speed_hz, uint8_t async_slot)
{
    return lidcal_start(&motorLidSeal,
                        LID_SEAL_LIMIT_GPIO_Port, LID_SEAL_LIMIT_Pin,
                        (float)speed_hz, true, async_slot);
}

void LidCal_Pump(void)
{
    if (g_lid_cal.phase == LCAL_IDLE) return;

    bool high = (HAL_GPIO_ReadPin(g_lid_cal.limit_port, g_lid_cal.limit_pin) == GPIO_PIN_SET);

    switch (g_lid_cal.phase) {

    case LCAL_SEEK:
        g_lid_cal.stable = high ? (g_lid_cal.stable + 1) : 0;
        if (g_lid_cal.stable >= LID_CAL_DEBOUNCE) {
            Motor_StopNow(g_lid_cal.motor);
            g_lid_cal.phase = LCAL_IDLE;
            (void)async_evt_push_isr(g_lid_cal.async_slot, ASYNC_RES_OK);
        }
        break;

    default:
        break;
    }
}
