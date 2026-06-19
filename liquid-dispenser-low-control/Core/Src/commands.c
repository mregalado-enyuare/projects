#include "commands.h"
#include "main.h"

/* Pin-name compat: commands.c uses the old cold/hot cup naming; main.h was
 * updated to the new CUP_1/CUP_2 + H-bridge scheme without updating this
 * file. Map old names to the closest equivalent so the build succeeds.
 * DISPENSE_CUP will not function correctly with the new hardware until
 * Handle_DispenseCup_Async is rewritten for the H-bridge + PWM interface. */
#define CUP_COLD_FET_GPIO_Port   CUP_1_IN1_GPIO_Port
#define CUP_COLD_FET_Pin         CUP_1_IN1_Pin
#define CUP_COLD_SENSOR_GPIO_Port CUP_1_ECHO_GPIO_Port
#define CUP_COLD_SENSOR_Pin       CUP_1_ECHO_Pin
#define CUP_HOT_FET_GPIO_Port    CUP_2_IN1_GPIO_Port
#define CUP_HOT_FET_Pin          CUP_2_IN1_Pin
#define CUP_HOT_SENSOR_GPIO_Port  CUP_2_ECHO_GPIO_Port
#define CUP_HOT_SENSOR_Pin        CUP_2_ECHO_Pin
#include "motors.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "cJSON.h"
#include <stdarg.h>
#include "homing.h"
#include "lwip.h"
#include "dm556_parabolic_motion.h"
#include "lid_motors.h"
#include "async_bus.h"
#include "cup_dispense.h"
#include "keg_dispense.h"
#include "ice_dispenser.h"
#include "homing_x.h"


extern IWDG_HandleTypeDef hiwdg1;

extern gantry_async_t g_gantry;
extern goto_seq_t g_goto_seq;

typedef enum {
  CD_IDLE = 0,
  CD_X_HOMING,
} calib_dual_phase_t;

static struct {
  uint8_t active;
  uint8_t slot;
  calib_dual_phase_t phase;
} g_calib_dual = {0};

cmd_status_t Command_Execute_Async(const char *cmdLine, int async_slot);

// Unified handler signature: returns true on success
typedef bool (*CmdHandler_t)(cJSON *cmdLine);

typedef struct {
  const char *cmd_name;   // without '$' prefix and without trailing '!'
  CmdHandler_t handler;
} Command_t;
// JSON field extraction helpers

const char* get_string(cJSON *root, const char *key) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
  if (!cJSON_IsString(item) || item->valuestring == NULL) {
    fprintf(stderr, "Error: \"%s\" missing or not a string\n", key);
    return NULL;
  }
  return item->valuestring;
}
int get_int(cJSON *root, const char *key, int *out) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
  if (!cJSON_IsNumber(item)) {
    fprintf(stderr, "Error: \"%s\" missing or not a number\n", key);
    return -1;
  }
  *out = item->valueint;
  return 0;
}

static cmd_status_t Handle_LidTranslateMove_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_LidTranslateCal_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_LidRotateMove_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_LidSealMove_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_LidSealCal_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_Goto_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_DispenseCup_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_DispenseKeg_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_Circle_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_DispenseIce_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_CalibrateDual_Async(cJSON *root, int async_slot);

// Still blocking
static bool Handle_Calibrate(cJSON*);
static bool Handle_Dispense(cJSON*);
static bool Handle_LidSuctionOn(cJSON*);
static bool Handle_LidSuctionOff(cJSON*);

static cmd_status_t Handle_MoveXIndependent_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_MoveXIndependentUntilLimit_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_HomeXDualUntilLimit_Async(cJSON *root, int async_slot);
static cmd_status_t Handle_MoveUntilLidLimit_Async(cJSON *root, int async_slot);


// async command dispatcher
cmd_status_t Command_Execute_Async(const char *cmdLine, int async_slot) {
  cJSON *root = cJSON_Parse(cmdLine);
  if (!root) return CMD_ERR_BADARGS;

  cJSON *j_cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
  if (!cJSON_IsString(j_cmd) || j_cmd->valuestring == NULL) {
    cJSON_Delete(root);
    return CMD_ERR_BADARGS;
  }

  // Dispatch to async-capable handlers; each returns whether it started async or finished immediately.
  cmd_status_t st = CMD_ERR_BADARGS;

  if      (strcmp(j_cmd->valuestring, "LID_TRANSLATE_MOVE") == 0)      st = Handle_LidTranslateMove_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "LID_TRANSLATE_CALIBRATE") == 0) st = Handle_LidTranslateCal_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "LID_ROTATE_MOVE") == 0)         st = Handle_LidRotateMove_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "LID_SEAL_MOVE") == 0)           st = Handle_LidSealMove_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "LID_SEAL_CALIBRATE") == 0)      st = Handle_LidSealCal_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "GOTO") == 0)          st = Handle_Goto_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "DISPENSE_CUP") == 0)  st = Handle_DispenseCup_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "DISPENSE_KEG") == 0)  st = Handle_DispenseKeg_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "CIRCLE") == 0)        st = Handle_Circle_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "DISPENSE_ICE") == 0)  st = Handle_DispenseIce_Async(root, async_slot);
  else if (strcmp(j_cmd->valuestring, "CALIBRATE_DUAL") == 0) st = Handle_CalibrateDual_Async(root, async_slot);

  // Simple instantaneous commands can just act and return IMMEDIATE:
  else if (strcmp(j_cmd->valuestring, "CALIBRATE") == 0)  { (void)Handle_Calibrate(root); st = CMD_OK_IMMEDIATE; }
  else if (strcmp(j_cmd->valuestring, "DISPENSE") == 0)  { bool _ok = Handle_Dispense(root); st = _ok ? CMD_OK_IMMEDIATE : CMD_ERR_STALLED; }
  else if (strcmp(j_cmd->valuestring, "LID_SUCTION_ON") == 0)  { (void)Handle_LidSuctionOn(root);  st = CMD_OK_IMMEDIATE; }
  else if (strcmp(j_cmd->valuestring, "LID_SUCTION_OFF") == 0) { (void)Handle_LidSuctionOff(root); st = CMD_OK_IMMEDIATE; }

  else if (strcmp(j_cmd->valuestring, "MOVE_X_INDEPENDENT") == 0)
      st = Handle_MoveXIndependent_Async(root, async_slot);

  else if (strcmp(j_cmd->valuestring, "MOVE_X_INDEPENDENT_UNTIL_LIMIT") == 0)
      st = Handle_MoveXIndependentUntilLimit_Async(root, async_slot);

  else if (strcmp(j_cmd->valuestring, "HOME_X_DUAL_UNTIL_LIMIT") == 0)
      st = Handle_HomeXDualUntilLimit_Async(root, async_slot);

  else if (strcmp(j_cmd->valuestring, "MOVE_UNTIL_LID_LIMIT") == 0)
      st = Handle_MoveUntilLidLimit_Async(root, async_slot);

  cJSON_Delete(root);
  return st;
}

static cmd_status_t Handle_Goto_Async(cJSON *root, int async_slot)
{
  int speed_k_i = 0;
  int accel_k_i = 0;

  if (get_int(root, "speed", &speed_k_i) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "accel", &accel_k_i) < 0) return CMD_ERR_BADARGS;

  cJSON *j_coords = cJSON_GetObjectItemCaseSensitive(root, "coords");
  if (!cJSON_IsArray(j_coords)) return CMD_ERR_BADARGS;

  int n = cJSON_GetArraySize(j_coords);
  if (n <= 0) return CMD_OK_IMMEDIATE;

  if (n > GOTO_SEQ_MAX) n = GOTO_SEQ_MAX; // clamp to our small buffer

  if (g_goto_seq.active || g_gantry.active || Motor_IsBusy(&motorX) || Motor_IsBusy(&motorY)) {
    return CMD_ERR_BUSY;
  }

  // Fill the sequence buffer
  for (int i = 0; i < n; ++i) {
    cJSON *pair = cJSON_GetArrayItem(j_coords, i);
    if (!cJSON_IsArray(pair) || cJSON_GetArraySize(pair) != 2) return CMD_ERR_BADARGS;

    int dx = cJSON_GetArrayItem(pair, 0)->valueint;
    int dy = cJSON_GetArrayItem(pair, 1)->valueint;

    g_goto_seq.dx[i] = (int32_t)(dx * STEPS_SCALE);
    g_goto_seq.dy[i] = (int32_t)(dy * STEPS_SCALE);
  }

  g_goto_seq.slot    = (uint8_t)async_slot;   // one DONE for the whole list
  g_goto_seq.v_min   = MIN_SPEED;
  g_goto_seq.speed_k = speed_k_i;
  g_goto_seq.accel_k = accel_k_i;
  g_goto_seq.count   = n;
  g_goto_seq.idx     = 0;
  g_goto_seq.active  = 1;

  // Kick off the first segment from thread context immediately
  g_goto_seq.kick = 1;
  GotoSeq_Pump();
  HomingX_Pump();


  return CMD_OK_STARTED;
}

static cmd_status_t Handle_DispenseCup_Async(cJSON *root, int async_slot) {
  const char *type = get_string(root, "type");
  if (!type) return CMD_ERR_BADARGS;

  GPIO_TypeDef *fet_port = NULL, *sensor_port = NULL;
  uint16_t fet_pin = 0, sensor_pin = 0;
  bool sensor_active_low = true;

  if (strcmp(type, "cold") == 0) {
    fet_port    = CUP_COLD_FET_GPIO_Port;
    fet_pin     = CUP_COLD_FET_Pin;
    sensor_port = CUP_COLD_SENSOR_GPIO_Port;
    sensor_pin  = CUP_COLD_SENSOR_Pin;
  } else if (strcmp(type, "hot") == 0) {
    fet_port    = CUP_HOT_FET_GPIO_Port;
    fet_pin     = CUP_HOT_FET_Pin;
    sensor_port = CUP_HOT_SENSOR_GPIO_Port;
    sensor_pin  = CUP_HOT_SENSOR_Pin;
  } else {
    fprintf(stderr, "Error: \"type\" must be \"cold\" or \"hot\"\n");
    return CMD_ERR_BADARGS;
  }

  int debounce_samples = 8;
  int sample_ms        = 1;
  int timeout_ms       = 4000;

  // Start the non-blocking sequence
  bool started = fet_until_sensor_start(
      fet_port, fet_pin,
      sensor_port, sensor_pin,
      /*sensor_active_low=*/sensor_active_low,
      (uint16_t)debounce_samples,
      (uint32_t)sample_ms,
      (uint32_t)timeout_ms,
      (uint8_t)async_slot);

  if (!started) return CMD_ERR_BUSY;
  // ACK was already sent by udp layer; DONE will be posted from helper via async queue.
  return CMD_OK_STARTED;
}

static cmd_status_t Handle_DispenseKeg_Async(cJSON *root, int async_slot)
{
  if (!root) return CMD_ERR_BADARGS;

  int keg_id = -1;
  int duration_ms = 0;
  if (get_int(root, "keg_id", &keg_id) < 0)      return CMD_ERR_BADARGS;
  if (get_int(root, "duration_ms", &duration_ms) < 0) return CMD_ERR_BADARGS;
  if (duration_ms <= 0) return CMD_ERR_BADARGS;

  GPIO_TypeDef *fet_port = NULL;
  uint16_t fet_pin = 0;

  switch (keg_id) {
    case 1: fet_port = KEG1_FET_GPIO_Port; fet_pin = KEG1_FET_Pin; break;
    case 2: fet_port = KEG2_FET_GPIO_Port; fet_pin = KEG2_FET_Pin; break;
    case 3: fet_port = KEG3_FET_GPIO_Port; fet_pin = KEG3_FET_Pin; break;
    case 4: fet_port = KEG4_FET_GPIO_Port; fet_pin = KEG4_FET_Pin; break;
    case 5: fet_port = KEG5_FET_GPIO_Port; fet_pin = KEG5_FET_Pin; break;
    default: return CMD_ERR_BADARGS;
  }

  // Start non-blocking keg dispense
  bool started = keg_dispense_start(fet_port, fet_pin,
                                    (uint32_t)duration_ms,
                                    (uint8_t)async_slot);
  if (!started) return CMD_ERR_BUSY;

  // ACK already sent by UDP layer; DONE will be issued by keg_off_cb via async queue.
  return CMD_OK_STARTED;
}

static cmd_status_t Handle_Circle_Async(cJSON *root, int async_slot)
{
  int count=0, radius=0, speed_k=0;
  if (get_int(root, "count", &count)   < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "radius", &radius) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "speed",  &speed_k)< 0) return CMD_ERR_BADARGS;

  bool clockwise = false;
  cJSON *j_cw = cJSON_GetObjectItemCaseSensitive(root, "cw");
  if (cJSON_IsBool(j_cw)) clockwise = cJSON_IsTrue(j_cw);

  uint32_t r_steps = (uint32_t)(radius * STEPS_SCALE);
  float    speedHz = (float)(speed_k * SPEED_SCALE);

  return Circle_Start_Async((uint32_t)count, r_steps, speedHz, clockwise, (uint8_t)async_slot);
}

// --- Handler implementations -----------------------------------------------
static cmd_status_t Handle_CalibrateDual_Async(cJSON *root, int async_slot)
{
  (void)root;

  if (g_calib_dual.active) return CMD_ERR_BUSY;

  // Don’t start if motors busy / other sequences running (match your project’s style)
  if (g_goto_seq.active || g_gantry.active || Motor_IsBusy(&motorX) || Motor_IsBusy(&motorY)) {
    return CMD_ERR_BUSY;
  }

  // Start X dual home using the existing HomingX async state machine,
  // but we want *our* command to own the DONE. So: start X homing with ASYNC_SLOT_NONE.
  cmd_status_t st = HomingX_HomeDualUntilLimit_Async(/*speed_k ignored*/0, ASYNC_SLOT_NONE);
  if (st != CMD_OK_STARTED && st != CMD_OK_IMMEDIATE) return CMD_ERR_BUSY;

  g_calib_dual.active = 1;
  g_calib_dual.slot   = (uint8_t)async_slot;
  g_calib_dual.phase  = CD_X_HOMING;

  return CMD_OK_STARTED;
}

static bool Handle_Calibrate(cJSON *root)
{
  (void)root;

  // Speeds for approach and latch
  const float FAST_SEEK = 5000.0f;   // fast seek
  const float SLOW_SEEK = 2000.0f;   // precise latch

  // Fixed backoff distance in steps
  const int32_t BACKOFF_STEPS = 1000;

  // Debounce sampling periods
  const uint32_t MS = 1;
  const uint32_t SAMP_FAST_PRESS = 7;   // stop ASAP on first press
//  const uint32_t SAMP_RELEASE    = 100; // require 100 consecutive released to back off from limit switch
  const uint32_t SAMP_SLOW_PRESS = 12;  // precise final engage - higher samples

  // Limit switches are active high
  const bool X_ACTIVE_LOW = false;
  const bool Y_ACTIVE_LOW = false;

  // We want to move x in the negative direction with fast seek speed until we hit the limit switch
  // limit switch is active low, we want to stop immediately when pressed, sample at 1ms intervals
  if (!X_MoveUntil_Limit(false, FAST_SEEK, GANTRY_X_LIMIT_GPIO_Port, GANTRY_X_LIMIT_Pin,
                         X_ACTIVE_LOW, /*want_pressed*/true,
                         SAMP_FAST_PRESS, MS)) return false;

//  // We want to back off slowly from the limit switch until it is released for 100 samples
//  if (!X_MoveUntil_Limit(true, SLOW_SEEK, GANTRY_X_LIMIT_GPIO_Port, GANTRY_X_LIMIT_Pin,
//                         X_ACTIVE_LOW, /*want_pressed*/false,
//                         SAMP_RELEASE, MS)) return false;

  // Back off from the X limit by a fixed number of steps (no sensor checking)
  // Positive deltaX = move in the same direction as `toward_positive = true` in X_MoveUntil_Limit
  Gantry_Goto_Parabolic(+BACKOFF_STEPS, 0, SLOW_SEEK, SLOW_SEEK);

  // Now slow approach again until imit switch is hit again, with higher debounce count
  if (!X_MoveUntil_Limit(false, SLOW_SEEK, GANTRY_X_LIMIT_GPIO_Port, GANTRY_X_LIMIT_Pin,
                         X_ACTIVE_LOW, /*want_pressed*/true,
                         SAMP_SLOW_PRESS, MS)) return false;

  // We want to move y in the negative direction with fast seek speed until we hit the limit switch
  // limit switch is active low, we want to stop immediately when pressed, sample at 1ms intervals
  if (!Y_MoveUntil_Limit(false, FAST_SEEK, GANTRY_Y_LIMIT_GPIO_Port, GANTRY_Y_LIMIT_Pin,
                         Y_ACTIVE_LOW, /*want_pressed*/true,
                         SAMP_FAST_PRESS, MS)) return false;

//  // We want to back off slowly from the limit switch until it is released for 100 samples
//  if (!Y_MoveUntil_Limit(true, SLOW_SEEK, GANTRY_Y_LIMIT_GPIO_Port, GANTRY_Y_LIMIT_Pin,
//                         Y_ACTIVE_LOW, /*want_pressed*/false,
//                         SAMP_RELEASE, MS)) return false;

  // Back off from the Y limit by a fixed number of steps (no sensor checking)
  // Positive deltaY = move in the same direction as `toward_positive = true` in Y_MoveUntil_Limit
  Gantry_Goto_Parabolic(0, +BACKOFF_STEPS, SLOW_SEEK, SLOW_SEEK);

  // Now slow approach again until imit switch is hit again, with higher debounce count
  if (!Y_MoveUntil_Limit(false, SLOW_SEEK, GANTRY_Y_LIMIT_GPIO_Port, GANTRY_Y_LIMIT_Pin,
                         Y_ACTIVE_LOW, /*want_pressed*/true,
                         SAMP_SLOW_PRESS, MS)) return false;

  return true;
}

static bool Handle_Dispense(cJSON *root) {

  // Pull parameters from JSON
  if (!root) {
    fprintf(stderr, "JSON parse error: %s\n", cJSON_GetErrorPtr());
    return false;
  }

  int motor_id = -1;
  int steps = 0;
  int retract_steps = 0;
  int speed = 0;
  int retract_speed = 0;

  if (get_int(root, "motor_id", &motor_id) < 0) {
    return false;
  }

  if (get_int(root, "steps", &steps) < 0) {
    return false;
  }

  if (get_int(root, "retract_steps", &retract_steps) < 0) {
    return false;
  }

  if (get_int(root, "speed", &speed) < 0) {
    return false;
  }

  if (get_int(root, "retract_speed", &retract_speed) < 0) {
    return false;
  }

  // MUX wants a 0–15 channel; JSON gives 1–16
  uint8_t channel = (uint8_t) (motor_id - 1);

  HAL_GPIO_WritePin(SYRINGE_SEL0_GPIO_Port, SYRINGE_SEL0_Pin, (channel & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SYRINGE_SEL1_GPIO_Port, SYRINGE_SEL1_Pin, (channel & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SYRINGE_SEL2_GPIO_Port, SYRINGE_SEL2_Pin, (channel & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SYRINGE_SEL3_GPIO_Port, SYRINGE_SEL3_Pin, (channel & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  int rack_num = 0;
  get_int(root, "rack_num", &rack_num);
  syringe_clear_stall_msg();
  syringe_set_stall_context(rack_num, motor_id);
  UpdateEncoderPosition();
  bool fwd_ok = moveForward(steps, speed);
  if (fwd_ok && steps > 0 && retract_steps > 0) { // only delay if we moved AND are about to retract
	  HAL_Delay(1000);
  }
  bool bwd_ok = moveBackward(retract_steps, retract_speed);
  return fwd_ok && bwd_ok;
}

static bool Handle_LidSuctionOn(cJSON *root) {
  (void)root;
  HAL_GPIO_WritePin(LID_SUCTION_FET_GPIO_Port, LID_SUCTION_FET_Pin, GPIO_PIN_SET);
  return true;
}

static bool Handle_LidSuctionOff(cJSON *root) {
  (void)root;
  HAL_GPIO_WritePin(LID_SUCTION_FET_GPIO_Port, LID_SUCTION_FET_Pin, GPIO_PIN_RESET);
  return true;
}

static cmd_status_t Handle_DispenseIce_Async(cJSON *root, int async_slot)
{
  if (!root) return CMD_ERR_BADARGS;

  const char *type = get_string(root, "type");
  if (!type) {
    fprintf(stderr, "Error: \"type\" missing for DISPENSE_ICE\n");
    return CMD_ERR_BADARGS;
  }

  // Map JSON "type" -> ice_machine_t
  ice_machine_t id;
  if (strcmp(type, "cube") == 0) {
    id = ICE_MACHINE_CUBE;
  } else if (strcmp(type, "pebble") == 0) {
    id = ICE_MACHINE_PEBBLE;
  } else {
    fprintf(stderr, "Error: \"type\" must be \"cube\" or \"pebble\" for DISPENSE_ICE\n");
    return CMD_ERR_BADARGS;
  }

  // Optionally check busy before starting (start() also returns false if busy)
  if (ice_dispenser_busy(id)) {
    fprintf(stderr, "Error: ice machine %d already active\n", (int)id);
    return CMD_ERR_BUSY;
  }

  // Non-blocking start; EXTI + ice_dispenser.c will push async DONE later.
  if (!ice_dispenser_start(id, (uint8_t)async_slot)) {
    // Failed to start (busy or other internal reason)
    return CMD_ERR_BUSY;
  }

  return CMD_OK_STARTED;   // DONE will be published from EXTI via async_evt_push_isr
}
static cmd_status_t Handle_MoveXIndependent_Async(cJSON *root, int async_slot)
{
    int speed_k=0, accel_k=0;
    int steps_i=0;

    if (get_int(root, "speed", &speed_k) < 0) return CMD_ERR_BADARGS;
    if (get_int(root, "accel", &accel_k) < 0) return CMD_ERR_BADARGS;
    if (get_int(root, "steps", &steps_i) < 0) return CMD_ERR_BADARGS;

    const char *motor = get_string(root, "motor");
    if (!motor) return CMD_ERR_BADARGS;

    bool motor_left  = (strcmp(motor, "left") == 0);
    bool motor_right = (strcmp(motor, "right") == 0);
    if (!motor_left && !motor_right) return CMD_ERR_BADARGS;

    // IMPORTANT: keep signed
    int32_t steps_scaled = (int32_t)steps_i * (int32_t)STEPS_SCALE;

    return HomingX_MoveIndependentSteps_Async(motor_left,
                                              steps_scaled,   // signed!
                                              speed_k,
                                              accel_k,
                                              (uint8_t)async_slot);
}


static cmd_status_t Handle_MoveXIndependentUntilLimit_Async(cJSON *root, int async_slot)
{
    int speed_k=0;
    if (get_int(root, "speed", &speed_k) < 0) return CMD_ERR_BADARGS;

    const char *motor = get_string(root, "motor");
    if (!motor) return CMD_ERR_BADARGS;

    bool motor_left  = (strcmp(motor, "left") == 0);
    bool motor_right = (strcmp(motor, "right") == 0);
    if (!motor_left && !motor_right) return CMD_ERR_BADARGS;

    return HomingX_MoveIndependentUntilLimit_Async(motor_left,
                                                   speed_k,
                                                   (uint8_t)async_slot);
}

static cmd_status_t Handle_HomeXDualUntilLimit_Async(cJSON *root, int async_slot)
{
    int speed_k=0;
    if (get_int(root, "speed", &speed_k) < 0) return CMD_ERR_BADARGS;

    return HomingX_HomeDualUntilLimit_Async(speed_k, (uint8_t)async_slot);
}

static cmd_status_t Handle_MoveUntilLidLimit_Async(cJSON *root, int async_slot)
{
  int speed_k = 0;
  if (get_int(root, "speed", &speed_k) < 0) return CMD_ERR_BADARGS;

  // Optional: guard against <=0
  if (speed_k <= 0) return CMD_ERR_BADARGS;

  // active-low? In your GPIO init, LID_COLD_LIMIT has PULLUP,
  // so it's often active-low (pressed -> RESET). If yours is opposite, flip this.
  const bool lid_limit_active_low = false;

  return HomingX_MoveDualUntilPin_Async(LID_SEAL_LIMIT_GPIO_Port,
                                       LID_SEAL_LIMIT_Pin,
                                       lid_limit_active_low,
                                       speed_k,
                                       (uint8_t)async_slot);
}

void CalibrateDual_Pump(void)
{
  if (!g_calib_dual.active) return;

  // Always pump HomingX so X home can progress
  HomingX_Pump();

  if (g_calib_dual.phase == CD_X_HOMING) {
    if (HomingX_IsActive()) return; // still homing X

    // X done — check result
    if (HomingX_LastResult() != ASYNC_RES_OK) {
      uint8_t slot = g_calib_dual.slot;
      memset(&g_calib_dual, 0, sizeof(g_calib_dual));
      (void)async_evt_push_isr(slot, ASYNC_RES_STALLED); // or ABORTED depending on taste
      return;
    }

    // Now do Y homing using your existing blocking logic (reuse the code from Handle_Calibrate)
    const float FAST_SEEK = 5000.0f;
    const float SLOW_SEEK = 2000.0f;
    const int32_t BACKOFF_STEPS = 1000;

    const uint32_t MS = 1;
    const uint32_t SAMP_FAST_PRESS = 7;
    const uint32_t SAMP_SLOW_PRESS = 12;

    const bool Y_ACTIVE_LOW = false;

    bool ok =
      Y_MoveUntil_Limit(false, FAST_SEEK, GANTRY_Y_LIMIT_GPIO_Port, GANTRY_Y_LIMIT_Pin,
                        Y_ACTIVE_LOW, /*want_pressed*/true,
                        SAMP_FAST_PRESS, MS) &&
      (Gantry_Goto_Parabolic(0, +BACKOFF_STEPS, SLOW_SEEK, SLOW_SEEK), true) &&
      Y_MoveUntil_Limit(false, SLOW_SEEK, GANTRY_Y_LIMIT_GPIO_Port, GANTRY_Y_LIMIT_Pin,
                        Y_ACTIVE_LOW, /*want_pressed*/true,
                        SAMP_SLOW_PRESS, MS);

    uint8_t slot = g_calib_dual.slot;
    memset(&g_calib_dual, 0, sizeof(g_calib_dual));
    (void)async_evt_push_isr(slot, ok ? ASYNC_RES_OK : ASYNC_RES_STALLED);
    return;
  }
}


/* ========================= LID STEPPER COMMANDS ========================= */

static cmd_status_t Handle_LidTranslateMove_Async(cJSON *root, int async_slot)
{
  int motor_id = 0, steps = 0, speed = 0, accel = 0;
  if (get_int(root, "motor_id", &motor_id) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "steps",    &steps)    < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "speed",    &speed)    < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "accel",    &accel)    < 0) return CMD_ERR_BADARGS;
  if (speed <= 0 || accel <= 0)                  return CMD_ERR_BADARGS;

  return LidTranslate_StartMove(motor_id, (int32_t)steps,
                                (uint32_t)speed, (uint32_t)accel,
                                (uint8_t)async_slot);
}

static cmd_status_t Handle_LidTranslateCal_Async(cJSON *root, int async_slot)
{
  int motor_id = 0, speed = 0;
  if (get_int(root, "motor_id", &motor_id) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "speed",    &speed)    < 0) return CMD_ERR_BADARGS;
  if (speed <= 0)                               return CMD_ERR_BADARGS;
  return LidTranslate_StartCalibrate(motor_id, (uint32_t)speed, (uint8_t)async_slot);
}

static cmd_status_t Handle_LidRotateMove_Async(cJSON *root, int async_slot)
{
  int motor_id = 0, steps = 0, speed = 0, accel = 0;
  if (get_int(root, "motor_id", &motor_id) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "steps",    &steps)    < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "speed",    &speed)    < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "accel",    &accel)    < 0) return CMD_ERR_BADARGS;
  if (speed <= 0 || accel <= 0)                  return CMD_ERR_BADARGS;

  return LidRotate_StartMove(motor_id, (int32_t)steps,
                             (uint32_t)speed, (uint32_t)accel,
                             (uint8_t)async_slot);
}

static cmd_status_t Handle_LidSealMove_Async(cJSON *root, int async_slot)
{
  int steps = 0, speed = 0, accel = 0;
  if (get_int(root, "steps", &steps) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "speed", &speed) < 0) return CMD_ERR_BADARGS;
  if (get_int(root, "accel", &accel) < 0) return CMD_ERR_BADARGS;
  if (speed <= 0 || accel <= 0)            return CMD_ERR_BADARGS;

  return LidSeal_StartMove((int32_t)steps, (uint32_t)speed, (uint32_t)accel,
                           (uint8_t)async_slot);
}

static cmd_status_t Handle_LidSealCal_Async(cJSON *root, int async_slot)
{
  int speed = 0;
  if (get_int(root, "speed", &speed) < 0) return CMD_ERR_BADARGS;
  if (speed <= 0)                         return CMD_ERR_BADARGS;
  return LidSeal_StartCalibrate((uint32_t)speed, (uint8_t)async_slot);
}
