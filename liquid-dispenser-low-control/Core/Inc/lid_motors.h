#ifndef INC_LID_MOTORS_H_
#define INC_LID_MOTORS_H_

#include "dm556_parabolic_motion.h"
#include "commands.h"
#include <stdbool.h>
#include <stdint.h>

// Maximum steps per lid motor move command (limits scratch buffer size)
#define LID_MOTOR_MAX_STEPS  8000U

extern ParabolicMotorControl_t motorLidTranslate1;
extern ParabolicMotorControl_t motorLidTranslate2;
extern ParabolicMotorControl_t motorLidRotate1;
extern ParabolicMotorControl_t motorLidRotate2;
extern ParabolicMotorControl_t motorLidSeal;

// Call once after timer inits, before the main loop
void lid_motors_init(void);

// Async profiled-step moves (return CMD_OK_STARTED or CMD_ERR_BUSY/CMD_ERR_BADARGS)
cmd_status_t LidTranslate_StartMove(int motor_id, int32_t steps,
                                    uint32_t speed_hz, uint32_t accel_hz,
                                    uint8_t async_slot);

cmd_status_t LidRotate_StartMove(int motor_id, int32_t steps,
                                 uint32_t speed_hz, uint32_t accel_hz,
                                 uint8_t async_slot);

cmd_status_t LidSeal_StartMove(int32_t steps, uint32_t speed_hz,
                               uint32_t accel_hz, uint8_t async_slot);

// Async two-pass homing (fast seek → back off → slow precision seek).
// Returns CMD_OK_STARTED; DONE pushed via async bus when complete.
// Only one calibration may run at a time across all lid motors.
cmd_status_t LidTranslate_StartCalibrate(int motor_id, uint32_t speed_hz, uint8_t async_slot);
cmd_status_t LidSeal_StartCalibrate(uint32_t speed_hz, uint8_t async_slot);

// Call every main-loop iteration to advance the calibration state machine.
void LidCal_Pump(void);

#endif /* INC_LID_MOTORS_H_ */
