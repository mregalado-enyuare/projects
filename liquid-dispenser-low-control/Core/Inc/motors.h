#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"   // for GPIO_TypeDef, TIM_HandleTypeDef, UART_HandleTypeDef

// how many motors you have:
#define NUM_MOTORS   2
// steps per manual jog:
#define MOTOR_STEPS 100

// motor states
typedef enum {
	MOTOR_STATE_IDLE = 0, MOTOR_STATE_MOVING_UP, MOTOR_STATE_MOVING_DOWN, MOTOR_STATE_DISPENSE_UP, MOTOR_STATE_PULSE_DOWN,
} MotorState_t;

// motor control struct
typedef struct {
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;
	TIM_HandleTypeDef *htim;
	uint32_t timChannel;

	// legacy “move N steps” support
	volatile uint32_t stepCount;
	MotorState_t state;
	uint32_t baseARR;
	uint32_t decelARR;
	uint32_t decelSteps;

	// —— new fields for profiler-driven moves ——
	uint32_t *stepProfile;   // points at the array of ARR values
	uint32_t profileSize;    // how many entries in stepProfile[]
	uint32_t profileIndex;   // next index to pull from stepProfile
} MotorControl_t;

// all of these live in main.c, so we just declare them extern here:
extern MotorControl_t motors[NUM_MOTORS];
extern int32_t gantryXPos, gantryYPos;
extern int32_t startTargetX, startTargetY;
extern int32_t cupTargetX, cupTargetY;
extern int32_t moveTargetX, moveTargetY;
extern int32_t serveTargetX, serveTargetY;
extern UART_HandleTypeDef huart3;

// prototypes for any functions commands.c calls
void DisableMotor(MotorControl_t *m);
void Motor_MoveSteps(MotorControl_t *m, bool up, uint32_t steps);
void UpdateEncoderPosition(void);
bool moveForward(uint32_t move_value, uint32_t speed);
bool moveBackward(uint32_t move_value, uint32_t speed);
const char *syringe_last_stall_msg(void);
void syringe_clear_stall_msg(void);
void syringe_set_stall_context(int rack_num, int motor_id);
void Gantry_Goto(int32_t targetX, int32_t targetY, float startSpeed, float endSpeed);
void Circle(int count, int radius, int speed);

#endif // MOTORS_H
