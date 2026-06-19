#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include "stm32h7xx_hal.h"
#include "sensor_data.h"
#include <stdbool.h>
#include <stdio.h>

#define RX_BUFFER_SIZE 64

extern UART_HandleTypeDef huart4;

void  UART_Init(void);
void  UART_ReceiveIT(void);
void  UART_ParseBuffer(char* buffer, SensorReading_t* reading);
void  UART_DebugPrint(SensorReading_t* reading);
bool  UART_IsDataReady(void);
void  UART_ClearDataReady(void);
char* UART_GetBuffer(void);

#endif /* INC_UART_HANDLER_H_ */
