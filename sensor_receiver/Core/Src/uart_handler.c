#include "uart_handler.h"
#include <stdio.h>
#include <string.h>

// ── Private Variables ───────────────────────────
static char    rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxByte;
static uint8_t rxIndex   = 0;
static bool    dataReady = false;

// ── Public Functions ────────────────────────────
void UART_Init(void) {
    HAL_UART_Receive_IT(&huart4, &rxByte, 1);
}

void UART_ReceiveIT(void) {
    if (rxByte == '\n') {
        rxBuffer[rxIndex] = '\0';
        rxIndex   = 0;
        dataReady = true;
    } else {
        if (rxIndex < RX_BUFFER_SIZE - 1) {
            rxBuffer[rxIndex++] = rxByte;
        }
    }
    HAL_UART_Receive_IT(&huart4, &rxByte, 1);
}

void UART_ParseBuffer(char* buffer, SensorReading_t* reading) {
    int parsed = sscanf(buffer,
                        "DIST:%hu,STATUS:%hhu",
                        &reading->distanceMm,
                        &reading->rangeStatus);
    reading->isValid = (parsed == 2);
}

void UART_DebugPrint(SensorReading_t* reading) {
    printf("[H723ZG] Distance: %u mm | Status: %u\r\n",
           reading->distanceMm,
           reading->rangeStatus);
}

bool  UART_IsDataReady(void)    { return dataReady; }
void  UART_ClearDataReady(void) { dataReady = false; }
char* UART_GetBuffer(void)      { return rxBuffer; }
