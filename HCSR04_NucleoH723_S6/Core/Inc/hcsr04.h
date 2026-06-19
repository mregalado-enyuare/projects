#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"

/* total number of HC-SR04 sensors wired up */
#define NUM_SENSORS 6

/* holds the pin config and last measured distance for one sensor */
typedef struct
{
    GPIO_TypeDef *trigPort;  // GPIO port for TRIG pin (e.g. GPIOA)
    uint16_t trigPin;        // TRIG pin number (e.g. GPIO_PIN_0)

    GPIO_TypeDef *echoPort;  // GPIO port for ECHO pin (e.g. GPIOB)
    uint16_t echoPin;        // ECHO pin number (e.g. GPIO_PIN_0)

    float distance_cm;       // last measured distance in cm, -1 if timeout

} HCSR04_t;

/* array of all sensors — defined in hcsr04.c, accessible everywhere */
extern HCSR04_t sensors[NUM_SENSORS];

void HCSR04_Init(void);                            // assign GPIO pins to each sensor
float HCSR04_Read(uint8_t sensor);                 // trigger one sensor and return distance in cm
void HCSR04_ScanAll(void);                         // scan all sensors and store results
float HCSR04_ReadAvg(uint8_t sensor, uint8_t samples); // average multiple readings for stability

#endif
