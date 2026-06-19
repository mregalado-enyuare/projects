#include "hcsr04.h"
#include "delay_us.h"

/* array holding pin config and last distance for all 6 sensors */
HCSR04_t sensors[NUM_SENSORS];

/* sends the trigger pulse to start one measurement
   HC-SR04 needs a clean 10us HIGH pulse on TRIG to fire */
static void Trigger(HCSR04_t *s)
{
    HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_RESET);
    delay_us(2);   // ensure line is low before triggering

    HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_SET);
    delay_us(10);  // hold high for 10us to trigger the sensor

    HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_RESET);
}

/* maps each sensor index to its TRIG and ECHO GPIO pins
   TRIG = PA0-PA5 (outputs), ECHO = PB0/1/2/10/11/12 (inputs)
   last field (0) initializes distance_cm to zero */
void HCSR04_Init(void)
{
    sensors[0] = (HCSR04_t){GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_0,  0};
    sensors[1] = (HCSR04_t){GPIOA, GPIO_PIN_1, GPIOB, GPIO_PIN_1,  0};
    sensors[2] = (HCSR04_t){GPIOA, GPIO_PIN_2, GPIOB, GPIO_PIN_2,  0};
    sensors[3] = (HCSR04_t){GPIOA, GPIO_PIN_3, GPIOB, GPIO_PIN_10, 0};
    sensors[4] = (HCSR04_t){GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_11, 0};
    sensors[5] = (HCSR04_t){GPIOA, GPIO_PIN_5, GPIOB, GPIO_PIN_12, 0};
}

/* triggers one sensor and returns the measured distance in cm
   returns -1.0 if the echo never arrives within the 30ms timeout */
float HCSR04_Read(uint8_t sensor)
{
    HCSR04_t *s = &sensors[sensor];

    Trigger(s);
    delay_us(5);  // short settle time after trigger before listening for echo

    uint32_t timeout = HAL_GetTick();

    /* wait for ECHO pin to go HIGH — this marks the start of the echo pulse
       if it never goes HIGH within 30ms the sensor is not responding */
    while(HAL_GPIO_ReadPin(s->echoPort, s->echoPin) == GPIO_PIN_RESET)
    {
        if((HAL_GetTick() - timeout) > 30)
        {
            s->distance_cm = -1.0f;  // store error value
            return -1.0f;
        }
    }

    /* record cycle count at the moment echo goes HIGH */
    uint32_t start = DWT->CYCCNT;
    timeout = HAL_GetTick();  // reset timeout for the HIGH phase

    /* wait for ECHO pin to go LOW — this marks the end of the echo pulse
       if it stays HIGH longer than 30ms the object is too far away */
    while(HAL_GPIO_ReadPin(s->echoPort, s->echoPin) == GPIO_PIN_SET)
    {
        if((HAL_GetTick() - timeout) > 30)
        {
            s->distance_cm = -1.0f;  // store error value
            return -1.0f;
        }
    }

    /* record cycle count at the moment echo goes LOW */
    uint32_t stop = DWT->CYCCNT;

    /* convert cycle count difference to microseconds using CPU clock speed */
    float pulse_us = (stop - start) / (SystemCoreClock / 1000000.0f);

    /* distance = pulse time * speed of sound / 2 (divide by 2 for round trip) */
    float distance = pulse_us * 0.0343f / 2.0f;

    s->distance_cm = distance;  // store result in sensor struct
    return distance;
}

/* takes multiple readings from one sensor and returns the average
   ignores any -1 timeout readings so they don't skew the result */
float HCSR04_ReadAvg(uint8_t sensor, uint8_t samples)
{
    float sum = 0;
    uint8_t valid = 0;

    for(uint8_t i = 0; i < samples; i++)
    {
        float r = HCSR04_Read(sensor);
        if(r > 0)  // only count successful readings
        {
            sum += r;
            valid++;
        }
        HAL_Delay(2);  // short gap between samples to avoid echo overlap
    }

    if(valid == 0) return -1.0f;  // all samples timed out
    return sum / valid;
}

/* scans all sensors one at a time and stores averaged distance in each sensor struct
   small delay between sensors prevents one sensor's echo from
   being picked up by the next sensor (crosstalk) */
void HCSR04_ScanAll(void)
{
    for(uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i].distance_cm = HCSR04_ReadAvg(i, 2);
        HAL_Delay(2);  // gap between sensors to prevent crosstalk
    }
}
