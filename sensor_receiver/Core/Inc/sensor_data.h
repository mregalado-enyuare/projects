#ifndef INC_SENSOR_DATA_H_
#define INC_SENSOR_DATA_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t distanceMm;
    uint8_t  rangeStatus;
    bool     isValid;
} SensorReading_t;

#endif /* INC_SENSOR_DATA_H_ */
