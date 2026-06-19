#include "cup_detector.h"
#include "hcsr04.h"

/* holds the current status for each sensor slot — accessible from main.c */
SlotStatus_t slotStatus[6];

#define HISTORY_SIZE  3      // number of past readings averaged per sensor
#define CUP_MIN_CM    19.0f  // minimum distance to count as a cup (top of 18cm cup at ~23cm)
#define TRASH_MAX_CM  35.0f  // maximum distance to count as trash before it reads as empty

/* each sensor may be mounted at a slightly different height so each
   has its own empty baseline — anything within 2cm of this = EMPTY */
static float sensorEmptyCm[6] = {
    42.0f,  // sensor 0
    42.0f,  // sensor 1
    42.0f,  // sensor 2
    42.0f,  // sensor 3
    38.0f,  // sensor 4 — sits slightly lower than others
    42.0f   // sensor 5
};

/* sensor 4 consistently reads higher when a cup is present
   so it gets a wider cup window than the others */
static float sensorCupMaxCm[6] = {
    27.0f,  // sensor 0
    27.0f,  // sensor 1
    27.0f,  // sensor 2
    27.0f,  // sensor 3
    30.0f,  // sensor 4 — wider range to catch cup at this position
    27.0f   // sensor 5
};

/* rolling history buffer — stores last HISTORY_SIZE readings per sensor */
static float history[6][HISTORY_SIZE];
static uint8_t histIndex[6]   = {0};  // next write position in the ring buffer
static uint8_t histFull[6]    = {0};  // set to 1 once the buffer has been filled once
static uint8_t stableCount[6] = {0};  // consecutive readings matching current state
static SlotStatus_t lastStatus[6] = {SLOT_EMPTY};  // last confirmed state per sensor

/* returns the rolling average of stored history readings for one sensor
   returns -1 if no valid readings are in the buffer yet */
static float GetAverage(uint8_t sensor)
{
    float sum = 0;
    uint8_t count = histFull[sensor] ? HISTORY_SIZE : histIndex[sensor];
    if(count == 0) return -1.0f;
    for(uint8_t i = 0; i < count; i++)
        sum += history[sensor][i];
    return sum / count;
}

/* takes an averaged distance and maps it to a slot status
   uses per-sensor thresholds since not all sensors sit at the same height */
static SlotStatus_t ClassifyDistance(float d, uint8_t sensor)
{
    float emptyCm  = sensorEmptyCm[sensor];
    float cupMaxCm = sensorCupMaxCm[sensor];

    if(d < 0)
        return SLOT_ERROR;                        // sensor timed out
    else if(d >= emptyCm - 2.0f)
        return SLOT_EMPTY;                        // nothing in slot
    else if(d >= CUP_MIN_CM && d <= cupMaxCm)
        return SLOT_CUP_PRESENT;                  // distance matches cup height
    else if(d > cupMaxCm && d <= TRASH_MAX_CM)
        return SLOT_TRASH;                        // shorter than cup = trash
    else
        return SLOT_OBSTRUCTION;                  // something unexpected
}

/* called every loop cycle — updates slotStatus[] for all sensors
   uses rolling average + stable count to avoid flickering */
void CupDetector_Update(void)
{
    for(int i = 0; i < NUM_SENSORS; i++)
    {
        float raw = sensors[i].distance_cm;

        /* only add valid readings to history — skip timeouts */
        if(raw > 0)
        {
            history[i][histIndex[i]] = raw;
            histIndex[i] = (histIndex[i] + 1) % HISTORY_SIZE;  // wrap around ring buffer
            if(histIndex[i] == 0) histFull[i] = 1;  // mark buffer as full after first pass
        }

        /* classify the smoothed average reading */
        float avg = GetAverage(i);
        SlotStatus_t current = ClassifyDistance(avg, i);

        /* only commit a state change after 2 consecutive matching readings
           this prevents the status flickering when a cup is moving or
           sitting between two sensors */
        if(current == lastStatus[i])
        {
            if(stableCount[i] < 2) stableCount[i]++;
        }
        else
        {
            stableCount[i] = 1;   // reset counter on state change
            lastStatus[i] = current;
        }

        if(stableCount[i] >= 2)
            slotStatus[i] = current;  // confirmed stable — update the slot status
    }
}

/* looks at all active sensors and returns one overall system state
   skips any sensors showing ERROR so dead sensors don't affect the result */
SystemStatus_t GetSystemConsensus(void)
{
    int cupCount   = 0;
    int trashCount = 0;
    int emptyCount = 0;

    for(int i = 0; i < NUM_SENSORS; i++)
    {
        if(slotStatus[i] == SLOT_ERROR) continue;  // ignore dead sensors
        switch(slotStatus[i])
        {
            case SLOT_CUP_PRESENT: cupCount++;   break;
            case SLOT_TRASH:       trashCount++; break;
            case SLOT_EMPTY:       emptyCount++; break;
            default:               break;
        }
    }

    if(cupCount > 0 && trashCount == 0)      return SYSTEM_CUP_DETECTED;   // cups only
    else if(trashCount > 0 && cupCount == 0) return SYSTEM_TRASH_DETECTED; // trash only
    else if(emptyCount == NUM_SENSORS - 1)   return SYSTEM_ALL_EMPTY;      // all clear
    else                                     return SYSTEM_MIXED;          // mix of both
}

/* counts cups across all sensors using neighbor deduplication —
   if two sensors next to each other both detect a cup it counts
   as one cup not two, since the cup is just between them
   result is capped at 3 matching the max slots in the tray */
uint8_t GetCupCount(void)
{
    uint8_t count = 0;
    uint8_t counted[6] = {0};  // tracks which sensors have already been counted

    for(int i = 0; i < NUM_SENSORS; i++)
    {
        if(slotStatus[i] == SLOT_CUP_PRESENT && !counted[i])
        {
            count++;
            counted[i] = 1;

            /* if the next sensor also sees a cup, skip it —
               same physical cup being picked up by two sensors */
            if(i + 1 < NUM_SENSORS &&
               slotStatus[i + 1] == SLOT_CUP_PRESENT)
                counted[i + 1] = 1;
        }
    }

    if(count > 3) count = 3;  // hard cap at 3 cups max
    return count;
}
