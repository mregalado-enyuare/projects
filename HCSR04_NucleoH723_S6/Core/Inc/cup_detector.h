#ifndef CUP_DETECTOR_H
#define CUP_DETECTOR_H

#include <stdint.h>

/* status for each individual sensor slot */
typedef enum
{
    SLOT_EMPTY,        // nothing detected
    SLOT_CUP_PRESENT,  // cup in slot
    SLOT_TRASH,        // trash detected (shorter than cup)
    SLOT_OBSTRUCTION,  // something unexpected
    SLOT_ERROR         // sensor timed out

} SlotStatus_t;

/* overall system status based on all sensors combined */
typedef enum
{
    SYSTEM_ALL_EMPTY,       // all slots empty
    SYSTEM_CUP_DETECTED,    // at least one cup, no trash
    SYSTEM_TRASH_DETECTED,  // at least one trash, no cups
    SYSTEM_MIXED            // mix of cups and trash

} SystemStatus_t;

/* accessible from main.c and anywhere that includes this header */
extern SlotStatus_t slotStatus[6];

void CupDetector_Update(void);       // run every loop to update all slot statuses
SystemStatus_t GetSystemConsensus(void); // returns overall system state
uint8_t GetCupCount(void);           // returns number of cups (max 3)

#endif
