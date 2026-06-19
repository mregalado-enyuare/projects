#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "cJSON.h"

#define MAX_ARR_VALUE  0xFFFFU
#define MAX_STEPS 25000

/**
 * @brief  Execute a single command line (null-terminated).
 * @param  cmdLine  Command string (trimmed, case-insensitive).
 * @retval true if handled, false to fall back.
 */
bool Command_Execute(const char *cmdLine);
void CalibrateDual_Pump(void);


typedef enum {
  CMD_OK_STARTED = 0,      // non-blocking, will post later
  CMD_OK_IMMEDIATE = 1,    // completed now; caller can send DONE immediately
  CMD_ERR_BADARGS = -1,
  CMD_ERR_BUSY    = -2,
  CMD_ERR_STALLED = -3,    // command completed but motor stalled
} cmd_status_t;

cmd_status_t Command_Execute_Async(const char *cmdLine, int async_slot);

const char *get_string(cJSON *root, const char *key);
int get_int(cJSON *root, const char *key, int *out);

#endif
