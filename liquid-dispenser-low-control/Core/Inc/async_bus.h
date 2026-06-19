/*
 * async_bus.h
 *
 *  Created on: Oct 30, 2025
 *      Author: eostroff_enyuare
 */

#ifndef INC_ASYNC_BUS_H_
#define INC_ASYNC_BUS_H_

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include <stdbool.h>
#include <stdint.h>

#define ASYNC_MAX_SLOTS 16 // concurrent ops (tune)
#define ASYNC_CMDID_LEN 37 // UUID 36 + NUL
#define ASYNC_Q_CAP     16 // completion events (power of two)

// Sentinel value meaning "no async slot / internal operation".
// This value must never equal a valid claimed slot (0..ASYNC_MAX_SLOTS-1).
#define ASYNC_SLOT_NONE  ((uint8_t)0xFF)

typedef enum {
  ASYNC_RES_NONE = 0,
  ASYNC_RES_OK,
  ASYNC_RES_STALLED,
  ASYNC_RES_ABORTED,
} async_result_t;

typedef struct {
  volatile bool in_use;
  char cmd_id[ASYNC_CMDID_LEN];
  struct udp_pcb *pcb;
  ip_addr_t addr;
  u16_t port;
} async_slot_t;

typedef struct {
  uint8_t slot; // which slot completed
  async_result_t result;
} async_evt_t;

void async_bus_init(void);
int async_claim_slot(const char *cmd_id, struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port); // returns slot >=0, else -1
void async_release_slot(uint8_t slot);
bool async_slot_valid(uint8_t slot);

bool async_evt_push_isr(uint8_t slot, async_result_t r); // ISR-safe push
bool async_evt_pop(async_evt_t *out); // main loop pop

// helpers to fetch saved reply info for a slot
bool async_slot_get(uint8_t slot, const char **cmd_id_out,
                    struct udp_pcb **pcb_out, const ip_addr_t **addr_out, u16_t *port_out);

#endif /* INC_ASYNC_BUS_H_ */