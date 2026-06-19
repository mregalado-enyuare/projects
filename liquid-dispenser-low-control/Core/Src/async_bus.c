/*
 * async_bus.c
 *
 *  Created on: Oct 30, 2025
 *      Author: eostroff_enyuare
 */

#include "async_bus.h"
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

static async_slot_t g_slots[ASYNC_MAX_SLOTS];

static volatile uint8_t q_head = 0, q_tail = 0;
static async_evt_t g_q[ASYNC_Q_CAP];

void async_bus_init(void) {
  memset(g_slots, 0, sizeof(g_slots));
  q_head = q_tail = 0;
}

int async_claim_slot(const char *cmd_id, struct udp_pcb *pcb,
                     const ip_addr_t *addr, u16_t port)
{
  for (int i = 0; i < ASYNC_MAX_SLOTS; ++i) {
    if (!g_slots[i].in_use) {
      g_slots[i].in_use = true;
      strncpy(g_slots[i].cmd_id, cmd_id ? cmd_id : "", ASYNC_CMDID_LEN-1);
      g_slots[i].cmd_id[ASYNC_CMDID_LEN-1] = '\0';
      g_slots[i].pcb  = pcb;
      ip_addr_set(&g_slots[i].addr, addr);
      g_slots[i].port = port;

      char ipbuf[48];
      ipaddr_ntoa_r(addr, ipbuf, sizeof ipbuf);
      printf("claim slot %d for %s:%u\n", i, ipbuf, (unsigned)port);

      return i;
    }
  }
  return -1;
}

void async_release_slot(uint8_t slot) {
  if (slot < ASYNC_MAX_SLOTS) {
    g_slots[slot].in_use = false;
  }
}

bool async_slot_valid(uint8_t slot) {
  return (slot < ASYNC_MAX_SLOTS) && g_slots[slot].in_use;
}

bool async_slot_get(uint8_t slot, const char **cmd_id_out,
                    struct udp_pcb **pcb_out, const ip_addr_t **addr_out, u16_t *port_out)
{
  if (!async_slot_valid(slot)) return false;
  if (cmd_id_out) *cmd_id_out = g_slots[slot].cmd_id;
  if (pcb_out)    *pcb_out    = g_slots[slot].pcb;
  if (addr_out)   *addr_out   = &g_slots[slot].addr;
  if (port_out)   *port_out   = g_slots[slot].port;
  return true;
}

// Single-producer (ISR) / single-consumer (main) ring buffer
bool async_evt_push_isr(uint8_t slot, async_result_t r) {
  uint8_t next = (uint8_t)(q_head + 1) & (ASYNC_Q_CAP - 1);
  if (next == q_tail) {
    printf("async_evt_push_isr: QUEUE OVERFLOW\n");
    return false; // overflow; drop
  }
  g_q[q_head].slot   = slot;
  g_q[q_head].result = r;
  q_head = next;
  return true;
}

bool async_evt_pop(async_evt_t *out) {
  if (q_tail == q_head) return false;
  *out = g_q[q_tail];
  q_tail = (uint8_t)(q_tail + 1) & (ASYNC_Q_CAP - 1);
  return true;
}
