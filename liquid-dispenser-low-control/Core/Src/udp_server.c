// udp_server.c
#include "udp_server.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "commands.h"
#include "motors.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include "cJSON.h"
#include "lwip/sys.h"
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip6_addr.h"
#include "lwip/ip.h"
#include "lwip.h"
#include "lwip/inet.h"
#include "async_bus.h"

// Debug over UART3
#include "stm32h7xx_hal.h"
extern UART_HandleTypeDef huart3;

static inline void DBG(const char *s) {
    HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}
static void DBGf(const char *tag, int er, unsigned len) {
    char b[96];
    int n = snprintf(b, sizeof(b), "[%lu] %s er=%d len=%u\r\n",
                     (unsigned long)HAL_GetTick(), tag, er, len);
    if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t*)b, n, HAL_MAX_DELAY);
}

#define MAX_CMD_LEN  512
#define UUID_STR_LEN 37       /* 36 chars + NUL */

static char prev_cmd_id[UUID_STR_LEN] = "";

static struct udp_pcb *cmd_pcb;
static uint8_t board_id;

static struct udp_pcb  *log_pcb;

/* All outbound messages are broadcast to this address/port */
static ip_addr_t bcast_addr;
#define BCAST_PORT  UDP_SERVER_PORT
#define LOG_PORT    9001

/* how long to wait (ms) before re-sending DONE */
#define DONE_TIMEOUT_MS   2000
#define DONE_MAX_RETRIES  3   /* resend DONE at most this many times */

static char    last_done_str[256];
static u16_t   last_done_len;
static bool    waiting_for_cmd = false;
static uint8_t done_retries = 0;

/* ---------- helpers ---------- */

static void udp_send_json(struct udp_pcb *pcb,
                          const ip_addr_t *addr,
                          u16_t port,
                          const char *json,
                          const char *tag)
{
    if (!json) return;
    u16_t len = (u16_t)strlen(json);
    DBG(json); DBG("\r\n");
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
    if (!p) {
        if (tag) DBG("[SEND] pbuf_alloc FAIL\r\n");
        return;
    }

    memcpy(p->payload, json, len);
    err_t er = udp_sendto(pcb, p, addr, port);
    if (tag) DBGf(tag, er, len);
    pbuf_free(p);
}

/* Sends a cJSON object, optionally snapshotting the serialized string. */
static void udp_send_cjson(struct udp_pcb *pcb,
                           const ip_addr_t *addr,
                           u16_t port,
                           cJSON *obj,
                           const char *tag,
                           char *snapshot_buf,     /* may be NULL */
                           size_t snapshot_buf_sz, /* 0 if none   */
                           u16_t *snapshot_len)    /* may be NULL */
{
    if (!obj) return;
    char *s = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (!s) { DBG("[SEND] cJSON_Print OOM\r\n"); return; }

    if (snapshot_buf && snapshot_buf_sz > 1) {
        size_t sl = strlen(s);
        size_t copy = (sl < (snapshot_buf_sz - 1)) ? sl : (snapshot_buf_sz - 1);
        memcpy(snapshot_buf, s, copy);
        snapshot_buf[copy] = '\0';
        if (snapshot_len) *snapshot_len = (u16_t)copy;
    }

    udp_send_json(pcb, addr, port, s, tag);
    cJSON_free(s);
}


static void resend_done(void *arg);

// --- DONE sender (used by the pump) ---
// Uses snprintf instead of cJSON to avoid malloc — the heap is very tight
// (< 2 KB) and cJSON_CreateObject was intermittently returning NULL.
static void send_done_now(const char *cmd_id, const char *msg)
{
    snprintf(last_done_str, sizeof(last_done_str),
             "{\"cmd_id\":\"%s\",\"cmd\":\"DONE\",\"msg\":\"%s\"}",
             cmd_id ? cmd_id : "",
             msg    ? msg    : "");
    last_done_len = (u16_t)strlen(last_done_str);
    udp_send_json(cmd_pcb, &bcast_addr, BCAST_PORT, last_done_str, "DONE send");
    done_retries = 0;
    waiting_for_cmd = true;
    sys_timeout(DONE_TIMEOUT_MS, resend_done, NULL);
}


void udp_async_pump(void)
{
    async_evt_t ev;
    while (async_evt_pop(&ev)) {  // <-- actually pop events
        const char *cmd_id = NULL;
        struct udp_pcb *pcb = NULL;
        const ip_addr_t *addr = NULL;
        u16_t port = 0;

        if (async_slot_get(ev.slot, &cmd_id, &pcb, &addr, &port)) {
            char stall_buf[128];
            const char *msg;
            if (ev.result == ASYNC_RES_OK) {
                msg = "OK EXECUTE";
            } else if (ev.result == ASYNC_RES_STALLED) {
                const char *detail = syringe_last_stall_msg();
                if (detail && detail[0]) {
                    snprintf(stall_buf, sizeof(stall_buf), "FAILED STALLED: %s", detail);
                    msg = stall_buf;
                } else {
                    msg = "FAILED STALLED";
                }
            } else if (ev.result == ASYNC_RES_ABORTED) {
                msg = "FAILED ABORTED";
            } else {
                msg = "UNKNOWN";
            }

            printf("DONE (broadcast) for %s (slot %u)\n", cmd_id, ev.slot);

            send_done_now(cmd_id, msg);
        }
        async_release_slot(ev.slot);
    }
}


/* ---------- resend timer ---------- */

static void resend_done(void *arg)
{
    if (!waiting_for_cmd) return;

    if (done_retries >= DONE_MAX_RETRIES) {
        DBG("[TMR] resend_done: max retries reached\r\n");
        waiting_for_cmd = false;
        return;
    }

    DBG("[TMR] resend_done firing\r\n");

    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, last_done_len, PBUF_POOL);
    if (!out) {
        DBG("[TMR] pbuf_alloc FAIL\r\n");
    } else {
        memcpy(out->payload, last_done_str, last_done_len);
        err_t er = udp_sendto(cmd_pcb, out, &bcast_addr, BCAST_PORT);
        DBGf("DONE resend", er, last_done_len);
        pbuf_free(out);
    }

    done_retries++;
    if (done_retries < DONE_MAX_RETRIES) {
        sys_timeout(DONE_TIMEOUT_MS, resend_done, NULL);
    } else {
        waiting_for_cmd = false;
    }
}

/* ---------- recv callback ---------- */

static void udp_recv_cb(void *arg,
                        struct udp_pcb *pcb,
                        struct pbuf *p,
                        const ip_addr_t *addr,
                        u16_t port)
{
    char b[96];
    char astr[48];
    uint32_t now = HAL_GetTick();
    ipaddr_ntoa_r(addr, astr, sizeof(astr));
    int n = snprintf(b, sizeof(b),
                     "[%lu] RECV from %s:%u tot_len=%u len=%u\r\n",
                     (unsigned long)now, astr, (unsigned)port,
                     (unsigned)p->tot_len, (unsigned)p->len);
    HAL_UART_Transmit(&huart3, (uint8_t*)b, n, HAL_MAX_DELAY);

    char error_msg[128] = {0};
    const char *cmd_id = NULL;
    char cmd_id_copy[UUID_STR_LEN] = {0};  /* <- safe copy lives after cJSON free */
    bool execute_command = true;

    /* bounds check */
    if (p->tot_len == 0 || p->tot_len >= MAX_CMD_LEN) {
        snprintf(error_msg, sizeof(error_msg),
                 "EXCEEDED MAX_CMD_LEN: %u > %u",
                 (unsigned)p->tot_len, (unsigned)MAX_CMD_LEN);
        pbuf_free(p);
        execute_command = false;
        goto send_ack;
    }

    /* copy & sanitize */
    char buf[MAX_CMD_LEN];
    pbuf_copy_partial(p, buf, p->tot_len, 0);
    buf[p->tot_len] = '\0';
    pbuf_free(p);

    size_t L = strlen(buf);
    if (L > 0 && (buf[L-1]=='\r' || buf[L-1]=='\n')) {
        buf[L-1] = '\0';
    }


    /* parse JSON */
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        snprintf(error_msg, sizeof(error_msg),
                 "JSON PARSE ERROR: %s",
                 cJSON_GetErrorPtr());
        execute_command = false;
        goto send_ack;
    }

    /* rack check — silently drop if rack_num doesn't match */
    {
        cJSON *rn = cJSON_GetObjectItemCaseSensitive(root, "rack_num");
        if (!cJSON_IsNumber(rn) || rn->valueint != board_id) {
            cJSON_Delete(root);
            return;
        }
    }
    DBG(buf); DBG("\r\n");

    /* DONE_ACK handling — host acknowledging our DONE */
    {
        cJSON *j_cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
        if (cJSON_IsString(j_cmd) && strcmp(j_cmd->valuestring, "DONE_ACK") == 0) {
            cJSON *j_id = cJSON_GetObjectItemCaseSensitive(root, "cmd_id");
            if (cJSON_IsString(j_id) && strcmp(j_id->valuestring, prev_cmd_id) == 0) {
                if (waiting_for_cmd) {
                    sys_untimeout(resend_done, NULL);
                    waiting_for_cmd = false;
                    DBG("[DONE_ACK] received, resend cancelled\r\n");
                }
            }
            cJSON_Delete(root);
            return;
        }
    }

    /* cancel pending resend — only after confirming this packet is for us */
    if (waiting_for_cmd) {
        sys_untimeout(resend_done, NULL);
        waiting_for_cmd = false;
    }

    /* cmd_id required */
    cmd_id = get_string(root, "cmd_id");
    if (!cmd_id) {
        snprintf(error_msg, sizeof(error_msg), "NO CMD_ID IN MESSAGE");
        execute_command = false;
        goto send_ack_free_root;
    }
    /* Make a SAFE copy before we might delete 'root' */
    strncpy(cmd_id_copy, cmd_id, UUID_STR_LEN - 1U);
    cmd_id_copy[UUID_STR_LEN - 1U] = '\0';

    /* duplicate suppression */
    if (prev_cmd_id[0] != '\0' && strcmp(cmd_id_copy, prev_cmd_id) == 0) {
        snprintf(error_msg, sizeof(error_msg), "DUPLICATE CMD_ID: %s", cmd_id_copy);
        execute_command = false;
        goto send_ack_free_root;
    }

    /* remember cmd_id */
    strncpy(prev_cmd_id, cmd_id_copy, UUID_STR_LEN - 1U);
    prev_cmd_id[UUID_STR_LEN - 1U] = '\0';

    log_pcb = cmd_pcb;

    /* Prime neighbor/ARP cache for the peer (ARP only for IPv4) */
    #if LWIP_IPV4 && LWIP_IPV6
    if (IP_IS_V4(addr)) {
        const ip4_addr_t *a4 = ip_2_ip4(addr);
        etharp_query(netif_default, a4, NULL);
    }
    #elif LWIP_IPV4
    etharp_query(netif_default, ip_2_ip4(addr), NULL);
    #endif

send_ack_free_root:
    cJSON_Delete(root);

send_ack:
    /* Build + send ACK — stack buffer, no malloc */
    {
        char ack_json[192];
        if (error_msg[0])
            snprintf(ack_json, sizeof(ack_json),
                     "{\"cmd_id\":\"%s\",\"cmd\":\"ACK\",\"msg\":\"%s\"}",
                     cmd_id_copy, error_msg);
        else
            snprintf(ack_json, sizeof(ack_json),
                     "{\"cmd_id\":\"%s\",\"cmd\":\"ACK\"}",
                     cmd_id_copy);
        udp_send_json(cmd_pcb, &bcast_addr, BCAST_PORT, ack_json, "ACK send");
        MX_LWIP_Process();
    }

    if (execute_command) {
        // 1) Claim a slot tied to this command's reply address
        int slot = async_claim_slot(cmd_id_copy, pcb, addr, port);
        if (slot < 0) {
            // Too busy to accept; tell caller right away.
            char busy_json[192];
            snprintf(busy_json, sizeof(busy_json),
                     "{\"cmd_id\":\"%s\",\"cmd\":\"DONE\",\"msg\":\"FAILED BUSY\"}",
                     cmd_id_copy);
            udp_send_json(cmd_pcb, &bcast_addr, BCAST_PORT, busy_json, "DONE send");
            return;
        }

        // 2) Start non-blocking execution; it must attach 'slot'
        cmd_status_t st = Command_Execute_Async(buf, slot);

        if (st == CMD_OK_STARTED) {
            // Perfect: ACK already sent above; DONE will be posted by ISR later.
            return;
        }

        // 3) For IMMEDIATE or FAILED-TO-START, enqueue a completion
        //    so the pump sends DONE (single-path for DONE sending).
        async_result_t r =
            (st == CMD_OK_IMMEDIATE) ? ASYNC_RES_OK :
            (st == CMD_ERR_STALLED)  ? ASYNC_RES_STALLED :
                                       ASYNC_RES_ABORTED;

        // Post from foreground (not ISR):
        (void)async_evt_push_isr((uint8_t)slot, r);  // foreground-safe ring buffer push
        // Let the pump send DONE and release the slot.
    }
}

/* Init: create PCB, bind & register callback */
void udp_server_init(uint8_t my_id)
{
    board_id = my_id;

    IP4_ADDR(ip_2_ip4(&bcast_addr), 10, 10, 1, 255);
    IP_SET_TYPE_VAL(bcast_addr, IPADDR_TYPE_V4);

    cmd_pcb = udp_new();
    if (!cmd_pcb) return;

    ip_set_option(cmd_pcb, SOF_BROADCAST);
    udp_bind(cmd_pcb, IP_ADDR_ANY, UDP_SERVER_PORT);
    udp_recv(cmd_pcb, udp_recv_cb, NULL);
}

/* Add: send raw log data to the last-known peer via UDP. */
void udp_log_send(const char *data, int len)
{
    if (!data || len <= 0) return;
    if (!log_pcb) return;          // no peer known yet

    /* clamp length to reasonable size to avoid pbuf allocation failures */
    if (len > 2048) len = 2048;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_POOL);
    if (!p) {
        DBG("[LOG] pbuf_alloc FAIL\r\n");
        return;
    }

    memcpy(p->payload, data, (size_t)len);
    err_t er = udp_sendto(log_pcb, p, &bcast_addr, BCAST_PORT);
    DBGf("LOG send", er, (unsigned)len);
    pbuf_free(p);
}

/* Non-blocking UDP log queue (ring buffer) */
#define LOG_RINGBUF_SIZE 4096
static uint8_t log_ringbuf[LOG_RINGBUF_SIZE];
static volatile uint32_t log_rb_head = 0; // write index
static volatile uint32_t log_rb_tail = 0; // read index
static volatile uint32_t log_dropped = 0;

/* enqueue up to len bytes into ring buffer; non-blocking. Returns number of bytes enqueued. */
void udp_log_enqueue(const char *data, int len)
{
    if (!data || len <= 0) return;

    uint32_t wrote = 0;
    for (int i = 0; i < len; i++) {
        __disable_irq();
        uint32_t next = (log_rb_head + 1) & (LOG_RINGBUF_SIZE - 1);
        if (next == log_rb_tail) {
            /* buffer full: drop remaining bytes */
            log_dropped += (uint32_t)(len - i);
            __enable_irq();
            break;
        }
        log_ringbuf[log_rb_head] = (uint8_t)data[i];
        log_rb_head = next;
        __enable_irq();
        wrote++;
    }
}

/* Helper: number of bytes available in ring buffer */
static uint32_t log_rb_avail(void)
{
    if (log_rb_head >= log_rb_tail) return log_rb_head - log_rb_tail;
    return LOG_RINGBUF_SIZE - (log_rb_tail - log_rb_head);
}

/* Helper: copy up to 'n' bytes starting from tail into dest; return actual copied */
static uint32_t log_rb_peek(uint8_t *dest, uint32_t n)
{
    uint32_t avail = log_rb_avail();
    if (avail == 0) return 0;
    uint32_t tocopy = (n < avail) ? n : avail;

    uint32_t first = LOG_RINGBUF_SIZE - log_rb_tail;
    if (first >= tocopy) {
        memcpy(dest, &log_ringbuf[log_rb_tail], tocopy);
    } else {
        memcpy(dest, &log_ringbuf[log_rb_tail], first);
        memcpy(dest + first, &log_ringbuf[0], tocopy - first);
    }
    return tocopy;
}

/* Helper: advance tail by n bytes after sending */
static void log_rb_advance(uint32_t n)
{
    uint32_t avail = log_rb_avail();
    if (n >= avail) {
        log_rb_tail = log_rb_head; // consumed all
    } else {
        log_rb_tail = (log_rb_tail + n) & (LOG_RINGBUF_SIZE - 1);
    }
}

/* Helper: find first newline ('\n') in the ring buffer; returns distance from tail
   (0..avail-1) if found, or UINT32_MAX if not found. */
static uint32_t log_rb_find_newline(void)
{
    uint32_t avail = log_rb_avail();
    if (avail == 0) return UINT32_MAX;

    uint32_t idx = log_rb_tail;
    for (uint32_t i = 0; i < avail; i++) {
        if (log_ringbuf[idx] == '\n') return i; // distance from tail
        idx = (idx + 1) & (LOG_RINGBUF_SIZE - 1);
    }
    return UINT32_MAX;
}

/* Process buffered logs: send exactly one newline-terminated line per UDP datagram.
   Called periodically from the main loop. It will attempt to send up to
   MAX_LINES_PER_CALL lines per invocation to avoid long blocking in case of
   heavy backlog. */
void udp_log_process(void)
{
    if (!log_pcb) return; // no peer known

    const unsigned MAX_LINES_PER_CALL = 8;
    const uint16_t CHUNK_MAX = 1024; // max line size we'll send
    uint8_t tmp[CHUNK_MAX];

    unsigned sent_lines = 0;
    while (sent_lines < MAX_LINES_PER_CALL) {
        uint32_t nl_off = log_rb_find_newline();
        if (nl_off == UINT32_MAX) break; // no complete line available

        uint32_t line_len = nl_off + 1; // include '\n'
        if (line_len > CHUNK_MAX) {
            // extremely long line: truncate to CHUNK_MAX and include newline if possible
            line_len = CHUNK_MAX;
        }

        /* copy the line into tmp */
        uint32_t first = LOG_RINGBUF_SIZE - log_rb_tail;
        if (first >= line_len) {
            memcpy(tmp, &log_ringbuf[log_rb_tail], line_len);
        } else {
            memcpy(tmp, &log_ringbuf[log_rb_tail], first);
            memcpy(tmp + first, &log_ringbuf[0], line_len - first);
        }

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)line_len, PBUF_POOL);
        if (!p) {
            DBG("[LOG] pbuf_alloc FAIL\r\n");
            break; // don't advance tail; try again later
        }
        memcpy(p->payload, tmp, line_len);
        err_t er = udp_sendto(log_pcb, p, &bcast_addr, LOG_PORT);
        DBGf("LOG send", er, (unsigned)line_len);
        pbuf_free(p);

        log_rb_advance(line_len);
        sent_lines++;
    }
}
