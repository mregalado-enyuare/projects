// udp_server.h
#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdint.h>

// UDP port we listen on for ASCII commands
#define UDP_SERVER_PORT 9000

/**
 * @brief  Initialize the UDP “GOTO” server.
 * @param  board_id  Which group of 16 motors this board owns (0..2).
 */
void udp_server_init(uint8_t board_id);

void udp_async_pump(void);

/*
 * Send raw log data (not JSON) to the last-known peer via UDP
 * if available. This allows rerouting printf/_write output to Ethernet.
 * The function is safe to call from application code; it will silently
 * do nothing if no peer is known yet.
 */
void udp_log_send(const char *data, int len);

/*
 * Non-blocking logging API:
 * - `udp_log_enqueue` is safe to call from any context and will copy bytes
 *   into a ring buffer and return immediately (may drop bytes if buffer is
 *   full).
 * - `udp_log_process` must be called periodically from the main loop (or a
 *   dedicated task) to send buffered logs over UDP. It is non-blocking in the
 *   sense that it doesn't block _write; it may still call lwIP UDP send.
 */
void udp_log_enqueue(const char *data, int len);
void udp_log_process(void);

#endif // UDP_SERVER_H