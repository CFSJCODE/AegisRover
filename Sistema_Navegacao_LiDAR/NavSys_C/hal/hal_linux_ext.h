/**
 * @file hal_linux_ext.h
 * @brief Linux-only HAL extensions — replay source and raw capture.
 *
 * This header is intentionally SEPARATE from hal.h.
 * nav_core/ modules must NEVER include it.
 * Only app/linux/ and tools/ code includes this.
 *
 * Provides a virtual "serial port" backed by a pre-captured binary file,
 * enabling offline development and regression testing without hardware.
 */

#ifndef HAL_LINUX_EXT_H
#define HAL_LINUX_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hal.h"

/**
 * @brief Open a captured binary file as a virtual serial source.
 *
 * Bytes are returned by hal_serial_read_replay() exactly as if they
 * arrived from UART.  Playback is paced at @p baud bits/second using
 * nanosleep() so the parser receives data at a realistic rate.
 *
 * Pass @p baud = 0 to disable pacing (instant return — good for unit
 * tests where speed matters and timing is irrelevant).
 *
 * @param path  Path to a .bin file captured with tools/log_capture.py,
 *              or any raw byte stream in FHL-LD20 packet format.
 * @param baud  Playback pacing baud rate (e.g. 230400), or 0 = maximum.
 * @return      HalSerial handle, or NULL on failure.
 *
 * @note  The returned handle must be closed with hal_serial_close_replay(),
 *        NOT hal_serial_close().
 */
HalSerial *hal_serial_open_replay(const char *path, uint32_t baud);

/**
 * @brief Read from a replay source opened with hal_serial_open_replay().
 *
 * Drop-in replacement for hal_serial_read() for replay handles.
 * Returns file bytes with optional timing pacing.
 * Returns 0 at end-of-file (treat as "no more scans").
 *
 * @param s           Replay handle from hal_serial_open_replay().
 * @param buf         Destination buffer.
 * @param max_len     Maximum bytes to read.
 * @param timeout_ms  Ignored for replay; present for API compatibility.
 * @return            Bytes read, 0 = EOF, -1 = error.
 */
int hal_serial_read_replay(HalSerial *s, uint8_t *buf, int max_len,
                           uint32_t timeout_ms);

/**
 * @brief Close a replay handle and release resources.
 *
 * Must be used in place of hal_serial_close() for replay handles.
 *
 * @param s  Replay handle from hal_serial_open_replay(), or NULL.
 */
void hal_serial_close_replay(HalSerial *s);

#ifdef __cplusplus
}
#endif

#endif /* HAL_LINUX_EXT_H */
