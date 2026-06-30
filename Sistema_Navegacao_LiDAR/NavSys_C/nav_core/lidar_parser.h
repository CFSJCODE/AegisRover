/**
 * @file lidar_parser.h
 * @brief FHL-LD20 LiDAR packet parser — public interface.
 *
 * Converts a raw UART byte stream into complete, validated LidarScan
 * structs.  The parser is a pure state machine: it holds no I/O handles,
 * calls no HAL functions, and performs no dynamic allocation in the hot
 * path.  It is safe to call from any thread or FreeRTOS task that owns
 * the LidarParser handle.
 *
 * ### Packet format (FHL-LD20 / LD06 compatible)
 *
 *  Offset  Size  Field
 *  ------  ----  -----
 *   0       1    SOF  = 0x54
 *   1       1    LEN  = 0x2C  (constant; marks 12-point packet)
 *   2–3     2    Radar speed, °/s, little-endian
 *   4–5     2    Start angle, ×0.01°, little-endian
 *   6–41   36    12 × (distance_mm u16 LE + intensity u8)
 *  42–43    2    End angle, ×0.01°, little-endian
 *  44–45    2    Timestamp, ms mod 30000, little-endian
 *  46       1    CRC8 (poly=0x4D, init=0x00) of bytes 0–45
 *
 * Total: 47 bytes per packet, 12 points per packet.
 *
 * ### Revolution detection
 *
 * A complete 360° scan is signalled when the parser observes a start angle
 * that is less than the previous packet's end angle (the angle wrapped
 * through 0°).  The completed scan is written to the caller's LidarScan
 * buffer and PARSE_OK is returned.
 *
 * ### Usage
 *
 * @code
 *   LidarParser *p = lidar_parser_create();
 *   LidarScan    scan;
 *   uint8_t      buf[256];
 *
 *   while (running) {
 *       int n = hal_serial_read(port, buf, sizeof(buf), 100);
 *       if (n > 0) {
 *           ParseResult r = lidar_parser_feed(p, buf, n, &scan);
 *           if (r == PARSE_OK) {
 *               // use scan
 *           }
 *       }
 *   }
 *   lidar_parser_destroy(p);
 * @endcode
**/

#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "nav_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
* Protocol constants
* ====================================================================== */

#define LD20_SOF            0x54u   /**< Start-of-frame byte.             */
#define LD20_DATA_LEN       0x2Cu   /**< Always 44 — identifies 12-pt pkt */
#define LD20_PKT_SIZE       47u     /**< Total bytes per packet.          */
#define LD20_POINTS_PER_PKT 12u     /**< Measurement points per packet.   */
#define LD20_CRC_POLY       0x4Du   /**< CRC8 polynomial.                 */
#define LD20_ANGLE_MAX_CDEG 35999u  /**< Maximum valid angle (359.99°).   */
#define LD20_TS_WRAP_MS     30000u  /**< Timestamp wraps at 30 000 ms.    */

/* =========================================================================
* Result codes
* ====================================================================== */

/**
 * @brief Return code from lidar_parser_feed().
**/
typedef enum {
   PARSE_OK             = 0,  /**< A complete 360° scan is ready.           */
   PARSE_NEED_MORE_DATA = 1,  /**< Packet incomplete; call again with more. */
   PARSE_CHECKSUM_ERROR = 2,  /**< CRC mismatch; packet discarded.          */
   PARSE_FRAME_ERROR    = 3,  /**< Unexpected LEN byte; re-syncing.         */
} ParseResult;

/* =========================================================================
* Diagnostics
* ====================================================================== */

/**
 * @brief Cumulative diagnostic counters.
 *
 * Monotonically increasing since the parser was created or last reset.
 * Useful for detecting noisy cables or baud-rate mismatches.
**/
typedef struct {
   uint32_t packets_ok;        /**< Packets accepted with valid CRC.        */
   uint32_t packets_crc_err;   /**< Packets rejected due to CRC failure.    */
   uint32_t packets_frame_err; /**< Frames rejected due to bad LEN byte.    */
   uint32_t scans_completed;   /**< Complete 360° scans delivered.          */
   uint32_t bytes_consumed;    /**< Raw bytes processed (including garbage) */
   uint32_t points_overflow;   /**< Times scan buffer was full mid-rev.     */
} ParserStats;

/* =========================================================================
* Opaque parser handle
* ====================================================================== */

/**
 * @brief Opaque parser state.
 *
 * Allocate with lidar_parser_create(); free with lidar_parser_destroy().
 * On embedded builds (-DEMBEDDED) a single static instance is used
 * instead of heap allocation.
**/
typedef struct LidarParser LidarParser;

/* =========================================================================
* Lifecycle
* ====================================================================== */

/**
 * @brief Allocate and initialise a new parser instance.
 *
 * On Linux : heap-allocated via hal_calloc().
 * On ESP32 : returns a pointer to a single static instance
 *            (only one parser may exist at a time).
 *
 * @return  Non-NULL parser handle on success, NULL on allocation failure.
**/
LidarParser *lidar_parser_create(void);

/**
 * @brief Release resources held by a parser.
 *
 * On embedded builds this is a no-op (static instance, nothing to free).
 * Safe to call with NULL.
 *
 * @param p  Parser handle returned by lidar_parser_create().
**/
void lidar_parser_destroy(LidarParser *p);

/**
 * @brief Reset parser to its initial state without freeing memory.
 *
 * Use after a detected error condition (cable disconnect, CRC storm) to
 * discard in-progress state and re-synchronise on the next SOF byte.
 * Clears the in-progress scan accumulation buffer.
 * Does NOT reset the diagnostic counters — use lidar_parser_reset_stats().
 *
 * @param p  Non-NULL parser handle.
**/
void lidar_parser_reset(LidarParser *p);

/* =========================================================================
* Core feed function
* ====================================================================== */

/**
 * @brief Feed raw bytes into the parser.
 *
 * This function must be called with every byte read from the serial port.
 * It is not required to feed complete packets — partial feeds work correctly
 * because the parser is a state machine.
 *
 * @param p         Non-NULL parser handle.
 * @param data      Buffer of raw bytes from the UART.
 * @param len       Number of valid bytes in @p data.  Must be > 0.
 * @param out_scan  Output scan struct.  Written only when PARSE_OK is
 *                  returned.  The caller owns this memory; the parser does
 *                  not retain a reference to it after returning.
 *
 * @return  PARSE_OK             — @p out_scan contains a complete scan.
 *          PARSE_NEED_MORE_DATA — no scan ready yet; call again.
 *          PARSE_CHECKSUM_ERROR — a packet was discarded; feeding continues.
 *          PARSE_FRAME_ERROR    — framing error; parser re-syncs.
 *
 * @note  Only PARSE_OK means @p out_scan was written.  The caller must not
 *        read @p out_scan for any other return code.
 *
 * @note  The function processes all @p len bytes in a single call and
 *        returns the result of the LAST significant event.  If two packets
 *        arrive in one buffer and both complete a revolution (unlikely but
 *        possible at high feed rates), only the second is returned.  For
 *        robustness feed small buffers (≤256 bytes) at the natural read rate.
**/
ParseResult lidar_parser_feed(
   LidarParser *p, const uint8_t *data, int len, LidarScan *out_scan
);

/* =========================================================================
* Diagnostics
* ====================================================================== */

/**
 * @brief Copy the current diagnostic counters into @p out.
 *
 * @param p    Non-NULL parser handle.
 * @param out  Destination struct; must not be NULL.
**/
void lidar_parser_get_stats(const LidarParser *p, ParserStats *out);

/**
 * @brief Zero all diagnostic counters.
 *
 * @param p  Non-NULL parser handle.
**/
void lidar_parser_reset_stats(LidarParser *p);

/**
 * @brief Deliver any partially-accumulated scan held in the WIP buffer.
 *
 * Call once after the byte stream ends (file replay, sensor disconnect)
 * to retrieve the final incomplete revolution rather than discarding it.
 *
 * The WIP buffer is cleared after the call; the parser is ready to accept
 * a fresh stream without losing the revolution-boundary state.
 *
 * If the WIP buffer is empty, the function returns without writing to @p out.
 * The caller can check @p out->count to determine if a scan was delivered.
 *
 * @param p    Non-NULL parser handle.
 * @param out  Destination scan (caller-owned).
 */
int lidar_parser_flush(LidarParser *p, LidarScan *out);

#ifdef __cplusplus
}
#endif

#endif /* LIDAR_PARSER_H */
