/**
 * @file lidar_parser.c
 * @brief FHL-LD20 LiDAR packet parser — implementation.
 *
 * Portable C11.  No HAL calls, no dynamic allocation in the hot path,
 * no floating-point.  Compiles unchanged on Linux and ESP32-C6.
 *
 * ── Packet layout (47 bytes) ─────────────────────────────────────────────
 *
 *  Byte(s)  Field
 *  ───────  ─────────────────────────────────────────────────────────────
 *   0        SOF = 0x54
 *   1        LEN = 0x2C  (fixed; identifies 12-point packet variant)
 *   2–3      Radar speed, °/s, uint16 little-endian
 *   4–5      Start angle, ×0.01°, uint16 little-endian
 *   6–41     12 × point { distance_mm: uint16 LE, intensity: uint8 }
 *  42–43     End angle,   ×0.01°, uint16 little-endian
 *  44–45     Timestamp, ms mod 30 000, uint16 little-endian
 *  46        CRC8(poly=0x4D, init=0x00) over bytes 0–45
 *
 * ── State machine ────────────────────────────────────────────────────────
 *
 *  WAIT_SOF ──(0x54)──▶ WAIT_LEN ──(0x2C)──▶ READ_BODY ──(46 bytes)──▶
 *  VERIFY_CRC ──(ok)──▶ WAIT_SOF
 *               │                   │                        │
 *            (other)            (other)                  (bad CRC)
 *               │                   │                        │
 *           WAIT_SOF            WAIT_SOF                WAIT_SOF
 *
 * ── Revolution detection ─────────────────────────────────────────────────
 *
 *  The FHL-LD20 always sweeps in one direction (increasing angle).
 *  Consecutive packets are contiguous: packet[i].end_angle == packet[i+1].
 *  start_angle (to within 1–2 cdeg of sensor firmware rounding).
 *
 *  Because consecutive start and end angles are equal (not overlapping),
 *  comparing new_start_angle < prev_end_angle never fires on contiguous
 *  data — the equality makes the strict less-than false.
 *
 *  The correct criterion compares consecutive START angles:
 *
 *    boundary := new_start_angle < prev_start_angle
 *
 *  Within a revolution start angles increase monotonically.  The only
 *  time a new start angle is less than the previous start angle is when
 *  the beam has wrapped through 0°.  This is both necessary and sufficient,
 *  requires no threshold, and works correctly whether packets are exactly
 *  contiguous or have small gaps/overlaps due to sensor firmware variation.
**/
#include "../hal/hal.h"
#include "lidar_parser.h"

#include <string.h>   /* memset           */
#include <stddef.h>   /* NULL             */
#include <stdlib.h>   /* malloc, free     */
#include <stdio.h>

/* =========================================================================
* CRC-8 look-up table
*
* Polynomial : 0x4D  (confirmed against documented example packet)
* Init       : 0x00
* RefIn      : false
* RefOut     : false
* XorOut     : 0x00
*
* The table is computed once at runtime on first use and stored in BSS.
* On ESP32-C6 BSS is in DRAM (256 bytes).  If RAM is critical the table
* can be replaced with the bit-loop directly — 8 iterations × 1 XOR each,
* still fast enough at 47 bytes per packet at 10 Hz.
* ====================================================================== */

/*
* Because pure C11 has no constexpr, we use a runtime-initialised table
* that is filled on the first call to crc8_compute().  On embedded targets
* this happens once during boot and the table is never regenerated.
*
* The table is a plain file-scope array (not heap-allocated) so it costs
* exactly 256 bytes of BSS on ESP32 (zeroed by the startup code) and is
* populated on first use.  The `volatile` write barrier is unnecessary on
* single-threaded embedded; on Linux it is harmless.
*/
static const uint8_t g_crc_table[256] = {
   0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,
   0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
   0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,
   0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
   0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,
   0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
   0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,
   0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
   0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,
   0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
   0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,
   0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
   0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,
   0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
   0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,
   0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
   0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,
   0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
   0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,
   0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
   0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,
   0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
   0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,
   0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
   0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,
   0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
   0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,
   0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
   0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,
   0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
   0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,
   0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
};

/**
 * @brief Compute CRC-8 (poly=0x4D, init=0x00) over @p len bytes.
 *
 * Table-driven: O(n) with a single XOR + table lookup per byte.
 * No branches inside the loop after the first call.
**/
static uint8_t crc8_compute(const uint8_t *data, uint8_t len) {
   uint8_t crc = 0x00;
   for(uint8_t i = 0; i < len; i++) {
      crc = g_crc_table[crc ^ data[i]];
   }
   return crc;
}

/* =========================================================================
* Parser state machine
* ====================================================================== */

/** Internal parser states. */
typedef enum {
   S_WAIT_SOF,   /**< Scanning for 0x54.                              */
   S_WAIT_LEN,   /**< Have SOF; expecting 0x2C.                       */
   S_READ_BODY,  /**< Accumulating all 45 body and CRC bytes.         */
} ParserState;

/**
 * @brief Full (non-opaque) definition of LidarParser.
 *
 * Kept in the .c file — callers only ever hold a pointer.
**/
struct LidarParser {
   /* ── state machine ── */
   ParserState state;

   /* ── raw packet buffer ──
   * Holds the current in-progress packet, byte by byte.
   * SOF and LEN are stored at [0] and [1], then body bytes fill [2..46].
   */
   uint8_t  pkt[LD20_PKT_SIZE];  /* 47 bytes                           */
   uint8_t  pkt_pos;             /* next write index into pkt[]        */

   /* ── scan accumulation buffer ──
   * Points from accepted packets are appended here until a revolution
   * boundary is detected, at which point the buffer is snapshotted and
   * handed to the caller.
   */
   LidarScan wip;           /**< Work-in-progress scan.                */

   /* ── inter-packet state for revolution detection ── */
   uint16_t prev_start_angle;  /**< Start angle of the last accepted packet. Initialised to 0xFFFF (invalid). */
   bool     first_packet;    /**< True until the first valid packet. */
   uint16_t last_timestamp_ms; /**< Timestamp from the last decoded packet. */

   /* ── speed accumulation for averaging ── */
   uint32_t speed_sum_dps;  /**< Sum of per-packet speeds this rev.    */
   uint16_t speed_count;    /**< Number of packets summed.             */

   /* ── diagnostics ── */
   ParserStats stats;
};

/* =========================================================================
* Static (embedded) instance pool
* ====================================================================== */

#ifdef EMBEDDED
static struct LidarParser g_parser_pool;
#endif

/* =========================================================================
* Packet byte-field offsets  (named constants, not magic numbers)
* ====================================================================== */

#define OFF_SOF          0u
#define OFF_LEN          1u
#define OFF_SPEED_L      2u
#define OFF_SPEED_H      3u
#define OFF_START_ANG_L  4u
#define OFF_START_ANG_H  5u
#define OFF_POINTS       6u          /* 36 bytes: 12 × (u16 dist + u8 int) */
#define OFF_END_ANG_L   42u
#define OFF_END_ANG_H   43u
#define OFF_TS_L        44u
#define OFF_TS_H        45u
#define OFF_CRC         46u

/* =========================================================================
* Internal helpers
* ====================================================================== */

/** Read a uint16 little-endian from a packet byte slice. */
static inline uint16_t u16le(const uint8_t *p) {
   return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief Interpolate the bearing for point index @p i within a packet.
 *
 * The sensor reports a start and end angle for each 12-point packet.
 * Individual point angles are linearly interpolated between the two.
 *
 * Wraparound (end_angle < start_angle, i.e. packet crosses 0°) is handled
 * by temporarily extending end_angle beyond 35999 before interpolation,
 * then wrapping the result back into [0, 35999].
 *
 * All arithmetic is integer; no float, no trig.
 *
 * @param start   Start angle in centidegrees [0, 35999].
 * @param end     End   angle in centidegrees [0, 35999].
 * @param i       Point index [0, 11].
 * @return        Interpolated angle in centidegrees [0, 35999].
**/
static uint16_t interpolate_angle(uint16_t start, uint16_t end, uint8_t i) {
   /* Extend end past 36000 if the packet crosses the 0° boundary. */
   uint32_t a_start = start;
   uint32_t a_end   = (end >= start) ? end : (uint32_t)end + 36000u;

   /*
   * Linear interpolation across 11 equal steps (indices 0 to 11).
   *
   *   angle[i] = start + i * (end - start) / 11
   *
   * Integer division is used throughout.  The maximum quantisation
   * error is < 0.01° per point, well within the sensor's angular
   * resolution.
   */
   uint32_t angle = a_start + (uint32_t)i * (a_end - a_start) / 11u;

   /* Wrap back into [0, 35999]. */
   while(angle >= 36000u) angle -= 36000u;

   return (uint16_t)angle;
}

/**
 * @brief Decode one validated 47-byte packet into the WIP scan.
 *
 * Called only after CRC passes.  Appends up to 12 LidarPoints into
 * p->wip.  If the accumulation buffer is full, increments the overflow
 * counter and silently drops the excess points (an anomaly that should
 * not occur at normal rotation speeds).
 *
 * @param p    Parser instance.
**/
static void decode_packet(struct LidarParser *p) {
   const uint8_t *pkt = p->pkt;

   uint16_t start_angle  = u16le(&pkt[OFF_START_ANG_L]);
   uint16_t end_angle    = u16le(&pkt[OFF_END_ANG_L]);
   uint16_t speed_dps    = u16le(&pkt[OFF_SPEED_L]);
   uint16_t timestamp_ms = u16le(&pkt[OFF_TS_L]);

   /* Accumulate speed for averaging across the revolution. */
   p->speed_sum_dps += speed_dps;
   p->speed_count++;

   /* Remember timestamp for boundary snapshot. */
   p->last_timestamp_ms = timestamp_ms;

   /* Append each of the 12 points into the WIP buffer. */
   for(uint8_t i = 0; i < LD20_POINTS_PER_PKT; i++) {
      if(p->wip.count >= LIDAR_MAX_POINTS) {
         p->stats.points_overflow++;
         break;
      }

      uint8_t off = (uint8_t)(OFF_POINTS + i * 3u);
      uint16_t dist_mm = u16le(&pkt[off]);
      uint8_t  inten   = pkt[off + 2u];

      uint16_t angle = interpolate_angle(start_angle, end_angle, i);

      LidarPoint *pt  = &p->wip.points[p->wip.count];
      pt->angle_cdeg  = angle;
      pt->distance_mm = dist_mm;
      pt->intensity   = inten;
      p->wip.count++;
   }

   /* Record this packet's start angle for the next revolution boundary check.
   * We track start-to-start, not end-to-start, because the
   * FHL-LD20 produces exactly contiguous packets where end[i] == start[i+1].
   * Comparing new_start < prev_end would be 0 < 0 at the 0° wrap when
   * the division is exact, making the boundary invisible.              */
   p->prev_start_angle = start_angle;
}

/**
 * @brief Copy WIP scan to @p out and reset the WIP buffer.
 *
 * @param p      Parser instance.
 * @param out    Caller-owned destination.
 * @param ts_ms  Timestamp to attach (from HAL or 0 when unavailable).
**/
static void snapshot_scan(
   struct LidarParser *p, LidarScan *out, uint32_t ts_ms
) {
   *out = p->wip;
   out->timestamp_ms = ts_ms;
   out->speed_dps    = (p->speed_count > 0u)
      ? (uint16_t)(p->speed_sum_dps / p->speed_count)
      : 0u;

   p->wip.count        = 0u;
   p->wip.timestamp_ms = 0u;
   p->wip.speed_dps    = 0u;
   p->speed_sum_dps    = 0u;
   p->speed_count      = 0u;

   p->stats.scans_completed++;
}

/* =========================================================================
* Internal state-machine reset (does not touch stats)
* ====================================================================== */

static void reset_sm(struct LidarParser *p) {
   p->state             = S_WAIT_SOF;
   p->pkt_pos           = 0;
   p->prev_start_angle  = 0xFFFFu; /* sentinel: "no previous packet yet" */
   p->first_packet      = true;
   p->last_timestamp_ms = 0;
   p->wip.count         = 0;
   p->wip.timestamp_ms  = 0;
   p->wip.speed_dps     = 0;
   p->speed_sum_dps     = 0;
   p->speed_count       = 0;
}

/* =========================================================================
* Public API — lifecycle
* ====================================================================== */

LidarParser *lidar_parser_create(void) {
#ifdef EMBEDDED
   struct LidarParser *p = &g_parser_pool;
#else
   struct LidarParser *p = (struct LidarParser *)malloc(sizeof(struct LidarParser));
   if(!p) return NULL;
#endif

   memset(p, 0, sizeof(*p));
   reset_sm(p);
   return p;
}

void lidar_parser_destroy(LidarParser *p) {
#ifndef EMBEDDED
   free(p);
#else
   (void)p;   /* static instance: nothing to free */
#endif
}

void lidar_parser_reset(LidarParser *p) {
   if(!p) return;
   /* Preserve diagnostics across a soft reset. */
   ParserStats saved = p->stats;
   reset_sm(p);
   p->stats = saved;
}

/* =========================================================================
* Public API — core feed function
* ====================================================================== */

ParseResult lidar_parser_feed(
   LidarParser *p, const uint8_t *data, int len, LidarScan *out_scan
) {
   ParseResult result = PARSE_NEED_MORE_DATA;

   // int x = 0;
   // printf("logging: BUFFER :=");
   // while(x < len) printf(" 0x%02X", data[x++]);
   // printf("\n");

   for(int i = 0; i < len; i++) {
      uint8_t byte = data[i];
      p->stats.bytes_consumed++;

      switch(p->state) {
         /* ── Wait for start-of-frame 0x54 ─────────────────────────── */
         case S_WAIT_SOF:
            if(byte == LD20_SOF) {
               p->pkt[OFF_SOF] = byte;
               p->pkt_pos      = 1;
               p->state        = S_WAIT_LEN;
            }
            /* Any other byte is silently discarded — garbage between
            * packets is normal at stream start or after an error.    */
            break;

         /* ── Validate data-length byte 0x2C ───────────────────────── */
         case S_WAIT_LEN:
            if(byte == LD20_DATA_LEN) {
               p->pkt[OFF_LEN] = byte;
               p->pkt_pos      = 2;
               p->state        = S_READ_BODY;
            } else {
               // printf("logging: FRAME_ERROR : expected 0x%02X, got 0x%02X\n", LD20_DATA_LEN, byte);

               /*
               * Not 0x2C.  The sensor only uses one packet format so
               * any other length is a framing error.  Discard the
               * tentative SOF and try the current byte as a new SOF.
               */
               p->stats.packets_frame_err++;
               result = PARSE_FRAME_ERROR;
               p->state = S_WAIT_SOF;
               if(byte == LD20_SOF) {
                  /* Current byte might be a real SOF — re-enter. */
                  p->pkt[OFF_SOF] = byte;
                  p->pkt_pos      = 1;
                  p->state        = S_WAIT_LEN;
               }
            }
            break;

         /* ── Accumulate body bytes and CRC (2..46) ────────────────── */
         case S_READ_BODY:
            p->pkt[p->pkt_pos++] = byte;

            /* pkt_pos == LD20_PKT_SIZE means we just wrote the CRC byte
             * at index 46 and the complete 47-byte packet is buffered.  */
            if(p->pkt_pos == LD20_PKT_SIZE) {
               /* Verify CRC over bytes 0..45 (LD20_PKT_SIZE - 1 bytes). */
               uint8_t computed = crc8_compute(
                  p->pkt, (uint8_t)(LD20_PKT_SIZE - 1u)
               );

               if(computed != p->pkt[OFF_CRC]) {
                  /* Corrupt packet: discard, keep WIP scan intact. */
                  p->stats.packets_crc_err++;
                  result     = PARSE_CHECKSUM_ERROR;
                  p->state   = S_WAIT_SOF;
                  p->pkt_pos = 0u;
                  break;
               }

               // x = 0;
               // printf("logging: PKT :=");
               // while(x < (int)LD20_PKT_SIZE) printf(" 0x%02X", p->pkt[x++]);
               // printf("\n");

               /* Valid packet. */
               p->stats.packets_ok++;
               uint16_t start_angle = u16le(&p->pkt[OFF_START_ANG_L]);

               /*
               * Revolution boundary detection.
               *
               * Within a revolution the sensor's start_angle increases
               * monotonically.  The only time a new start_angle is strictly
               * less than the previous packet's start_angle is when the beam
               * has wrapped through 0°, completing one full revolution.
               *
               * We compare start-to-start (not start-to-end) because the
               * FHL-LD20 produces exactly contiguous packets:
               *   packet[i].end_angle == packet[i+1].start_angle
               * This means new_start == prev_end at the wrap point when
               * the span divides 36000 evenly, making new_start < prev_end
               * evaluate to 0 < 0 = false and missing the boundary entirely.
               * Comparing start angles is immune to this because start[i+1]
               * is always less than start[i] at the genuine 0° crossing.
               */
               if(!p->first_packet
               && start_angle < p->prev_start_angle
               && p->wip.count > 0u) {
                  snapshot_scan(p, out_scan, hal_time_ms());
                  result = PARSE_OK;
               }

               // printf("logging: START_ANGLE := %u, PREV_START_ANGLE := %u, wip.count := %u\n", start_angle, p->prev_start_angle, p->wip.count);

               p->first_packet = false;
               decode_packet(p);

               p->state   = S_WAIT_SOF;
               p->pkt_pos = 0u;
            }
            break;
      } /* switch */
   } /* for each byte */

   return result;
}

/* =========================================================================
* Public API — diagnostics
* ====================================================================== */

void lidar_parser_get_stats(const LidarParser *p, ParserStats *out) {
   if(!p || !out) return;
   *out = p->stats;
}

void lidar_parser_reset_stats(LidarParser *p) {
   if(!p) return;
   memset(&p->stats, 0, sizeof(p->stats));
}

int lidar_parser_flush(LidarParser *p, LidarScan *out) {
   if(!p || !out || p->wip.count == 0) return false;
   snapshot_scan(p, out, hal_time_ms());
   return true;
}
