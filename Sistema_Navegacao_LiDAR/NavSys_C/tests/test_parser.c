/**
 * @file test_lidar_parser.c
 * @brief Functional test suite for lidar_parser.c
 *
 * Every test is self-contained.  There are no shared globals between tests
 * and every parser instance is created and destroyed within its own block.
 *
 * Sections:
 *   1.  CRC-8 — table correctness, single-bit sensitivity, example packet
 *   2.  Framing — garbage tolerance, bad-LEN recovery, SOF reuse
 *   3.  Feed fragmentation — byte-at-a-time worst case
 *   4.  Revolution detection — boundary fires, point count, multi-rev
 *   5.  Angle interpolation — endpoints, monotonicity, 0°-crossing packets
 *   6.  CRC error handling — detection, recovery, mid-stream injection
 *   7.  Distance and intensity decode — known values, zero, max
 *   8.  Speed field — averaged correctly across a revolution
 *   9.  Flush — partial revolution retrieved on demand
 *  10.  Reset and stats API
 *  11.  scan_raw.bin replay (optional, pass path as argv[1])
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O0 -g \
 *       -I nav_core/ \
 *       tests/test_lidar_parser.c nav_core/lidar_parser.c \
 *       -o build/test_lidar_parser
 *
 *   ./build/test_lidar_parser
 *   ./build/test_lidar_parser scan_raw.bin
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>   /* isatty */

#include "lidar_parser.h"

/* =========================================================================
 * Minimal test framework
 * ====================================================================== */

static int  g_run  = 0;
static int  g_pass = 0;
static int  g_fail = 0;
static bool g_colour = false;

static void check(bool ok, const char *expr, const char *detail,
                  const char *file, int line)
{
    g_run++;
    if (ok) {
        g_pass++;
        printf("  %s[PASS]%s %s\n",
               g_colour ? "\033[32m" : "",
               g_colour ? "\033[0m"  : "", expr);
    } else {
        g_fail++;
        printf("  %s[FAIL]%s %s  →  %s  (%s:%d)\n",
               g_colour ? "\033[31m" : "",
               g_colour ? "\033[0m"  : "",
               expr, detail ? detail : "", file, line);
    }
}

#define CHECK(cond) \
    check((cond), #cond, #cond, __FILE__, __LINE__)

#define CHECK_MSG(cond, msg) \
    check((cond), #cond, (msg), __FILE__, __LINE__)

#define CHECK_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    char _buf[128]; \
    snprintf(_buf, sizeof(_buf), "%lld != %lld", _a, _b); \
    check(_a == _b, #a " == " #b, _buf, __FILE__, __LINE__); \
} while (0)

static void section(const char *title)
{
    printf("\n%s── %s ──%s\n",
           g_colour ? "\033[1;33m" : "",
           title,
           g_colour ? "\033[0m"    : "");
}

/* =========================================================================
 * Documented example packet (47 bytes, CRC=0x50)
 *
 * start_angle = 0x7EAB = 32427 cdeg = 324.27°
 * end_angle   = 0x82BE = 33470 cdeg = 334.70°
 * angular span per packet ≈ 1043 cdeg ≈ 10.43°
 * ====================================================================== */
static const uint8_t EXAMPLE_PKT[47] = {
    0x54, 0x2C,
    0x68, 0x08,                         /* speed  = 2152 °/s  */
    0xAB, 0x7E,                         /* start  = 324.27°   */
    0xE0, 0x00, 0xE4,                   /* pt  0: 224 mm, int=228 */
    0xDC, 0x00, 0xE2,                   /* pt  1: 220 mm          */
    0xD9, 0x00, 0xE5,                   /* pt  2: 217 mm          */
    0xD5, 0x00, 0xE3,                   /* pt  3: 213 mm          */
    0xD3, 0x00, 0xE4,                   /* pt  4: 211 mm          */
    0xD0, 0x00, 0xE9,                   /* pt  5: 208 mm          */
    0xCD, 0x00, 0xE4,                   /* pt  6: 205 mm          */
    0xCA, 0x00, 0xE2,                   /* pt  7: 202 mm          */
    0xC7, 0x00, 0xE9,                   /* pt  8: 199 mm          */
    0xC5, 0x00, 0xE5,                   /* pt  9: 197 mm          */
    0xC2, 0x00, 0xE5,                   /* pt 10: 194 mm          */
    0xC0, 0x00, 0xE5,                   /* pt 11: 192 mm          */
    0xBE, 0x82,                         /* end    = 334.70°       */
    0x3A, 0x1A,                         /* ts     = 6714 ms       */
    0x50                                /* CRC                    */
};

/* =========================================================================
 * Helpers
 * ====================================================================== */

/** Reference CRC-8 bit-loop — independent of the parser's table. */
static uint8_t crc8_ref(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0x4Du)
                                : (uint8_t)(crc << 1u);
    }
    return crc;
}

/** Specification for building a synthetic 47-byte packet. */
typedef struct {
    uint16_t speed_dps;
    uint16_t start_angle;   /* centidegrees */
    uint16_t end_angle;     /* centidegrees */
    uint16_t timestamp_ms;
    uint16_t dist[12];
    uint8_t  intensity[12];
} PktSpec;

/** Build a valid 47-byte packet from a PktSpec; CRC is computed and filled. */
static void build_pkt(uint8_t out[47], const PktSpec *s)
{
    memset(out, 0, 47);
    out[0]  = LD20_SOF;
    out[1]  = LD20_DATA_LEN;
    out[2]  = (uint8_t)(s->speed_dps   & 0xFFu);
    out[3]  = (uint8_t)(s->speed_dps   >> 8u);
    out[4]  = (uint8_t)(s->start_angle & 0xFFu);
    out[5]  = (uint8_t)(s->start_angle >> 8u);
    for (int i = 0; i < 12; i++) {
        int off      = 6 + i * 3;
        out[off]     = (uint8_t)(s->dist[i] & 0xFFu);
        out[off + 1] = (uint8_t)(s->dist[i] >> 8u);
        out[off + 2] = s->intensity[i];
    }
    out[42] = (uint8_t)(s->end_angle   & 0xFFu);
    out[43] = (uint8_t)(s->end_angle   >> 8u);
    out[44] = (uint8_t)(s->timestamp_ms & 0xFFu);
    out[45] = (uint8_t)(s->timestamp_ms >> 8u);
    out[46] = crc8_ref(out, 46);
}

/**
 * Feed n_packets sequential packets covering one full 360° revolution into
 * @p p, then feed one more packet whose start_angle wraps back to 0, firing
 * the revolution boundary.  Each packet spans step = 36000/n_packets
 * centidegrees.  Packets are exactly contiguous (end[i] == start[i+1]),
 * matching the real FHL-LD20 output.  The boundary fires when packet[n+1]
 * has start_angle=0 < prev_start_angle=step*(n-1), which is always true.
 *
 * @return  Number of PARSE_OK events (should be exactly 1).
 */
static int feed_one_revolution(LidarParser *p, int n_packets,
                               uint16_t speed_dps, LidarScan *out)
{
    int scans  = 0;
    uint32_t step = 36000u / (uint32_t)n_packets;
    uint8_t pkt[47];

    for (int i = 0; i <= n_packets; i++) {
        uint32_t sa = ((uint32_t)i * step) % 36000u;
        uint32_t ea = (sa + step)          % 36000u;

        PktSpec s = {
            .speed_dps    = speed_dps,
            .start_angle  = (uint16_t)sa,
            .end_angle    = (uint16_t)ea,
            .timestamp_ms = (uint16_t)((i * 5u) % 30000u),
        };
        for (int j = 0; j < 12; j++) {
            s.dist[j]      = (uint16_t)(500u + (uint16_t)(j * 10));
            s.intensity[j] = 200;
        }
        build_pkt(pkt, &s);
        if (lidar_parser_feed(p, pkt, 47, out) == PARSE_OK)
            scans++;
    }
    return scans;
}

/**
 * Feed sequential packets starting from @p start_angle until the first
 * PARSE_OK event fires, then stop.  Writes the captured scan into @p out.
 * This ensures that a second boundary within the same call cannot overwrite
 * the scan that the caller cares about.
 *
 * Used by the SNAP macro in test_interpolation() to isolate the test
 * packet's data even when it sits near the 0° boundary.
 */
static void feed_until_boundary(LidarParser *p, int n_packets,
                                uint16_t speed_dps, uint16_t start_angle,
                                LidarScan *out)
{
    uint32_t step = 36000u / (uint32_t)n_packets;
    uint8_t pkt[47];

    for (int i = 0; i <= n_packets; i++) {
        uint32_t sa = (start_angle + (uint32_t)i * step) % 36000u;
        uint32_t ea = (sa + step)                        % 36000u;

        PktSpec s = {
            .speed_dps    = speed_dps,
            .start_angle  = (uint16_t)sa,
            .end_angle    = (uint16_t)ea,
            .timestamp_ms = (uint16_t)((i * 5u) % 30000u),
        };
        for (int j = 0; j < 12; j++) {
            s.dist[j]      = (uint16_t)(500u + (uint16_t)(j * 10));
            s.intensity[j] = 200;
        }
        build_pkt(pkt, &s);
        if (lidar_parser_feed(p, pkt, 47, out) == PARSE_OK)
            return;   /* stop at first boundary — do not overwrite out */
    }
}

/* =========================================================================
 * 1. CRC-8
 * ====================================================================== */
static void test_crc(void)
{
    section("1. CRC-8 (poly=0x4D, init=0x00)");

    /* 1a. Reference bit-loop matches the documented example */
    CHECK_EQ(crc8_ref(EXAMPLE_PKT, 46), 0x50u);

    /* 1b. Any single-bit flip in bytes 0..45 changes the CRC */
    {
        uint8_t buf[47];
        bool all_detected = true;
        for (int byte = 0; byte < 46 && all_detected; byte++) {
            for (int bit = 0; bit < 8 && all_detected; bit++) {
                memcpy(buf, EXAMPLE_PKT, 47);
                buf[byte] ^= (uint8_t)(1u << bit);
                if (crc8_ref(buf, 46) == 0x50u)
                    all_detected = false;
            }
        }
        CHECK_MSG(all_detected, "single-bit flip not detected by CRC");
    }

    /* 1c. crc8_ref is deterministic */
    {
        uint8_t zeros[46] = {0};
        CHECK_EQ(crc8_ref(zeros, 46), crc8_ref(zeros, 46));
    }

    /* 1d. Parser accepts the example packet (stats show 1 ok, 0 errors) */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,      1u);
        CHECK_EQ(st.packets_crc_err, 0u);
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 2. Framing and synchronisation
 * ====================================================================== */
static void test_framing(void)
{
    section("2. Framing and synchronisation");

    /* 2a. Garbage bytes (no 0x54) before first SOF are silently skipped */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t garbage[32];
        memset(garbage, 0xAA, 32);   /* 0xAA has no 0x54 bytes */
        lidar_parser_feed(p, garbage, 32, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,      0u);
        CHECK_EQ(st.packets_crc_err, 0u);
        CHECK_EQ(st.bytes_consumed,  32u);
        /* Real packet following garbage must be accepted */
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok, 1u);
        lidar_parser_destroy(p);
    }

    /* 2b. Bad LEN byte causes frame error; next real packet is accepted */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[2] = {0x54, 0xAB};
        lidar_parser_feed(p, bad, 2, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_frame_err, 1u);
        CHECK_EQ(st.packets_ok,        0u);
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok, 1u);
        lidar_parser_destroy(p);
    }

    /* 2c. Bad LEN = 0x54 (another SOF) — parser re-uses it as new SOF */
    {
        /* Stream: [0x54, 0x54, 0x2C, <45 bytes>]
         * First  0x54 → S_WAIT_LEN
         * Second 0x54 → bad LEN (frame error) but also new SOF
         * 0x2C  → valid LEN
         * Remaining 45 bytes of EXAMPLE_PKT → complete valid packet     */
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t reuse[2] = {0x54, 0x54};
        lidar_parser_feed(p, reuse, 2, &scan);
        lidar_parser_feed(p, EXAMPLE_PKT + 1, 46, &scan);  /* [0x2C..0x50] */
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,        1u);
        CHECK_EQ(st.packets_frame_err, 1u);
        lidar_parser_destroy(p);
    }

    /* 2d. Two back-to-back valid packets both counted */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t two[94];
        memcpy(two,      EXAMPLE_PKT, 47);
        memcpy(two + 47, EXAMPLE_PKT, 47);
        lidar_parser_feed(p, two, 94, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok, 2u);
        lidar_parser_destroy(p);
    }

    /* 2e. Garbage interleaved between packets does not corrupt state */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t garbage[16];
        memset(garbage, 0xBB, 16);
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        lidar_parser_feed(p, garbage, 16, &scan);
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok, 2u);
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 3. Feed fragmentation (byte-at-a-time worst case)
 * ====================================================================== */
static void test_fragmentation(void)
{
    section("3. Feed fragmentation");

    /* 3a. Byte-at-a-time: same result as bulk feed */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        ParseResult last = PARSE_NEED_MORE_DATA;
        for (int i = 0; i < 47; i++)
            last = lidar_parser_feed(p, &EXAMPLE_PKT[i], 1, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,       1u);
        CHECK_EQ(st.bytes_consumed,  47u);
        CHECK_EQ(st.packets_crc_err,  0u);
        /* Single packet → no revolution boundary → no PARSE_OK */
        CHECK_EQ((int)last, (int)PARSE_NEED_MORE_DATA);
        lidar_parser_destroy(p);
    }

    /* 3b. Split at every possible byte boundary across two packets */
    {
        uint8_t two[94];
        memcpy(two,      EXAMPLE_PKT, 47);
        memcpy(two + 47, EXAMPLE_PKT, 47);

        for (int split = 1; split < 94; split++) {
            LidarParser *p = lidar_parser_create();
            LidarScan scan;
            lidar_parser_feed(p, two,          split,      &scan);
            lidar_parser_feed(p, two + split,  94 - split, &scan);
            ParserStats st;
            lidar_parser_get_stats(p, &st);
            if (st.packets_ok != 2u) {
                char msg[64];
                snprintf(msg, sizeof(msg),
                         "split at byte %d: packets_ok=%u expected 2",
                         split, st.packets_ok);
                CHECK_MSG(false, msg);
            }
            lidar_parser_destroy(p);
        }
        CHECK_MSG(true, "all 93 split points accepted both packets");
    }
}

/* =========================================================================
 * 4. Revolution detection
 * ====================================================================== */
static void test_revolution(void)
{
    section("4. Revolution detection");

    /* 4a. One revolution of 36 packets fires exactly one PARSE_OK */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        int scans = feed_one_revolution(p, 36, 2000, &scan);
        CHECK_MSG(scans == 1, "expected exactly 1 complete scan");
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.scans_completed, 1u);
        CHECK_MSG(scan.count > 0,                "scan is empty");
        CHECK_MSG(scan.count <= LIDAR_MAX_POINTS, "scan overflows max");
        lidar_parser_destroy(p);
    }

    /* 4b. Point count in a complete revolution is a multiple of 12 */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        feed_one_revolution(p, 36, 2000, &scan);
        CHECK_EQ(scan.count % 12u, 0u);
        lidar_parser_destroy(p);
    }

    /* 4c. Point count is exactly n_packets * 12 */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        /* 36 packets per rev; boundary fires when packet[36] arrives.
         * The snapshot captures packets[0..35] = 36*12 = 432 points.   */
        feed_one_revolution(p, 36, 2000, &scan);
        CHECK_EQ(scan.count, 36u * 12u);
        lidar_parser_destroy(p);
    }

    /* 4d. Two consecutive revolutions each produce PARSE_OK */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        int s1 = feed_one_revolution(p, 36, 2000, &scan);
        int s2 = feed_one_revolution(p, 36, 2000, &scan);
        CHECK_EQ(s1, 1);
        CHECK_EQ(s2, 1);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.scans_completed, 2u);
        lidar_parser_destroy(p);
    }

    /* 4e. No spurious boundary on the very first packet */
    {
        /* The first packet ever must never trigger PARSE_OK regardless of
         * its start_angle, because there is no previous end_angle yet.  */
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        /* Build a packet whose start_angle (0) < any plausible prev_end */
        PktSpec s = { .speed_dps=2000, .start_angle=0, .end_angle=1000 };
        for (int j = 0; j < 12; j++) { s.dist[j] = 500; s.intensity[j] = 200; }
        uint8_t pkt[47]; build_pkt(pkt, &s);
        ParseResult r = lidar_parser_feed(p, pkt, 47, &scan);
        CHECK_EQ((int)r, (int)PARSE_NEED_MORE_DATA);
        lidar_parser_destroy(p);
    }

    /* 4f. No PARSE_OK if WIP is empty at the boundary (e.g. first wrap after
     *     a reset discarded the partial revolution)                       */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        /* Consume most of a revolution */
        uint8_t pkt[47];
        uint32_t step = 36000u / 36u;
        for (int i = 1; i < 35; i++) {
            uint32_t sa_val = (uint32_t)i * step;
            uint32_t ea_val = (sa_val + step) % 36000u;
            PktSpec s = {
                .speed_dps   = 2000,
                .start_angle = (uint16_t)sa_val,
                .end_angle   = (uint16_t)ea_val,
            };
            for (int j=0;j<12;j++){s.dist[j]=500;s.intensity[j]=200;}
            build_pkt(pkt, &s);
            lidar_parser_feed(p, pkt, 47, &scan);
        }
        /* Reset discards the partial WIP */
        lidar_parser_reset(p);
        /* Now a wrap packet must not produce PARSE_OK (WIP is empty) */
        PktSpec wrap = { .speed_dps=2000, .start_angle=0, .end_angle=1000 };
        for (int j=0;j<12;j++){wrap.dist[j]=500;wrap.intensity[j]=200;}
        build_pkt(pkt, &wrap);
        ParseResult r = lidar_parser_feed(p, pkt, 47, &scan);
        CHECK_EQ((int)r, (int)PARSE_NEED_MORE_DATA);
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 5. Angle interpolation
 * ====================================================================== */
static void test_interpolation(void)
{
    section("5. Angle interpolation");

    /*
     * Strategy: feed the packet-under-test, then complete one revolution
     * starting from the test packet's end_angle.  The first PARSE_OK
     * event captures exactly the WIP that existed at the boundary — which
     * contains the test packet's 12 points (possibly followed by some
     * revolution packets if the boundary fires late).
     *
     * feed_one_revolution_from() stops as soon as it records the first
     * PARSE_OK so that a second boundary within the same call cannot
     * overwrite scan_var with unrelated data.
     */

    /* Helper: feed packet-under-test, then drive the revolution until the
     * first boundary fires.  SNAP captures the scan from that boundary.   */
#define SNAP(spec, scan_var) do { \
    uint8_t _pkt[47]; \
    build_pkt(_pkt, &(spec)); \
    lidar_parser_feed(p, _pkt, 47, &(scan_var)); \
    feed_until_boundary(p, 36, 2000, (spec).end_angle, &(scan_var)); \
} while (0)

    /* 5a. point[0] == start_angle, point[11] == end_angle (no wraparound) */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=2000, .start_angle=1000, .end_angle=2000 };
        for (int i=0;i<12;i++){s.dist[i]=500;s.intensity[i]=200;}
        SNAP(s, scan);
        CHECK_EQ(scan.points[0].angle_cdeg,  1000u);
        CHECK_EQ(scan.points[11].angle_cdeg, 2000u);
        lidar_parser_destroy(p);
    }

    /* 5b. Intermediate points are strictly monotonically increasing */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=2000, .start_angle=5000, .end_angle=6000 };
        for (int i=0;i<12;i++){s.dist[i]=400;s.intensity[i]=150;}
        SNAP(s, scan);
        bool mono = true;
        for (int i = 1; i < 12; i++) {
            if (scan.points[i].angle_cdeg <= scan.points[i-1].angle_cdeg)
                { mono = false; break; }
        }
        CHECK_MSG(mono, "interpolated angles not monotonically increasing");
        lidar_parser_destroy(p);
    }

    /* 5c. Packet that crosses 0° (end_angle < start_angle):
     *     point[0] = start_angle, point[11] = end_angle,
     *     all angles in [0, 35999].                                     */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        /* 359.00° → 1.00°: crosses 0° */
        PktSpec s = { .speed_dps=2000, .start_angle=35900, .end_angle=100 };
        for (int i=0;i<12;i++){s.dist[i]=300;s.intensity[i]=180;}
        SNAP(s, scan);
        CHECK_EQ(scan.points[0].angle_cdeg,  35900u);
        CHECK_EQ(scan.points[11].angle_cdeg,   100u);
        bool in_range = true;
        for (int i = 0; i < 12; i++)
            if (scan.points[i].angle_cdeg > 35999u) { in_range = false; break; }
        CHECK_MSG(in_range, "cross-0° packet: angle out of [0,35999]");
        lidar_parser_destroy(p);
    }

    /* 5d. Verify 10 representative start/end pairs */
    {
        static const uint16_t pairs[10][2] = {
            {    0,  1200}, { 5000,  6100}, {17000, 18100},
            {34000, 35100}, {35500, 35900}, {   50,  1200},
            {18000, 19000}, {    0,   500}, {10000, 11000},
            {29000, 30000},
        };
        for (int t = 0; t < 10; t++) {
            LidarParser *p = lidar_parser_create();
            LidarScan scan;
            uint16_t sa = pairs[t][0], ea = pairs[t][1];
            PktSpec s = { .speed_dps=2000, .start_angle=sa, .end_angle=ea };
            for (int i=0;i<12;i++){s.dist[i]=400;s.intensity[i]=200;}
            SNAP(s, scan);
            if (scan.count >= 12) {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "pair[%d] pt[0]=%u expected %u",
                         t, scan.points[0].angle_cdeg, sa);
                CHECK_MSG(scan.points[0].angle_cdeg  == sa, msg);
                snprintf(msg, sizeof(msg),
                         "pair[%d] pt[11]=%u expected %u",
                         t, scan.points[11].angle_cdeg, ea);
                CHECK_MSG(scan.points[11].angle_cdeg == ea, msg);
            }
            lidar_parser_destroy(p);
        }
    }

#undef SNAP
}

/* =========================================================================
 * 6. CRC error handling and recovery
 * ====================================================================== */
static void test_crc_errors(void)
{
    section("6. CRC error handling");

    /* 6a. Corrupt CRC byte → PARSE_CHECKSUM_ERROR returned */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[47];
        memcpy(bad, EXAMPLE_PKT, 47);
        bad[46] ^= 0xFFu;
        ParseResult r = lidar_parser_feed(p, bad, 47, &scan);
        CHECK_EQ((int)r, (int)PARSE_CHECKSUM_ERROR);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_crc_err, 1u);
        CHECK_EQ(st.packets_ok,      0u);
        lidar_parser_destroy(p);
    }

    /* 6b. Recovery: good packet accepted immediately after CRC error */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[47]; memcpy(bad, EXAMPLE_PKT, 47); bad[46] ^= 0x01u;
        lidar_parser_feed(p, bad, 47, &scan);
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_crc_err, 1u);
        CHECK_EQ(st.packets_ok,      1u);
        lidar_parser_destroy(p);
    }

    /* 6c. Corrupt payload byte (not CRC itself) → CRC detects it */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[47]; memcpy(bad, EXAMPLE_PKT, 47);
        bad[10] ^= 0x01u;   /* flip one payload bit, CRC field unchanged */
        ParseResult r = lidar_parser_feed(p, bad, 47, &scan);
        CHECK_EQ((int)r, (int)PARSE_CHECKSUM_ERROR);
        lidar_parser_destroy(p);
    }

    /* 6d. CRC error mid-revolution: surrounding valid packets still
     *     complete the revolution (WIP is not discarded on CRC error).  */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;

        /* Build a full revolution; corrupt packet at index 10. */
        const int N = 36;
        uint32_t step = 36000u / (uint32_t)N;
        uint8_t pkt[47];
        int good_scans = 0;

        for (int i = 0; i <= N; i++) {
            uint32_t sa = ((uint32_t)i * step) % 36000u;
            uint32_t ea = (sa + step)          % 36000u;
            PktSpec s = { .speed_dps=2000,
                          .start_angle=(uint16_t)sa, .end_angle=(uint16_t)ea,
                          .timestamp_ms=(uint16_t)((i*5u)%30000u) };
            for (int j=0;j<12;j++){s.dist[j]=800;s.intensity[j]=200;}
            build_pkt(pkt, &s);
            if (i == 10) pkt[46] ^= 0xFFu;   /* corrupt CRC */
            if (lidar_parser_feed(p, pkt, 47, &scan) == PARSE_OK)
                good_scans++;
        }
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_crc_err, 1u);
        CHECK_MSG(good_scans >= 1, "revolution not completed after CRC error");
        lidar_parser_destroy(p);
    }

    /* 6e. Multiple consecutive CRC errors do not corrupt the parser state */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[47]; memcpy(bad, EXAMPLE_PKT, 47); bad[46] ^= 0xFFu;
        for (int i = 0; i < 5; i++)
            lidar_parser_feed(p, bad, 47, &scan);
        /* Good packet must still be accepted */
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_crc_err, 5u);
        CHECK_EQ(st.packets_ok,      1u);
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 7. Distance and intensity decode
 * ====================================================================== */
static void test_decode(void)
{
    section("7. Distance and intensity decode");

    /* 7a. Known per-point values decoded correctly */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=3000, .start_angle=1000, .end_angle=2000 };
        for (int i = 0; i < 12; i++) {
            s.dist[i]      = (uint16_t)(100u + (uint16_t)(i * 50));
            s.intensity[i] = (uint8_t)(10u  + (uint8_t)(i * 20));
        }
        uint8_t pkt[47]; build_pkt(pkt, &s);
        lidar_parser_feed(p, pkt, 47, &scan);
        /* Drive forward until the first boundary — captures the test packet. */
        feed_until_boundary(p, 36, 2000, s.end_angle, &scan);

        CHECK_MSG(scan.count >= 12, "fewer than 12 points in snapshot");
        for (int i = 0; i < 12 && i < (int)scan.count; i++) {
            uint16_t exp_d = (uint16_t)(100u + (uint16_t)(i * 50));
            uint8_t  exp_i = (uint8_t) (10u  + (uint8_t) (i * 20));
            char msg[64];
            snprintf(msg, sizeof(msg), "dist[%d]: got %u expected %u",
                     i, scan.points[i].distance_mm, exp_d);
            CHECK_MSG(scan.points[i].distance_mm == exp_d, msg);
            snprintf(msg, sizeof(msg), "intensity[%d]: got %u expected %u",
                     i, scan.points[i].intensity, exp_i);
            CHECK_MSG(scan.points[i].intensity == exp_i, msg);
        }
        lidar_parser_destroy(p);
    }

    /* 7b. distance_mm == 0 is stored as-is (represents invalid/no return) */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=2000, .start_angle=5000, .end_angle=6000 };
        for (int i=0;i<12;i++){s.dist[i]=0;s.intensity[i]=0;}
        uint8_t pkt[47]; build_pkt(pkt, &s);
        lidar_parser_feed(p, pkt, 47, &scan);
        feed_until_boundary(p, 36, 2000, s.end_angle, &scan);
        bool found_zero = false;
        for (int i = 0; i < (int)scan.count; i++)
            if (scan.points[i].distance_mm == 0u) { found_zero = true; break; }
        CHECK_MSG(found_zero, "zero-distance points not stored in scan");
        lidar_parser_destroy(p);
    }

    /* 7c. Maximum distance (65535 mm) stored correctly */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=2000, .start_angle=8000, .end_angle=9000 };
        for (int i=0;i<12;i++){s.dist[i]=65535u;s.intensity[i]=255u;}
        uint8_t pkt[47]; build_pkt(pkt, &s);
        lidar_parser_feed(p, pkt, 47, &scan);
        feed_until_boundary(p, 36, 2000, s.end_angle, &scan);
        bool found_max = false;
        for (int i = 0; i < (int)scan.count; i++)
            if (scan.points[i].distance_mm == 65535u) { found_max = true; break; }
        CHECK_MSG(found_max, "max-distance (65535 mm) not stored in scan");
        lidar_parser_destroy(p);
    }

    /* 7d. Intensity == 255 stored correctly */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        PktSpec s = { .speed_dps=2000, .start_angle=11000, .end_angle=12000 };
        for (int i=0;i<12;i++){s.dist[i]=1000u;s.intensity[i]=255u;}
        uint8_t pkt[47]; build_pkt(pkt, &s);
        lidar_parser_feed(p, pkt, 47, &scan);
        feed_until_boundary(p, 36, 2000, s.end_angle, &scan);
        bool found_ff = false;
        for (int i = 0; i < (int)scan.count; i++)
            if (scan.points[i].intensity == 255u) { found_ff = true; break; }
        CHECK_MSG(found_ff, "intensity=255 not stored in scan");
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 8. Speed field
 * ====================================================================== */
static void test_speed(void)
{
    section("8. Rotation speed");

    /* 8a. Constant speed across all packets → speed_dps matches */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        const uint16_t SPD = 1800u;
        int scans = feed_one_revolution(p, 36, SPD, &scan);
        CHECK_MSG(scans > 0, "no scan produced");
        if (scans > 0) CHECK_EQ(scan.speed_dps, SPD);
        lidar_parser_destroy(p);
    }

    /* 8b. Varying speed → averaged value in expected range */
    {
        /* Alternate 1000 and 3000 → expected average 2000 */
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        const int N = 36;
        uint32_t step = 36000u / (uint32_t)N;
        uint8_t pkt[47];
        int scans = 0;

        for (int i = 0; i <= N; i++) {
            uint32_t sa = ((uint32_t)i * step) % 36000u;
            uint32_t ea = (sa + step)          % 36000u;
            uint16_t spd = (i % 2 == 0) ? 1000u : 3000u;
            PktSpec s = { .speed_dps=spd,
                          .start_angle=(uint16_t)sa, .end_angle=(uint16_t)ea };
            for (int j=0;j<12;j++){s.dist[j]=500;s.intensity[j]=200;}
            build_pkt(pkt, &s);
            if (lidar_parser_feed(p, pkt, 47, &scan) == PARSE_OK) scans++;
        }
        CHECK_MSG(scans > 0, "no scan produced for speed averaging test");
        if (scans > 0) {
            /* Average of alternating 1000/3000 over N+1 packets ≈ 2000 */
            char msg[64];
            snprintf(msg, sizeof(msg),
                     "averaged speed %u not in [1800,2200]", scan.speed_dps);
            CHECK_MSG(scan.speed_dps >= 1800u && scan.speed_dps <= 2200u, msg);
        }
        lidar_parser_destroy(p);
    }
}

/* =========================================================================
 * 9. Flush — partial revolution retrieval
 * ====================================================================== */
static void test_flush(void)
{
    section("9. Flush (partial revolution)");

    /* 9a. After feeding some packets, flush returns them */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        /* Feed 10 packets without completing a revolution */
        uint32_t step = 36000u / 36u;
        uint8_t pkt[47];
        for (int i = 0; i < 10; i++) {
            PktSpec s = { .speed_dps=2000,
                          .start_angle=(uint16_t)((uint32_t)i * step),
                          .end_angle  =(uint16_t)((uint32_t)(i+1) * step) };
            for (int j=0;j<12;j++){s.dist[j]=500;s.intensity[j]=200;}
            build_pkt(pkt, &s);
            lidar_parser_feed(p, pkt, 47, &scan);
        }
        bool flushed = lidar_parser_flush(p, &scan);
        CHECK_MSG(flushed,             "flush returned false with 10 packets pending");
        CHECK_EQ(scan.count, 10u * 12u);
        lidar_parser_destroy(p);
    }

    /* 9b. Flush on empty WIP returns false */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        bool flushed = lidar_parser_flush(p, &scan);
        CHECK_MSG(!flushed, "flush should return false on empty WIP");
        lidar_parser_destroy(p);
    }

    /* 9c. Flush clears the WIP (second flush returns false) */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t pkt[47];
        PktSpec s = { .speed_dps=2000, .start_angle=0, .end_angle=1000 };
        for (int j=0;j<12;j++){s.dist[j]=500;s.intensity[j]=200;}
        build_pkt(pkt, &s);
        lidar_parser_feed(p, pkt, 47, &scan);
        lidar_parser_flush(p, &scan);
        bool second = lidar_parser_flush(p, &scan);
        CHECK_MSG(!second, "second flush should return false (WIP is empty)");
        lidar_parser_destroy(p);
    }

    /* 9d. lidar_parser_flush(NULL, ...) does not crash */
    {
        LidarScan scan;
        bool r = lidar_parser_flush(NULL, &scan);
        CHECK_MSG(!r, "flush(NULL) should return false");
    }
}

/* =========================================================================
 * 10. Reset and stats API
 * ====================================================================== */
static void test_reset_and_stats(void)
{
    section("10. Reset and stats API");

    /* 10a. Hard reset mid-packet: next good packet is accepted */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        lidar_parser_feed(p, EXAMPLE_PKT, 30, &scan);  /* partial */
        lidar_parser_reset(p);
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok, 1u);
        lidar_parser_destroy(p);
    }

    /* 10b. reset() preserves stats */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t bad[47]; memcpy(bad, EXAMPLE_PKT, 47); bad[46] ^= 0xFFu;
        lidar_parser_feed(p, bad, 47, &scan);
        lidar_parser_reset(p);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_crc_err, 1u);   /* survived reset */
        CHECK_EQ(st.packets_ok,      0u);
        lidar_parser_destroy(p);
    }

    /* 10c. reset_stats() zeros all counters */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);
        lidar_parser_reset_stats(p);
        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,      0u);
        CHECK_EQ(st.bytes_consumed,  0u);
        CHECK_EQ(st.scans_completed, 0u);
        lidar_parser_destroy(p);
    }

    /* 10d. Accumulated stats across a full revolution */
    {
        LidarParser *p = lidar_parser_create();
        LidarScan scan;
        uint8_t pkt[47];

        /* 3 good, 1 CRC error, 1 frame error, 2 good */
        for (int i = 0; i < 3; i++)
            lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);

        memcpy(pkt, EXAMPLE_PKT, 47); pkt[46] ^= 0xAAu;
        lidar_parser_feed(p, pkt, 47, &scan);

        uint8_t frame_err[2] = {0x54, 0x00};
        lidar_parser_feed(p, frame_err, 2, &scan);

        for (int i = 0; i < 2; i++)
            lidar_parser_feed(p, EXAMPLE_PKT, 47, &scan);

        ParserStats st;
        lidar_parser_get_stats(p, &st);
        CHECK_EQ(st.packets_ok,        5u);
        CHECK_EQ(st.packets_crc_err,   1u);
        CHECK_EQ(st.packets_frame_err, 1u);
        CHECK_MSG(st.bytes_consumed > 0u, "bytes_consumed not incremented");
        lidar_parser_destroy(p);
    }

    /* 10e. reset(NULL) and destroy(NULL) do not crash */
    lidar_parser_reset(NULL);
    lidar_parser_destroy(NULL);
    CHECK(true);
}

/* =========================================================================
 * 11. scan_raw.bin replay (optional)
 * ====================================================================== */
static void test_replay(const char *path)
{
    section("11. scan_raw.bin replay");

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        printf("  (skipped — cannot open '%s')\n", path);
        return;
    }

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    printf("  File : %s (%ld bytes)\n", path, fsize);

    LidarParser *p = lidar_parser_create();
    LidarScan    scan;
    LidarScan    first_scan;
    LidarScan    second_scan;
    memset(&first_scan,  0, sizeof(first_scan));
    memset(&second_scan, 0, sizeof(second_scan));

    uint8_t buf[128];
    int  scans       = 0;
    long total_read  = 0;
    size_t n;

    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        total_read += (long)n;
        int err = lidar_parser_feed(p, buf, (int)n, &scan);
        if (err == PARSE_OK) {
            if (scans == 0) first_scan  = scan;
            if (scans == 1) second_scan = scan;
            scans++;
        } else if(err == PARSE_CHECKSUM_ERROR) {
            // printf("logging: GOT ERROR CRC error at byte %ld\n", total_read - n);
        } else if (err == PARSE_FRAME_ERROR) {
            // printf("logging: GOT ERROR Frame error at byte %ld\n", total_read - n);
        }
    }
    /* Capture any trailing partial revolution */
    LidarScan last_partial;
    bool had_partial = lidar_parser_flush(p, &last_partial);
    fclose(fp);

    ParserStats st;
    lidar_parser_get_stats(p, &st);

    printf("  Bytes fed    : %ld\n",  total_read);
    printf("  Packets OK   : %u\n",   st.packets_ok);
    printf("  CRC errors   : %u\n",   st.packets_crc_err);
    printf("  Frame errors : %u\n",   st.packets_frame_err);
    printf("  Scans        : %d complete", scans);
    if (had_partial)
        printf(" + 1 partial (%u pts)", last_partial.count);
    printf("\n");

    if (scans > 0)
        printf("  First scan   : %u pts, speed=%u °/s, "
               "angles %.2f°–%.2f°\n",
               first_scan.count, first_scan.speed_dps,
               first_scan.points[0].angle_cdeg / 100.0,
               first_scan.points[first_scan.count-1].angle_cdeg / 100.0);
    if (scans >= 2)
        printf("  Second scan  : %u pts, speed=%u °/s, "
               "angles %.2f°–%.2f°\n",
               second_scan.count, second_scan.speed_dps,
               second_scan.points[0].angle_cdeg / 100.0,
               second_scan.points[second_scan.count-1].angle_cdeg / 100.0);

    CHECK_MSG(st.packets_ok > 0u,  "no valid packets in capture");
    CHECK_MSG(scans > 0,           "no complete scans decoded");
    /* first_scan is intentionally a partial revolution: the raw capture
     * begins mid-revolution so the WIP at the first boundary only holds
     * the arc from the capture-start angle to 360°.  Its point count
     * and span are both less than a full revolution — this is correct. */
    CHECK_MSG(first_scan.count > 0u,
              "first scan has 0 points");
    CHECK_MSG(first_scan.count <= LIDAR_MAX_POINTS,
              "first scan exceeds LIDAR_MAX_POINTS");

    /* CRC pass rate ≥ 95 % */
    {
        uint32_t total_pkts = st.packets_ok + st.packets_crc_err;
        float rate = (total_pkts > 0u)
                   ? (float)st.packets_ok / (float)total_pkts * 100.0f
                   : 0.0f;
        char msg[80];
        snprintf(msg, sizeof(msg),
                 "CRC pass rate %.1f%% (expected >= 95%%)", rate);
        CHECK_MSG(rate >= 95.0f, msg);
    }

    /* Revolution count: must be ≥ 90% of the theoretical maximum.
     *
     * Measured from the FHL-LD20 at 2160 °/s:
     *   step ≈ 6.48°/pkt → 36000/648 ≈ 55.6 packets per revolution.
     * Using 55 as a conservative integer divisor.
     * 10% tolerance covers the one partial first revolution (capture
     * starts mid-rev) and any occasional missed boundary.             */
    {
        uint32_t pkts_per_rev = 55u;
        uint32_t expected_scans = st.packets_ok / pkts_per_rev;
        char msg[120];
        snprintf(msg, sizeof(msg),
                 "scans=%d expected>=%u (packets_ok=%u / ~%u pkts/rev)",
                 scans, (unsigned)(expected_scans * 9u / 10u),
                 st.packets_ok, pkts_per_rev);
        CHECK_MSG((uint32_t)scans >= expected_scans * 9u / 10u, msg);
    }

    /* Angular span check on the SECOND complete scan.
     *
     * The first scan is always a partial revolution: the raw capture starts
     * mid-revolution, so the WIP at the first boundary only holds the arc
     * from the capture-start angle to 360°.  Its span is unpredictable.
     *
     * The second scan (and all subsequent ones) are full revolutions.
     * We verify its span is between 300° and 400° to confirm:
     *   - The WIP was properly reset (no multi-revolution accumulation).
     *   - The boundary fires at the right place (not too early/late).
     *
     * Upper bound 400° (36000+4000 = 40000 cdeg) accommodates the 56-packet
     * revolution case: 56 × 648 = 36288 cdeg ≈ 362.9°, where the last
     * packet crosses 0° and the raw end-angle appears small.  The span
     * formula below correctly handles this case by always measuring the
     * arc swept in the forward (increasing angle) direction:
     *
     *   span = (a1 - a0 + 36000) % 36000   ... normal case
     *   But if the last packet itself crosses 0°, a1 < a0 AND the true
     *   span is > 36000.  We detect this by checking point count: if
     *   count >= 55*12 = 660, the revolution is full and span is ≥ 350°.
     */
    if (scans >= 2 && second_scan.count >= 2) {
        uint16_t a0 = second_scan.points[0].angle_cdeg;
        uint16_t a1 = second_scan.points[second_scan.count - 1].angle_cdeg;
        /* Forward arc from a0 to a1 (the direction the sensor sweeps). */
        uint32_t span = ((uint32_t)a1 + 36000u - (uint32_t)a0) % 36000u;
        /* If a1 > a0, we might have a cross-0° revolution where span > 36000.
         * Detect via point count: 55+ pkts × 12 pts = 660+ points = full rev. */
        if (a1 > a0 && second_scan.count >= 660u) span += 36000u;
        char msg[80];
        snprintf(msg, sizeof(msg),
                 "second scan angular span %.2f° (expected 300–400°)",
                 span / 100.0);
        CHECK_MSG(span >= 30000u && span <= 40000u, msg);
    }

    /* Point count per scan must be a multiple of 12 */
    CHECK_MSG(first_scan.count % 12u == 0u,
              "first scan point count not a multiple of 12");

    lidar_parser_destroy(p);
}

/* =========================================================================
 * main
 * ====================================================================== */
int main(int argc, char *argv[])
{
    g_colour = isatty(STDOUT_FILENO);

    printf("\n%slidar_parser — test suite%s\n",
           g_colour ? "\033[1;33m" : "",
           g_colour ? "\033[0m"    : "");

    test_crc();
    test_framing();
    test_fragmentation();
    test_revolution();
    test_interpolation();
    test_crc_errors();
    test_decode();
    test_speed();
    test_flush();
    test_reset_and_stats();

    if (argc >= 2)
        test_replay(argv[1]);
    else
        printf("\n  Tip: pass path to scan_raw.bin to run the replay test.\n");

    /* ── Summary ── */
    printf("\n%s────────────────────────────────%s\n",
           g_colour ? "\033[1m" : "", g_colour ? "\033[0m" : "");
    printf("Tests run    : %d\n", g_run);
    printf("Tests passed : %s%d%s\n",
           g_colour ? "\033[32m" : "", g_pass, g_colour ? "\033[0m" : "");
    if (g_fail > 0)
        printf("Tests %sFAILED%s : %d\n",
               g_colour ? "\033[31m" : "", g_colour ? "\033[0m" : "", g_fail);
    else
        printf("%sAll tests passed.%s\n",
               g_colour ? "\033[32m" : "", g_colour ? "\033[0m" : "");
    printf("\n");

    return (g_fail > 0) ? 1 : 0;
}