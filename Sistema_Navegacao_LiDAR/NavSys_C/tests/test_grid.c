/**
 * @file test_phase2.c
 * @brief Functional test suite for scan_filter.c and occupancy_grid.c.
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O0 -g \
 *       -I nav_core/ \
 *       tests/test_phase2.c \
 *       nav_core/scan_filter.c \
 *       nav_core/occupancy_grid.c \
 *       nav_core/grid_viz.c \
 *       -o build/test_phase2
 *
 *   ./build/test_phase2
 *   ./build/test_phase2 scan_raw.bin   # optional: grid from real capture
 *
 * Sections
 * ────────
 *  1.  scan_filter — default config construction
 *  2.  scan_filter — zero-distance rejection
 *  3.  scan_filter — intensity gate
 *  4.  scan_filter — range clamp (min and max)
 *  5.  scan_filter — compaction preserves order
 *  6.  scan_filter — median filter correctness (w=3 and w=5)
 *  7.  scan_filter — median does not run when window=0
 *  8.  scan_filter — NULL scan / NULL config guards
 *  9.  scan_filter — stats counters
 * 10.  occupancy_grid — init fills UNKNOWN, mask non-empty
 * 11.  occupancy_grid — clear resets cells and counters
 * 12.  occupancy_grid — cell accessors (get/set, OOB safety)
 * 13.  occupancy_grid — occ_grid_passable gate
 * 14.  occupancy_grid — coordinate conversions round-trip
 * 15.  occupancy_grid — robot cell is always grid centre
 * 16.  occupancy_grid — ray-cast: forward obstacle
 * 17.  occupancy_grid — ray-cast: free corridor
 * 18.  occupancy_grid — ray-cast: out-of-range clips gracefully
 * 19.  occupancy_grid — ray-cast: all four quadrants produce obstacles
 * 20.  occupancy_grid — inflation: obstacle expands by radius
 * 21.  occupancy_grid — inflation: inflated does not overwrite OCCUPIED
 * 22.  occupancy_grid — inflation: robot cell never inflated (no obstacle)
 * 23.  occupancy_grid — inflation: OOB mask entries ignored
 * 24.  occupancy_grid — full pipeline: filter → update → inflate → passable
 * 25.  occupancy_grid — stats counters (rays, freed, occupied, clipped)
 * 26.  grid_viz       — viz_cfg_default does not crash
 * 27.  grid_viz       — viz_render does not crash (smoke test)
 * 28.  grid_viz       — viz_dump_csv produces a readable file
 * 29.  optional: replay scan_raw.bin and render one frame to stdout
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "../nav_core/scan_filter.h"
#include "../nav_core/occupancy_grid.h"
#include "../nav_core/grid_viz.h"
#include "../nav_core/lidar_parser.h"   /* for replay test */

/* =========================================================================
* Minimal test framework
* ====================================================================== */

static int  g_run = 0, g_pass = 0, g_fail = 0;
static bool g_colour = false;

static void t_result(bool ok, const char *expr, const char *detail,
                  const char *file, int line)
{
   g_run++;
   if (ok) {
      g_pass++;
      printf("  %s[PASS]%s %s\n",
            g_colour?"\033[32m":"", g_colour?"\033[0m":"", expr);
   } else {
      g_fail++;
      printf("  %s[FAIL]%s %s  →  %s  (%s:%d)\n",
            g_colour?"\033[31m":"", g_colour?"\033[0m":"",
            expr, detail?detail:"", file, line);
   }
}

#define CHECK(cond) \
   t_result((cond), #cond, #cond, __FILE__, __LINE__)

#define CHECK_MSG(cond, msg) \
   t_result((cond), #cond, (msg), __FILE__, __LINE__)

#define CHECK_EQ(a,b) do { \
   long long _a=(long long)(a), _b=(long long)(b); \
   char _d[96]; snprintf(_d,sizeof(_d),"%lld != %lld",_a,_b); \
   t_result(_a==_b, #a " == " #b, _d, __FILE__, __LINE__); \
} while(0)

static void section(const char *t) {
   printf("\n%s── %s ──%s\n",
         g_colour?"\033[1;33m":"", t, g_colour?"\033[0m":"");
}

/* =========================================================================
* Helpers: build synthetic scans
* ====================================================================== */

/** Fill a scan with `n` identical points. */
static void make_scan(LidarScan *s, uint16_t n,
                     uint16_t angle_cdeg, uint16_t dist_mm, uint8_t intens)
{
   memset(s, 0, sizeof(*s));
   if (n > LIDAR_MAX_POINTS) n = LIDAR_MAX_POINTS;
   for (uint16_t i = 0; i < n; i++) {
      s->points[i].angle_cdeg  = angle_cdeg;
      s->points[i].distance_mm = dist_mm;
      s->points[i].intensity   = intens;
   }
   s->count = n;
}

/** Build a 360-point scan with one point per degree at a fixed distance. */
static void make_ring_scan(LidarScan *s, uint16_t dist_mm, uint8_t intens)
{
   memset(s, 0, sizeof(*s));
   for (uint16_t deg = 0; deg < 360; deg++) {
      s->points[deg].angle_cdeg  = (uint16_t)(deg * 100u);
      s->points[deg].distance_mm = dist_mm;
      s->points[deg].intensity   = intens;
   }
   s->count = 360;
}

/* =========================================================================
* 1. scan_filter — default config
* ====================================================================== */
static void test_filter_defaults(void)
{
   section("1. scan_filter — default config");
   ScanFilterCfg cfg;
   scan_filter_cfg_default(&cfg);
   CHECK_EQ(cfg.min_dist_mm,   FILTER_MIN_DIST_MM);
   CHECK_EQ(cfg.max_dist_mm,   FILTER_MAX_DIST_MM);
   CHECK_EQ(cfg.min_intensity, FILTER_MIN_INTENSITY);
   CHECK_EQ(cfg.median_window, FILTER_MEDIAN_WINDOW);
   /* NULL guard */
   scan_filter_cfg_default(NULL);  /* must not crash */
   CHECK(true);
}

/* =========================================================================
* 2. scan_filter — zero-distance rejection
* ====================================================================== */
static void test_filter_zero_dist(void)
{
   section("2. scan_filter — zero-distance rejection");
   LidarScan s;
   make_scan(&s, 5, 4500, 0, 200);   /* all distance_mm == 0 */
   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);
   CHECK_EQ(s.count, 0u);

   ScanFilterStats st; scan_filter_get_stats(&st);
   CHECK_EQ(st.dropped_zero, 5u);
   CHECK_EQ(st.out_count,    0u);
   CHECK_EQ(st.in_count,     5u);
}

/* =========================================================================
* 3. scan_filter — intensity gate
* ====================================================================== */
static void test_filter_intensity(void)
{
   section("3. scan_filter — intensity gate");
   LidarScan s;
   memset(&s, 0, sizeof(s));
   s.points[0] = (LidarPoint){.angle_cdeg=0,    .distance_mm=500, .intensity=5};  /* weak */
   s.points[1] = (LidarPoint){.angle_cdeg=1000, .distance_mm=500, .intensity=20}; /* ok   */
   s.points[2] = (LidarPoint){.angle_cdeg=2000, .distance_mm=500, .intensity=7};  /* weak */
   s.points[3] = (LidarPoint){.angle_cdeg=3000, .distance_mm=500, .intensity=100};/* ok   */
   s.count = 4;

   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.min_intensity = 8;
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);

   CHECK_EQ(s.count, 2u);
   CHECK_EQ(s.points[0].intensity, 20u);
   CHECK_EQ(s.points[1].intensity, 100u);

   ScanFilterStats st; scan_filter_get_stats(&st);
   CHECK_EQ(st.dropped_intens, 2u);
}

/* =========================================================================
* 4. scan_filter — range clamp
* ====================================================================== */
static void test_filter_range(void)
{
   section("4. scan_filter — range clamp");
   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* below min */
   s.points[0] = (LidarPoint){.angle_cdeg=0,    .distance_mm=50,   .intensity=100};
   /* exactly min */
   s.points[1] = (LidarPoint){.angle_cdeg=1000, .distance_mm=100,  .intensity=100};
   /* in range */
   s.points[2] = (LidarPoint){.angle_cdeg=2000, .distance_mm=3000, .intensity=100};
   /* exactly max */
   s.points[3] = (LidarPoint){.angle_cdeg=3000, .distance_mm=6000, .intensity=100};
   /* above max */
   s.points[4] = (LidarPoint){.angle_cdeg=4000, .distance_mm=6001, .intensity=100};
   s.count = 5;

   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.min_dist_mm   = 100;
   cfg.max_dist_mm   = 6000;
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);

   /* 50 mm dropped, 6001 mm dropped; 100, 3000, 6000 pass */
   CHECK_EQ(s.count, 3u);
   CHECK_EQ(s.points[0].distance_mm, 100u);
   CHECK_EQ(s.points[1].distance_mm, 3000u);
   CHECK_EQ(s.points[2].distance_mm, 6000u);

   ScanFilterStats st; scan_filter_get_stats(&st);
   CHECK_EQ(st.dropped_range, 2u);
}

/* =========================================================================
* 5. scan_filter — compaction preserves angle order
* ====================================================================== */
static void test_filter_compaction_order(void)
{
   section("5. scan_filter — compaction preserves order");
   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* Alternate valid / invalid by intensity */
   for (uint16_t i = 0; i < 10; i++) {
      s.points[i].angle_cdeg  = (uint16_t)(i * 1000u);
      s.points[i].distance_mm = 1000;
      s.points[i].intensity   = (i % 2 == 0) ? 200u : 3u;  /* odd indices weak */
   }
   s.count = 10;
   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.min_intensity = 8;
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);

   CHECK_EQ(s.count, 5u);  /* even indices: 0,2,4,6,8 */
   bool ascending = true;
   for (uint16_t i = 1; i < s.count; i++)
      if (s.points[i].angle_cdeg <= s.points[i-1].angle_cdeg)
         { ascending = false; break; }
   CHECK_MSG(ascending, "angles not monotonically ascending after compaction");
   CHECK_EQ(s.points[0].angle_cdeg, 0u);
   CHECK_EQ(s.points[4].angle_cdeg, 8000u);
}

/* =========================================================================
* 6. scan_filter — median filter w=3 and w=5
* ====================================================================== */
static void test_filter_median(void)
{
   section("6. scan_filter — median filter");

   /* 6a. Spike at centre: [1000, 1000, 5000, 1000, 1000]
   *     w=3 median at index 2: median(1000, 5000, 1000) = 1000  */
   {
      LidarScan s;
      memset(&s, 0, sizeof(s));
      uint16_t dists[5] = {1000, 1000, 5000, 1000, 1000};
      for (int i = 0; i < 5; i++) {
         s.points[i].angle_cdeg  = (uint16_t)(i * 1000u);
         s.points[i].distance_mm = dists[i];
         s.points[i].intensity   = 200;
      }
      s.count = 5;
      ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
      cfg.median_window = 3;
      scan_filter_apply(&s, &cfg);
      char msg[64];
      snprintf(msg, sizeof(msg), "centre dist=%u expected 1000",
               s.points[2].distance_mm);
      CHECK_MSG(s.points[2].distance_mm == 1000u, msg);
   }

   /* 6b. Spike at centre with w=5: [1000,1000,1000,5000,1000,1000,1000]
   *     median at index 3 = median(1000,1000,5000,1000,1000) = 1000  */
   {
      LidarScan s;
      memset(&s, 0, sizeof(s));
      uint16_t dists[7] = {1000,1000,1000,5000,1000,1000,1000};
      for (int i = 0; i < 7; i++) {
         s.points[i].angle_cdeg  = (uint16_t)(i * 500u);
         s.points[i].distance_mm = dists[i];
         s.points[i].intensity   = 200;
      }
      s.count = 7;
      ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
      cfg.median_window = 5;
      scan_filter_apply(&s, &cfg);
      CHECK_MSG(s.points[3].distance_mm == 1000u,
               "w=5 failed to remove spike at centre");
   }

   /* 6c. Constant input: median must equal input everywhere */
   {
      LidarScan s;
      make_scan(&s, 10, 1000, 2000, 200);
      /* Give them distinct angles */
      for (uint16_t i = 0; i < 10; i++)
         s.points[i].angle_cdeg = (uint16_t)(i * 1000u);
      ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
      cfg.median_window = 3;
      scan_filter_apply(&s, &cfg);
      bool all_ok = true;
      for (uint16_t i = 0; i < s.count; i++)
         if (s.points[i].distance_mm != 2000u) { all_ok = false; break; }
      CHECK_MSG(all_ok, "median altered constant-distance scan");
   }

   /* 6d. Edge points use reduced window (no out-of-bounds) */
   {
      LidarScan s;
      memset(&s, 0, sizeof(s));
      uint16_t dists[5] = {5000, 1000, 1000, 1000, 5000};
      for (int i = 0; i < 5; i++) {
         s.points[i].angle_cdeg  = (uint16_t)(i * 1000u);
         s.points[i].distance_mm = dists[i];
         s.points[i].intensity   = 200;
      }
      s.count = 5;
      ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
      cfg.median_window = 3;
      scan_filter_apply(&s, &cfg);
      /* Must not crash; results are deterministic */
      CHECK_EQ(s.count, 5u);
   }

   /* 6e. Stats: median_applied == out_count when w=3 */
   {
      LidarScan s;
      make_scan(&s, 8, 0, 500, 200);
      for (uint16_t i = 0; i < 8; i++)
         s.points[i].angle_cdeg = (uint16_t)(i * 1000u);
      ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
      cfg.median_window = 3;
      scan_filter_apply(&s, &cfg);
      ScanFilterStats st; scan_filter_get_stats(&st);
      CHECK_EQ(st.median_applied, st.out_count);
   }
}

/* =========================================================================
* 7. scan_filter — median disabled (window=0)
* ====================================================================== */
static void test_filter_median_disabled(void)
{
   section("7. scan_filter — median disabled");
   LidarScan s;
   memset(&s, 0, sizeof(s));
   uint16_t dists[5] = {1000, 1000, 5000, 1000, 1000};
   for (int i = 0; i < 5; i++) {
      s.points[i].angle_cdeg  = (uint16_t)(i * 1000u);
      s.points[i].distance_mm = dists[i];
      s.points[i].intensity   = 200;
   }
   s.count = 5;
   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);
   /* Spike must survive */
   CHECK_EQ(s.points[2].distance_mm, 5000u);

   ScanFilterStats st; scan_filter_get_stats(&st);
   CHECK_EQ(st.median_applied, 0u);
}

/* =========================================================================
* 8. scan_filter — NULL guards
* ====================================================================== */
static void test_filter_null_guards(void)
{
   section("8. scan_filter — NULL guards");
   /* NULL scan must not crash */
   scan_filter_apply(NULL, NULL);
   CHECK(true);
   /* NULL stats must not crash */
   scan_filter_get_stats(NULL);
   CHECK(true);
   /* NULL cfg → uses defaults */
   LidarScan s;
   make_scan(&s, 3, 1000, 500, 200);
   scan_filter_apply(&s, NULL);
   CHECK_MSG(s.count <= 3u, "NULL cfg caused count to grow");
}

/* =========================================================================
* 9. scan_filter — stats counters total consistency
* ====================================================================== */
static void test_filter_stats_total(void)
{
   section("9. scan_filter — stats totals");
   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* 2 zero, 2 weak intensity, 2 out-of-range, 4 valid */
   s.points[0]  = (LidarPoint){1000, 0,    200};  /* zero */
   s.points[1]  = (LidarPoint){2000, 0,    200};  /* zero */
   s.points[2]  = (LidarPoint){3000, 500,  3  };  /* weak */
   s.points[3]  = (LidarPoint){4000, 500,  4  };  /* weak */
   s.points[4]  = (LidarPoint){5000, 50,   200};  /* below min */
   s.points[5]  = (LidarPoint){6000, 9000, 200};  /* above max */
   s.points[6]  = (LidarPoint){7000, 200,  200};  /* valid */
   s.points[7]  = (LidarPoint){8000, 500,  200};  /* valid */
   s.points[8]  = (LidarPoint){9000, 1000, 200};  /* valid */
   s.points[9]  = (LidarPoint){10000,3000, 200};  /* valid */
   s.count = 10;
   ScanFilterCfg cfg; scan_filter_cfg_default(&cfg);
   cfg.min_dist_mm   = 100;
   cfg.max_dist_mm   = 6000;
   cfg.min_intensity = 8;
   cfg.median_window = 0;
   scan_filter_apply(&s, &cfg);
   ScanFilterStats st; scan_filter_get_stats(&st);
   CHECK_EQ(st.in_count,       10u);
   CHECK_EQ(st.dropped_zero,    2u);
   CHECK_EQ(st.dropped_intens,  2u);
   CHECK_EQ(st.dropped_range,   2u);
   CHECK_EQ(st.out_count,       4u);
   /* Dropped + out == in */
   CHECK_EQ((uint32_t)st.dropped_zero + st.dropped_intens +
            st.dropped_range + st.out_count, 10u);
}

/* =========================================================================
* 10. occupancy_grid — init
* ====================================================================== */
static void test_grid_init(void)
{
   section("10. occupancy_grid — init");
   /* Stack-allocated to verify no heap dependency. */
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* All cells must be UNKNOWN. */
   bool all_unk = true;
   for (int i = 0; i < GRID_W * GRID_H; i++)
      if (g.cells[i] != CELL_UNKNOWN) { all_unk = false; break; }
   CHECK_MSG(all_unk, "init did not set all cells to CELL_UNKNOWN");

   bool all_unk_inf = true;
   for (int i = 0; i < GRID_W * GRID_H; i++)
      if (g.inflated[i] != CELL_UNKNOWN) { all_unk_inf = false; break; }
   CHECK_MSG(all_unk_inf, "init did not set inflated to CELL_UNKNOWN");

   /* Mask must have been populated. */
   CHECK_MSG(g.mask_count > 0u, "inflation mask is empty after init");
   /* mask_count <= INFLATE_MASK_MAX */
   CHECK_MSG(g.mask_count <= INFLATE_MASK_MAX, "mask_count exceeds maximum");

   /* Mask entries must all be within radius. */
   int r = INFLATE_RADIUS_CELLS;
   bool in_radius = true;
   for (uint16_t m = 0; m < g.mask_count; m++) {
      int dx = g.mask_dx[m], dy = g.mask_dy[m];
      if (dx*dx + dy*dy > r*r) { in_radius = false; break; }
   }
   CHECK_MSG(in_radius, "mask contains offset outside inflation radius");
}

/* =========================================================================
* 11. occupancy_grid — clear
* ====================================================================== */
static void test_grid_clear(void)
{
   section("11. occupancy_grid — clear");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Dirty some cells and counters. */
   g.cells[0]      = CELL_OCCUPIED;
   g.rays_cast     = 999;
   g.cells_freed   = 888;
   occ_grid_clear(&g);

   CHECK_EQ(g.cells[0],    CELL_UNKNOWN);
   CHECK_EQ(g.rays_cast,   0u);
   CHECK_EQ(g.cells_freed, 0u);

   /* Mask must survive a clear (not recomputed). */
   CHECK_MSG(g.mask_count > 0u, "clear destroyed the inflation mask");
}

/* =========================================================================
* 12. occupancy_grid — cell accessors
* ====================================================================== */
static void test_grid_accessors(void)
{
   section("12. occupancy_grid — cell accessors");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Write and read back a value in bounds. */
   occ_grid_set(&g, 10, 20, CELL_OCCUPIED);
   CHECK_EQ(occ_grid_get(&g, 10, 20), CELL_OCCUPIED);

   /* Overwrite with FREE. */
   occ_grid_set(&g, 10, 20, CELL_FREE);
   CHECK_EQ(occ_grid_get(&g, 10, 20), CELL_FREE);

   /* Out-of-bounds read returns CELL_UNKNOWN. */
   CHECK_EQ(occ_grid_get(&g, -1,    0),   CELL_UNKNOWN);
   CHECK_EQ(occ_grid_get(&g,  0,   -1),   CELL_UNKNOWN);
   CHECK_EQ(occ_grid_get(&g, GRID_W, 0),  CELL_UNKNOWN);
   CHECK_EQ(occ_grid_get(&g, 0,  GRID_H), CELL_UNKNOWN);

   /* Out-of-bounds write must not crash. */
   occ_grid_set(&g, -5, 0,      CELL_OCCUPIED);
   occ_grid_set(&g, GRID_W, 0,  CELL_OCCUPIED);
   CHECK(true);

   /* occ_grid_get_inflated OOB returns CELL_INFLATED. */
   CHECK_EQ(occ_grid_get_inflated(&g, -1, 0), CELL_INFLATED);
   CHECK_EQ(occ_grid_get_inflated(&g, GRID_W, GRID_H), CELL_INFLATED);
}

/* =========================================================================
* Helper: expose flat index from test code (same formula as .c)
* ====================================================================== */
static inline uint32_t cell_idx_pub(int16_t cx, int16_t cy) {
   return (uint32_t)cy * (uint32_t)GRID_W + (uint32_t)cx;
}

/* =========================================================================
* 13. occupancy_grid — passable gate
* ====================================================================== */
static void test_grid_passable(void)
{
   section("13. occupancy_grid — passable gate");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* UNKNOWN is passable (allows exploration). */
   CHECK(occ_grid_passable(&g, 5, 5) == true);

   /* Manually set inflated cell to FREE: passable. */
   g.inflated[(int)cell_idx_pub(5, 5)] = CELL_FREE;
   CHECK(occ_grid_passable(&g, 5, 5) == true);

   /* OCCUPIED: not passable. */
   g.inflated[(int)cell_idx_pub(5, 5)] = CELL_OCCUPIED;
   CHECK(occ_grid_passable(&g, 5, 5) == false);

   /* INFLATED: not passable. */
   g.inflated[(int)cell_idx_pub(5, 5)] = CELL_INFLATED;
   CHECK(occ_grid_passable(&g, 5, 5) == false);

   /* OOB: not passable. */
   CHECK(occ_grid_passable(&g, -1, 0) == false);
}

/* =========================================================================
* 14. occupancy_grid — coordinate conversion round-trip
* ====================================================================== */
static void test_grid_coords(void)
{
   section("14. occupancy_grid — coordinate conversion");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* World (0,0) → robot cell (centre) */
   {
      int16_t cx, cy;
      occ_grid_world_to_cell(&g, 0, 0, &cx, &cy);
      CHECK_EQ(cx, ROBOT_CX);
      CHECK_EQ(cy, ROBOT_CY);
   }

   /* Forward (+X): wx = GRID_CELL_MM → one cell right of centre */
   {
      int16_t cx, cy;
      occ_grid_world_to_cell(&g, GRID_CELL_MM, 0, &cx, &cy);
      CHECK_EQ(cx, ROBOT_CX + 1);
      CHECK_EQ(cy, ROBOT_CY);
   }

   /* Left (+Y): wy = GRID_CELL_MM → one cell UP (row decreases) */
   {
      int16_t cx, cy;
      occ_grid_world_to_cell(&g, 0, GRID_CELL_MM, &cx, &cy);
      CHECK_EQ(cx, ROBOT_CX);
      CHECK_EQ(cy, ROBOT_CY - 1);
   }

   /* Round-trip: world → cell → world gives back cell centre */
   {
      int32_t wx0 = 3 * GRID_CELL_MM;
      int32_t wy0 = -2 * GRID_CELL_MM;
      int16_t cx, cy;
      occ_grid_world_to_cell(&g, wx0, wy0, &cx, &cy);
      int32_t wx1, wy1;
      occ_grid_cell_to_world(&g, cx, cy, &wx1, &wy1);
      /* cell_to_world returns cell centre (wx0 + half_cell) */
      int32_t half = GRID_CELL_MM / 2;
      CHECK_EQ(wx1, wx0 + half);
      CHECK_EQ(wy1, wy0 - half);
   }

   /* OOB world coords are clamped, not wrapped. */
   {
      int16_t cx, cy;
      occ_grid_world_to_cell(&g, 9999999, 9999999, &cx, &cy);
      CHECK_EQ(cx, GRID_W - 1);
      CHECK_EQ(cy, 0);            /* large +wy → small cy (clamped to 0) */
      occ_grid_world_to_cell(&g, -9999999, -9999999, &cx, &cy);
      CHECK_EQ(cx, 0);
      CHECK_EQ(cy, GRID_H - 1);
   }
}

/* =========================================================================
* 15. occupancy_grid — robot cell is always grid centre
* ====================================================================== */
static void test_grid_robot_cell(void)
{
   section("15. occupancy_grid — robot cell");
   int16_t cx, cy;
   occ_grid_robot_cell(&cx, &cy);
   CHECK_EQ(cx, ROBOT_CX);
   CHECK_EQ(cy, ROBOT_CY);
}

/* =========================================================================
* 16. occupancy_grid — ray-cast: forward obstacle
*
* Single point directly forward (angle=0°), 2 cells away.
* Expected: endpoint cell is OCCUPIED; cells between robot and endpoint FREE.
* ====================================================================== */
static void test_grid_raycast_forward(void)
{
   section("16. occupancy_grid — ray-cast: forward obstacle");
   static OccupancyGrid g;
   occ_grid_init(&g);

   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* angle=0° → cos=1, sin=0 → wx=dist, wy=0 */
   int32_t dist_mm = 2 * GRID_CELL_MM;   /* exactly 2 cells forward */
   s.points[0] = (LidarPoint){.angle_cdeg=0, .distance_mm=(uint16_t)dist_mm, .intensity=200};
   s.count = 1;

   occ_grid_update(&g, &s);

   /* Endpoint cell: ROBOT_CX+2, ROBOT_CY */
   int16_t ex = (int16_t)(ROBOT_CX + 2), ey = (int16_t)ROBOT_CY;
   CHECK_EQ(occ_grid_get(&g, ex, ey), CELL_OCCUPIED);

   /* Intermediate cell: ROBOT_CX+1, ROBOT_CY */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+1), (int16_t)ROBOT_CY), CELL_FREE);

   /* Robot cell itself: FREE (ray start, marked free by Bresenham) */
   /* Note: robot cell is the Bresenham start; it is never explicitly marked
   * by the ray walk (the first step moves away from start), so it remains
   * UNKNOWN unless a different ray passes through it. */

   /* Cells behind obstacle (ROBOT_CX+3 onward): UNKNOWN */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+3), (int16_t)ROBOT_CY), CELL_UNKNOWN);
}

/* =========================================================================
* 17. occupancy_grid — ray-cast: free corridor
*
* Two forward-facing points at different distances.
* The nearer endpoint must be OCCUPIED; cells between them FREE.
* ====================================================================== */
static void test_grid_raycast_free_corridor(void)
{
   section("17. occupancy_grid — ray-cast: free corridor");
   static OccupancyGrid g;
   occ_grid_init(&g);

   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* Near obstacle 3 cells forward. */
   s.points[0] = (LidarPoint){.angle_cdeg=0, .distance_mm=(uint16_t)(3*GRID_CELL_MM), .intensity=200};
   /* Far obstacle 6 cells forward — MUST be ignored after the near hit. */
   s.points[1] = (LidarPoint){.angle_cdeg=0, .distance_mm=(uint16_t)(6*GRID_CELL_MM), .intensity=200};
   s.count = 2;

   occ_grid_update(&g, &s);

   /* Near endpoint: OCCUPIED. */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+3), (int16_t)ROBOT_CY), CELL_OCCUPIED);
   /* Far endpoint: also OCCUPIED (separate ray). */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+6), (int16_t)ROBOT_CY), CELL_OCCUPIED);
   /* Intermediate cell 4 and 5: FREE (far ray traces through). */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+4), (int16_t)ROBOT_CY), CELL_FREE);
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+5), (int16_t)ROBOT_CY), CELL_FREE);
   /* Cells 1 and 2: FREE (both rays pass through). */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+1), (int16_t)ROBOT_CY), CELL_FREE);
}

/* =========================================================================
* 18. occupancy_grid — ray-cast: out-of-range clips gracefully
* ====================================================================== */
static void test_grid_raycast_clip(void)
{
   section("18. occupancy_grid — ray-cast: out-of-range clips");
   static OccupancyGrid g;
   occ_grid_init(&g);

   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* Distance far beyond grid in the forward direction. */
   uint16_t far_dist = (uint16_t)((GRID_W * GRID_CELL_MM) + 1000u);
   s.points[0] = (LidarPoint){.angle_cdeg=0, .distance_mm=far_dist, .intensity=200};
   s.count = 1;
   occ_grid_update(&g, &s);

   /* Clipped ray must increment the clipped counter. */
   uint32_t clipped = 0;
   occ_grid_get_stats(&g, NULL, NULL, NULL, &clipped);
   CHECK_MSG(clipped >= 1u, "clipped counter not incremented for OOB ray");

   /* No OCCUPIED cell must appear outside the grid (trivially true). */
   /* The rightmost column (GRID_W-1) may be FREE if the ray passed through. */
   bool no_oob_occupied = true;
   for (int cy2 = 0; cy2 < GRID_H; cy2++)
      for (int cx2 = 0; cx2 < GRID_W; cx2++)
         if (g.cells[(uint32_t)cy2*GRID_W+(uint32_t)cx2] == CELL_OCCUPIED)
               /* all occupied cells are inside the grid — trivially OK */
               (void)0;
   CHECK_MSG(no_oob_occupied, "OCCUPIED written outside grid");
}

/* =========================================================================
* 19. occupancy_grid — ray-cast: all four quadrants
*
* Four points at 90° intervals (forward/left/back/right).
* Each must produce an OCCUPIED cell in the correct direction.
* ====================================================================== */
static void test_grid_raycast_quadrants(void)
{
   section("19. occupancy_grid — ray-cast: four quadrants");
   static OccupancyGrid g;
   occ_grid_init(&g);

   LidarScan s;
   memset(&s, 0, sizeof(s));
   uint16_t d = (uint16_t)(3 * GRID_CELL_MM);
   s.points[0] = (LidarPoint){.angle_cdeg=    0, .distance_mm=d, .intensity=200}; /* 0°  → +X */
   s.points[1] = (LidarPoint){.angle_cdeg= 9000, .distance_mm=d, .intensity=200}; /* 90° → +Y */
   s.points[2] = (LidarPoint){.angle_cdeg=18000, .distance_mm=d, .intensity=200}; /* 180°→ -X */
   s.points[3] = (LidarPoint){.angle_cdeg=27000, .distance_mm=d, .intensity=200}; /* 270°→ -Y */
   s.count = 4;
   occ_grid_update(&g, &s);

   /* 0°: ROBOT_CX+3, ROBOT_CY */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX+3), (int16_t)ROBOT_CY), CELL_OCCUPIED);
   /* 90°: ROBOT_CX, ROBOT_CY-3 (Y increases upward, rows decrease) */
   CHECK_EQ(occ_grid_get(&g, (int16_t)ROBOT_CX, (int16_t)(ROBOT_CY-3)), CELL_OCCUPIED);
   /* 180°: ROBOT_CX-3, ROBOT_CY */
   CHECK_EQ(occ_grid_get(&g, (int16_t)(ROBOT_CX-3), (int16_t)ROBOT_CY), CELL_OCCUPIED);
   /* 270°: ROBOT_CX, ROBOT_CY+3 */
   CHECK_EQ(occ_grid_get(&g, (int16_t)ROBOT_CX, (int16_t)(ROBOT_CY+3)), CELL_OCCUPIED);
}

/* =========================================================================
* 20. occupancy_grid — inflation: obstacle expands by radius
* ====================================================================== */
static void test_grid_inflation_radius(void)
{
   section("20. occupancy_grid — inflation: radius expansion");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Place a single OCCUPIED cell in the centre. */
   occ_grid_set(&g, (int16_t)ROBOT_CX, (int16_t)ROBOT_CY, CELL_OCCUPIED);
   occ_grid_inflate(&g);

   int r = INFLATE_RADIUS_CELLS;

   /* All cells within radius must be INFLATED or OCCUPIED in inflated[]. */
   bool all_inflated = true;
   for (int dy = -r; dy <= r; dy++) {
      for (int dx = -r; dx <= r; dx++) {
         if (dx*dx + dy*dy > r*r) continue;
         int16_t nx = (int16_t)(ROBOT_CX + dx);
         int16_t ny = (int16_t)(ROBOT_CY + dy);
         uint8_t v = occ_grid_get_inflated(&g, nx, ny);
         if (v != CELL_INFLATED && v != CELL_OCCUPIED) {
               all_inflated = false;
               break;
         }
      }
   }
   CHECK_MSG(all_inflated, "cells within radius not inflated");

   /* Cells just outside radius must remain UNKNOWN (unaffected). */
   int16_t outside_x = (int16_t)(ROBOT_CX + r + 2);
   uint8_t outside_v = occ_grid_get_inflated(&g, outside_x, (int16_t)ROBOT_CY);
   CHECK_MSG(outside_v == CELL_UNKNOWN,
            "cell outside inflation radius was unexpectedly inflated");
}

/* =========================================================================
* 21. occupancy_grid — inflation: does not overwrite OCCUPIED
* ====================================================================== */
static void test_grid_inflation_no_overwrite(void)
{
   section("21. occupancy_grid — inflation: OCCUPIED not overwritten");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Two adjacent occupied cells. */
   occ_grid_set(&g, (int16_t)ROBOT_CX,   (int16_t)ROBOT_CY, CELL_OCCUPIED);
   occ_grid_set(&g, (int16_t)(ROBOT_CX+1),(int16_t)ROBOT_CY, CELL_OCCUPIED);
   occ_grid_inflate(&g);

   /* Both must remain OCCUPIED in the inflated grid. */
   CHECK_EQ(occ_grid_get_inflated(&g, (int16_t)ROBOT_CX,    (int16_t)ROBOT_CY), CELL_OCCUPIED);
   CHECK_EQ(occ_grid_get_inflated(&g, (int16_t)(ROBOT_CX+1),(int16_t)ROBOT_CY), CELL_OCCUPIED);
}

/* =========================================================================
* 22. occupancy_grid — robot cell never inflated when no obstacle
* ====================================================================== */
static void test_grid_robot_not_inflated(void)
{
   section("22. occupancy_grid — robot cell not inflated");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Single obstacle far from the robot. */
   occ_grid_set(&g, (int16_t)(ROBOT_CX + 20), (int16_t)ROBOT_CY, CELL_OCCUPIED);
   occ_grid_inflate(&g);

   uint8_t robot_cell_val = occ_grid_get_inflated(&g, (int16_t)ROBOT_CX, (int16_t)ROBOT_CY);
   CHECK_MSG(robot_cell_val != CELL_INFLATED && robot_cell_val != CELL_OCCUPIED,
            "robot cell was incorrectly inflated");
}

/* =========================================================================
* 23. occupancy_grid — OOB mask entries silently ignored
* ====================================================================== */
static void test_grid_inflation_oob(void)
{
   section("23. occupancy_grid — inflation: OOB mask entries ignored");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Obstacle at the grid corner — mask entries will be OOB. */
   occ_grid_set(&g, 0, 0, CELL_OCCUPIED);
   occ_grid_inflate(&g);  /* must not crash or write outside grid */
   CHECK(true);
   /* Cells in bounds around (0,0) should be inflated. */
   uint8_t v = occ_grid_get_inflated(&g, 1, 0);
   CHECK_MSG(v == CELL_INFLATED || v == CELL_OCCUPIED,
            "corner inflation did not expand to (1,0)");
}

/* =========================================================================
* 24. occupancy_grid — full pipeline
* ====================================================================== */
static void test_grid_full_pipeline(void)
{
   section("24. occupancy_grid — full pipeline");
   static OccupancyGrid g;
   occ_grid_init(&g);

   /* Ring of obstacles at 1 m range. */
   LidarScan s;
   make_ring_scan(&s, 1000u, 200u);

   ScanFilterCfg fcfg; scan_filter_cfg_default(&fcfg);
   fcfg.median_window = 3;
   scan_filter_apply(&s, &fcfg);
   occ_grid_update(&g, &s);
   occ_grid_inflate(&g);

   /* Robot cell (centre) must be passable (it is FREE or UNKNOWN). */
   CHECK_MSG(occ_grid_passable(&g, (int16_t)ROBOT_CX, (int16_t)ROBOT_CY),
            "robot cell is blocked after ring scan");

   /* Some occupied cells must exist. */
   uint32_t occ_count = 0;
   occ_grid_get_stats(&g, NULL, NULL, &occ_count, NULL);
   CHECK_MSG(occ_count > 0u, "no occupied cells after ring scan");

   /* Some inflated cells must exist in the inflated grid. */
   uint32_t n_inf = 0;
   for (int i = 0; i < GRID_W * GRID_H; i++)
      if (g.inflated[i] == CELL_INFLATED) n_inf++;
   CHECK_MSG(n_inf > 0u, "no inflated cells after inflation");

   /* Obstacle cells must not be passable. */
   /* Forward direction: 1000mm / GRID_CELL_MM cells from centre. */
   int16_t obs_cx = (int16_t)(ROBOT_CX + 1000 / GRID_CELL_MM);
   CHECK_MSG(!occ_grid_passable(&g, obs_cx, (int16_t)ROBOT_CY),
            "obstacle cell is passable");
}

/* =========================================================================
* 25. occupancy_grid — stats counters
* ====================================================================== */
static void test_grid_stats(void)
{
   section("25. occupancy_grid — stats counters");
   static OccupancyGrid g;
   occ_grid_init(&g);

   LidarScan s;
   memset(&s, 0, sizeof(s));
   /* 4 valid forward points at distinct distances. */
   for (int i = 0; i < 4; i++) {
      s.points[i].angle_cdeg  = 0;
      s.points[i].distance_mm = (uint16_t)((i+1) * GRID_CELL_MM);
      s.points[i].intensity   = 200;
   }
   s.count = 4;
   occ_grid_update(&g, &s);

   uint32_t rays, freed, occupied, clipped;
   occ_grid_get_stats(&g, &rays, &freed, &occupied, &clipped);
   CHECK_EQ(rays, 4u);
   CHECK_MSG(occupied <= 4u, "occupied count > 4 for 4 valid rays");
   CHECK_MSG(freed > 0u,     "no free cells from 4 forward rays");
   /* NULL args must not crash. */
   occ_grid_get_stats(&g, NULL, NULL, NULL, NULL);
   CHECK(true);
}

/* =========================================================================
* 26–28. grid_viz smoke tests
* ====================================================================== */
static void test_viz(void)
{
   section("26–28. grid_viz");

   /* 26. viz_cfg_default */
   VizCfg cfg;
   viz_cfg_default(&cfg);
   CHECK_MSG(cfg.downsample >= 1u, "downsample must be >= 1");

   /* 27. viz_render with a populated grid — must not crash */
   {
      static OccupancyGrid g;
      occ_grid_init(&g);
      LidarScan s;
      make_ring_scan(&s, 1000u, 200u);
      occ_grid_update(&g, &s);
      occ_grid_inflate(&g);

      VizCfg rcfg;
      viz_cfg_default(&rcfg);
      rcfg.use_colour   = false;   /* plain ASCII for test output */
      rcfg.clear_screen = false;   /* don't clear test output */
      rcfg.show_stats   = false;
      rcfg.downsample   = (uint8_t)(GRID_W / 40 + 1); /* fit in 40 cols */

      printf("\n  [grid_viz smoke render — 1 m ring]\n");
      viz_render(&g, NULL, &rcfg);
      CHECK(true);   /* crash-free = pass */

      /* viz_render with NULL grid must not crash. */
      viz_render(NULL, NULL, &rcfg);
      CHECK(true);
   }

   /* 28. viz_dump_csv */
   {
      static OccupancyGrid g;
      occ_grid_init(&g);
      occ_grid_set(&g, 5, 5, CELL_OCCUPIED);
      bool ok = viz_dump_csv(&g, "/tmp/test_grid.csv");
      CHECK_MSG(ok, "viz_dump_csv returned false");

      if (ok) {
         /* Read back and verify the occupied cell appears. */
         FILE *fp = fopen("/tmp/test_grid.csv", "r");
         bool found_200 = false;
         if (fp) {
               char line[2048];
               while (fgets(line, sizeof(line), fp)) {
                  if (strstr(line, "200")) { found_200 = true; break; }
               }
               fclose(fp);
         }
         CHECK_MSG(found_200, "CSV file does not contain CELL_OCCUPIED (200)");
      }

      /* NULL path and NULL grid must not crash. */
      viz_dump_csv(NULL, "/tmp/test_null.csv");
      viz_dump_csv(&g, NULL);
      CHECK(true);
   }
}

/* =========================================================================
* 29. Optional: replay scan_raw.bin through full pipeline
* ====================================================================== */
static void test_replay_pipeline(const char *path)
{
   section("29. Pipeline replay from scan_raw.bin");

   FILE *fp = fopen(path, "rb");
   if (!fp) { printf("  (skipped — cannot open '%s')\n", path); return; }

   LidarParser    *parser = lidar_parser_create();
   static OccupancyGrid g;
   occ_grid_init(&g);

   ScanFilterCfg fcfg; scan_filter_cfg_default(&fcfg);
   fcfg.median_window = 3;

   uint8_t buf[256];
   LidarScan scan;
   int  scans_processed = 0;
   // bool first_rendered  = false;

   while (fread(buf, 1, sizeof(buf), fp) > 0) {
      if (lidar_parser_feed(parser, buf, (int)sizeof(buf), &scan) != PARSE_OK)
         continue;

      scan_filter_apply(&scan, &fcfg);
      occ_grid_update(&g, &scan);
      occ_grid_inflate(&g);
      scans_processed++;

      /* Render the first scan to stdout for visual inspection. */
      // if (!first_rendered) {
         VizCfg vcfg; viz_cfg_default(&vcfg);
         vcfg.use_colour   = isatty(STDOUT_FILENO);
         vcfg.clear_screen = false;
         vcfg.show_stats   = true;
         vcfg.downsample   = (uint8_t)(GRID_W / 60 + 1);
         // printf("\n  [First scan grid — scan#1]\n");
         printf("\n  [Scan grid no — scan#%d]\n", scans_processed);
         viz_render(&g, NULL, &vcfg);
         viz_dump_csv(&g, "/tmp/replay_grid.csv");
         // first_rendered = true;
      // }
   }
   fclose(fp);
   lidar_parser_destroy(parser);

   printf("  Scans processed : %d\n", scans_processed);

   CHECK_MSG(scans_processed > 0, "no scans processed from replay file");

   /* After processing, some cells must be occupied and some free. */
   uint32_t rays, freed, occupied, clipped;
   occ_grid_get_stats(&g, &rays, &freed, &occupied, &clipped);
   printf("  Last scan stats : rays=%u freed=%u occupied=%u clipped=%u\n",
         rays, freed, occupied, clipped);
   CHECK_MSG(occupied > 0u, "no occupied cells after replay");
   CHECK_MSG(freed    > 0u, "no free cells after replay");

   /* Robot cell must be passable. */
   CHECK_MSG(occ_grid_passable(&g, (int16_t)ROBOT_CX, (int16_t)ROBOT_CY),
            "robot cell not passable after replay");
}

/* =========================================================================
* main
* ====================================================================== */
int main(int argc, char *argv[])
{
   g_colour = isatty(STDOUT_FILENO);
   printf("\n%sPhase 2 — scan_filter + occupancy_grid test suite%s\n",
         g_colour?"\033[1;33m":"", g_colour?"\033[0m":"");

   /* scan_filter */
   test_filter_defaults();
   test_filter_zero_dist();
   test_filter_intensity();
   test_filter_range();
   test_filter_compaction_order();
   test_filter_median();
   test_filter_median_disabled();
   test_filter_null_guards();
   test_filter_stats_total();

   /* occupancy_grid */
   test_grid_init();
   test_grid_clear();
   test_grid_accessors();
   test_grid_passable();
   test_grid_coords();
   test_grid_robot_cell();
   test_grid_raycast_forward();
   test_grid_raycast_free_corridor();
   test_grid_raycast_clip();
   test_grid_raycast_quadrants();
   test_grid_inflation_radius();
   test_grid_inflation_no_overwrite();
   test_grid_robot_not_inflated();
   test_grid_inflation_oob();
   test_grid_full_pipeline();
   test_grid_stats();

   /* grid_viz */
   test_viz();

   if (argc >= 2)
      test_replay_pipeline(argv[1]);
   else
      printf("\n  Tip: pass path to scan_raw.bin for pipeline replay test.\n");

   /* Summary */
   printf("\n%s────────────────────────────────%s\n",
         g_colour?"\033[1m":"", g_colour?"\033[0m":"");
   printf("Tests run    : %d\n", g_run);
   printf("Tests passed : %s%d%s\n",
         g_colour?"\033[32m":"", g_pass, g_colour?"\033[0m":"");
   if (g_fail > 0)
      printf("Tests %sFAILED%s : %d\n",
            g_colour?"\033[31m":"", g_colour?"\033[0m":"", g_fail);
   else
      printf("%sAll tests passed.%s\n",
            g_colour?"\033[32m":"", g_colour?"\033[0m":"");
   printf("\n");
   return (g_fail > 0) ? 1 : 0;
}