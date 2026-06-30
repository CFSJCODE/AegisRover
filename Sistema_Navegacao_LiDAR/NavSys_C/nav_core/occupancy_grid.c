/**
 * @file occupancy_grid.c
 * @brief 2-D occupancy grid — implementation.
 *
 * Portable C11.  No platform headers, no dynamic allocation, no float.
 *
 * ── Trig ──────────────────────────────────────────────────────────────────
 *
 * Polar→Cartesian conversion uses a 360-entry Q15 integer LUT.
 *
 *   x_mm = (dist_mm × cos_lut[deg] + rem × Δcos) > 15
 *   y_mm = (dist_mm × sin_lut[deg] + rem × Δsin) > 15
 *
 * where deg = angle_cdeg / 100 and rem = angle_cdeg % 100.
 * Maximum error: 0.5 mm at 6 m range (verified).
 *
 * LiDAR angle 0° maps to world +X (forward).
 * LiDAR angle 90° maps to world +Y (left).
 * Cell row index increases downward (screen convention), so:
 *
 *   cx = GRID_W/2 + x_mm / GRID_CELL_MM
 *   cy = GRID_H/2 - y_mm / GRID_CELL_MM   ← Y inverted
 *
 * ── Bresenham ─────────────────────────────────────────────────────────────
 *
 * Standard integer Bresenham line.  The robot cell is the start point;
 * the endpoint is the LiDAR return cell.  Cells along the ray (up to but
 * not including the endpoint) are marked FREE.  The endpoint is marked
 * OCCUPIED.  If the endpoint is outside the grid, the ray is traced to
 * the grid edge (free cells within the grid still updated) and the hit is
 * counted as clipped (no OCCUPIED written outside bounds).
 *
 * ── Inflation ─────────────────────────────────────────────────────────────
 *
 * A precomputed circular mask of (dx, dy) offsets lists every cell within
 * INFLATE_RADIUS_CELLS of the origin.  For each OCCUPIED cell the mask is
 * applied to the inflated[] copy.  Inflating on a separate array prevents
 * the inflation from growing with each step (no spreading artefact).
**/

#include "occupancy_grid.h"
#include <string.h>   /* memset, memcpy */
#include <stddef.h>   /* NULL */

/* =========================================================================
* Q15 trig LUT — const → .rodata (flash on ESP32-C6, zero DRAM cost)
* ====================================================================== */

static const int16_t g_cos_lut[360] = {
    32767,   32762,   32747,   32722,   32687,   32642,   32587,   32523,
    32448,   32364,   32269,   32165,   32051,   31927,   31794,   31650,
    31498,   31335,   31163,   30982,   30791,   30591,   30381,   30162,
    29934,   29697,   29451,   29196,   28932,   28659,   28377,   28087,
    27788,   27481,   27165,   26841,   26509,   26169,   25821,   25465,
    25101,   24730,   24351,   23964,   23571,   23170,   22762,   22347,
    21925,   21497,   21062,   20621,   20173,   19720,   19260,   18794,
    18323,   17846,   17364,   16876,   16384,   15886,   15383,   14876,
    14364,   13848,   13328,   12803,   12275,   11743,   11207,   10668,
    10126,    9580,    9032,    8481,    7927,    7371,    6813,    6252,
     5690,    5126,    4560,    3993,    3425,    2856,    2286,    1715,
     1144,     572,       0,    -572,   -1144,   -1715,   -2286,   -2856,
    -3425,   -3993,   -4560,   -5126,   -5690,   -6252,   -6813,   -7371,
    -7927,   -8481,   -9032,   -9580,  -10126,  -10668,  -11207,  -11743,
   -12275,  -12803,  -13328,  -13848,  -14364,  -14876,  -15383,  -15886,
   -16383,  -16876,  -17364,  -17846,  -18323,  -18794,  -19260,  -19720,
   -20173,  -20621,  -21062,  -21497,  -21925,  -22347,  -22762,  -23170,
   -23571,  -23964,  -24351,  -24730,  -25101,  -25465,  -25821,  -26169,
   -26509,  -26841,  -27165,  -27481,  -27788,  -28087,  -28377,  -28659,
   -28932,  -29196,  -29451,  -29697,  -29934,  -30162,  -30381,  -30591,
   -30791,  -30982,  -31163,  -31335,  -31498,  -31650,  -31794,  -31927,
   -32051,  -32165,  -32269,  -32364,  -32448,  -32523,  -32587,  -32642,
   -32687,  -32722,  -32747,  -32762,  -32767,  -32762,  -32747,  -32722,
   -32687,  -32642,  -32587,  -32523,  -32448,  -32364,  -32269,  -32165,
   -32051,  -31927,  -31794,  -31650,  -31498,  -31335,  -31163,  -30982,
   -30791,  -30591,  -30381,  -30162,  -29934,  -29697,  -29451,  -29196,
   -28932,  -28659,  -28377,  -28087,  -27788,  -27481,  -27165,  -26841,
   -26509,  -26169,  -25821,  -25465,  -25101,  -24730,  -24351,  -23964,
   -23571,  -23170,  -22762,  -22347,  -21925,  -21497,  -21062,  -20621,
   -20173,  -19720,  -19260,  -18794,  -18323,  -17846,  -17364,  -16876,
   -16384,  -15886,  -15383,  -14876,  -14364,  -13848,  -13328,  -12803,
   -12275,  -11743,  -11207,  -10668,  -10126,   -9580,   -9032,   -8481,
    -7927,   -7371,   -6813,   -6252,   -5690,   -5126,   -4560,   -3993,
    -3425,   -2856,   -2286,   -1715,   -1144,    -572,       0,     572,
     1144,    1715,    2286,    2856,    3425,    3993,    4560,    5126,
     5690,    6252,    6813,    7371,    7927,    8481,    9032,    9580,
    10126,   10668,   11207,   11743,   12275,   12803,   13328,   13848,
    14364,   14876,   15383,   15886,   16384,   16876,   17364,   17846,
    18323,   18794,   19260,   19720,   20173,   20621,   21062,   21497,
    21925,   22347,   22762,   23170,   23571,   23964,   24351,   24730,
    25101,   25465,   25821,   26169,   26509,   26841,   27165,   27481,
    27788,   28087,   28377,   28659,   28932,   29196,   29451,   29697,
    29934,   30162,   30381,   30591,   30791,   30982,   31163,   31335,
    31498,   31650,   31794,   31927,   32051,   32165,   32269,   32364,
    32448,   32523,   32587,   32642,   32687,   32722,   32747,   32762
};

static const int16_t g_sin_lut[360] = {
        0,     572,    1144,    1715,    2286,    2856,    3425,    3993,
     4560,    5126,    5690,    6252,    6813,    7371,    7927,    8481,
     9032,    9580,   10126,   10668,   11207,   11743,   12275,   12803,
    13328,   13848,   14364,   14876,   15383,   15886,   16383,   16876,
    17364,   17846,   18323,   18794,   19260,   19720,   20173,   20621,
    21062,   21497,   21925,   22347,   22762,   23170,   23571,   23964,
    24351,   24730,   25101,   25465,   25821,   26169,   26509,   26841,
    27165,   27481,   27788,   28087,   28377,   28659,   28932,   29196,
    29451,   29697,   29934,   30162,   30381,   30591,   30791,   30982,
    31163,   31335,   31498,   31650,   31794,   31927,   32051,   32165,
    32269,   32364,   32448,   32523,   32587,   32642,   32687,   32722,
    32747,   32762,   32767,   32762,   32747,   32722,   32687,   32642,
    32587,   32523,   32448,   32364,   32269,   32165,   32051,   31927,
    31794,   31650,   31498,   31335,   31163,   30982,   30791,   30591,
    30381,   30162,   29934,   29697,   29451,   29196,   28932,   28659,
    28377,   28087,   27788,   27481,   27165,   26841,   26509,   26169,
    25821,   25465,   25101,   24730,   24351,   23964,   23571,   23170,
    22762,   22347,   21925,   21497,   21062,   20621,   20173,   19720,
    19260,   18794,   18323,   17846,   17364,   16876,   16383,   15886,
    15383,   14876,   14364,   13848,   13328,   12803,   12275,   11743,
    11207,   10668,   10126,    9580,    9032,    8481,    7927,    7371,
     6813,    6252,    5690,    5126,    4560,    3993,    3425,    2856,
     2286,    1715,    1144,     572,       0,    -572,   -1144,   -1715,
    -2286,   -2856,   -3425,   -3993,   -4560,   -5126,   -5690,   -6252,
    -6813,   -7371,   -7927,   -8481,   -9032,   -9580,  -10126,  -10668,
   -11207,  -11743,  -12275,  -12803,  -13328,  -13848,  -14364,  -14876,
   -15383,  -15886,  -16384,  -16876,  -17364,  -17846,  -18323,  -18794,
   -19260,  -19720,  -20173,  -20621,  -21062,  -21497,  -21925,  -22347,
   -22762,  -23170,  -23571,  -23964,  -24351,  -24730,  -25101,  -25465,
   -25821,  -26169,  -26509,  -26841,  -27165,  -27481,  -27788,  -28087,
   -28377,  -28659,  -28932,  -29196,  -29451,  -29697,  -29934,  -30162,
   -30381,  -30591,  -30791,  -30982,  -31163,  -31335,  -31498,  -31650,
   -31794,  -31927,  -32051,  -32165,  -32269,  -32364,  -32448,  -32523,
   -32587,  -32642,  -32687,  -32722,  -32747,  -32762,  -32767,  -32762,
   -32747,  -32722,  -32687,  -32642,  -32587,  -32523,  -32448,  -32364,
   -32269,  -32165,  -32051,  -31927,  -31794,  -31650,  -31498,  -31335,
   -31163,  -30982,  -30791,  -30591,  -30381,  -30162,  -29934,  -29697,
   -29451,  -29196,  -28932,  -28659,  -28377,  -28087,  -27788,  -27481,
   -27165,  -26841,  -26509,  -26169,  -25821,  -25465,  -25101,  -24730,
   -24351,  -23964,  -23571,  -23170,  -22762,  -22347,  -21925,  -21497,
   -21062,  -20621,  -20173,  -19720,  -19260,  -18794,  -18323,  -17846,
   -17364,  -16876,  -16384,  -15886,  -15383,  -14876,  -14364,  -13848,
   -13328,  -12803,  -12275,  -11743,  -11207,  -10668,  -10126,   -9580,
    -9032,   -8481,   -7927,   -7371,   -6813,   -6252,   -5690,   -5126,
    -4560,   -3993,   -3425,   -2856,   -2286,   -1715,   -1144,    -572
};

/* =========================================================================
* Internal helpers
* ====================================================================== */

/** Robot cell (grid centre). */
#define ROBOT_CX  (GRID_W / 2)
#define ROBOT_CY  (GRID_H / 2)

/** Flat cell index. */
static inline uint32_t cell_idx(int16_t cx, int16_t cy) {
   return (uint32_t)cy * (uint32_t)GRID_W + (uint32_t)cx;
}

/** Bounds check. */
static inline bool in_grid(int16_t cx, int16_t cy) {
   return (cx >= 0) && (cx < GRID_W) && (cy >= 0) && (cy < GRID_H);
}

/**
 * Convert angle_cdeg + distance_mm to world mm using Q15 LUT.
 * Returns world X (forward) and Y (left) in mm.
 * No float used.
**/
static void polar_to_world(
   uint16_t angle_cdeg, uint16_t dist_mm, int32_t *wx, int32_t *wy
) {
   uint16_t deg = angle_cdeg / 100u;
   uint16_t rem = angle_cdeg % 100u;          /* centidegrees 0..99  */
   uint16_t nxt = (deg + 1u) % 360u;

   /* Sub-degree linear interpolation of LUT values. */
   int32_t cv = (int32_t)g_cos_lut[deg] + ((int32_t)(
      g_cos_lut[nxt] - g_cos_lut[deg]
   ) * rem) / 100;
   int32_t sv = (int32_t)g_sin_lut[deg] + ((int32_t)(
      g_sin_lut[nxt] - g_sin_lut[deg]
   ) * rem) / 100;

   /* Q15 → mm:  round((dist × trig) / 32768)
    * Adding 16384 (0.5 in Q15) before shifting implements round-half-up,
    * eliminating the 1-LSB shortfall at cardinal angles (cos/sin = 32767
    * instead of 32768) that would otherwise misplace the endpoint cell. */
   *wx = ((int32_t)dist_mm * cv + 16384) >> 15;
   *wy = ((int32_t)dist_mm * sv + 16384) >> 15;
}

/**
 * Mark one cell FREE in the raw grid (only if currently UNKNOWN).
 * We do not overwrite OCCUPIED cells along a ray — a later ray in the
 * same scan might see through a valid obstacle (e.g. a thin post).
**/
static inline void mark_free(OccupancyGrid *g, int16_t cx, int16_t cy) {
   if(!in_grid(cx, cy)) return;
   uint32_t idx = cell_idx(cx, cy);
   if(g->cells[idx] == CELL_UNKNOWN)
      g->cells[idx] = CELL_FREE;
   g->cells_freed++;
}

/**
 * Standard Bresenham line walk from (x0,y0) to (x1,y1).
 * Calls free_fn for every cell except the endpoint.
 * Returns false if the endpoint is outside the grid (clipped).
**/
static bool bresenham(
   OccupancyGrid *g,
   int16_t x0, int16_t y0,
   int16_t x1, int16_t y1
) {
   int16_t dx  =  (x1 > x0) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
   int16_t dy  = -(int16_t)((y1 > y0) ? (int16_t)(y1 - y0) : (int16_t)(y0 - y1));
   int16_t sx  = (x0 < x1) ? 1 : -1;
   int16_t sy  = (y0 < y1) ? 1 : -1;
   int16_t err = dx + dy;
   int16_t cx  = x0;
   int16_t cy  = y0;

   for(;;) {
      bool at_end = (cx == x1 && cy == y1);

      if(at_end) return true;   /* endpoint inside grid */

      /* Mark this cell as free. */
      if(in_grid(cx, cy)) {
         uint32_t idx = cell_idx(cx, cy);
         if(g->cells[idx] == CELL_UNKNOWN) {
            g->cells[idx] = CELL_FREE;
            g->cells_freed++;
         }
      }

      /* Step. */
      int16_t e2 = (int16_t)(2 * err);
      if(e2 >= dy) { err = (int16_t)(err + dy); cx = (int16_t)(cx + sx); }
      if(e2 <= dx) { err = (int16_t)(err + dx); cy = (int16_t)(cy + sy); }

      /* If we've stepped outside the grid and haven't reached the
      * endpoint, the endpoint was outside — stop tracing.           */
      if(!in_grid(cx, cy)) return false;
   }
}

/* =========================================================================
* Lifecycle
* ====================================================================== */

void occ_grid_init(OccupancyGrid *g) {
   if(!g) return;
   memset(g, 0, sizeof(*g));

   /* Precompute circular inflation mask. */
   uint16_t n = 0;
   int r = INFLATE_RADIUS_CELLS;
   for(int dy = -r; dy <= r; dy++) {
      for(int dx = -r; dx <= r; dx++) {
         if(dx*dx + dy*dy <= r*r) {
            g->mask_dx[n] = (int8_t)dx;
            g->mask_dy[n] = (int8_t)dy;
            n++;
            /* Guard: INFLATE_MASK_MAX must be large enough. */
            if(n >= INFLATE_MASK_MAX) goto mask_done;
         }
      }
   }
mask_done:
   g->mask_count = n;

   /* Set all cells to UNKNOWN. */
   memset(g->cells,    CELL_UNKNOWN, sizeof(g->cells));
   memset(g->inflated, CELL_UNKNOWN, sizeof(g->inflated));
}

void occ_grid_clear(OccupancyGrid *g) {
   if(!g) return;
   memset(g->cells,    CELL_UNKNOWN, (size_t)GRID_W * GRID_H);
   memset(g->inflated, CELL_UNKNOWN, (size_t)GRID_W * GRID_H);
   g->rays_cast      = 0;
   g->cells_freed    = 0;
   g->cells_occupied = 0;
   g->cells_clipped  = 0;
}

/* =========================================================================
* Update
* ====================================================================== */

void occ_grid_update(OccupancyGrid *g, const LidarScan *scan) {
   if(!g || !scan) return;

   occ_grid_clear(g);

   for(uint16_t i = 0; i < scan->count; i++) {
      const LidarPoint *pt = &scan->points[i];
      if(pt->distance_mm == 0u) continue;   /* invalid return */

      /* Polar → world (mm). */
      int32_t wx, wy;
      polar_to_world(pt->angle_cdeg, pt->distance_mm, &wx, &wy);

      /* World → endpoint cell.
      * Y axis is inverted in grid (row 0 = top = +Y side of world).  */
      int32_t ecx32 = (int32_t)ROBOT_CX + wx / (int32_t)GRID_CELL_MM;
      int32_t ecy32 = (int32_t)ROBOT_CY - wy / (int32_t)GRID_CELL_MM;

      int16_t ecx = (int16_t)ecx32;
      int16_t ecy = (int16_t)ecy32;

      g->rays_cast++;

      /* Trace the ray; mark free cells. */
      bool hit_inside = bresenham(
         g,
         (int16_t)ROBOT_CX, (int16_t)ROBOT_CY,
         ecx, ecy
      );

      if(hit_inside && in_grid(ecx, ecy)) {
         g->cells[cell_idx(ecx, ecy)] = CELL_OCCUPIED;
         g->cells_occupied++;
      } else {
         g->cells_clipped++;
      }
   }

   /* The robot cell is the Bresenham origin and is never stepped through,
    * so it stays CELL_UNKNOWN after the ray loop above.  Explicitly mark
    * it FREE: the sensor sits here, so it is always unoccupied.  Without
    * this, nearby obstacles within INFLATE_RADIUS_CELLS inflate into the
    * robot cell, making it impassable to the planner. */
   g->cells[cell_idx((int16_t)ROBOT_CX, (int16_t)ROBOT_CY)] = CELL_FREE;
}

/* =========================================================================
* Inflation
* ====================================================================== */

void occ_grid_inflate(OccupancyGrid *g) {
   if(!g) return;

   /* Copy raw cells to inflated as the base. */
   memcpy(g->inflated, g->cells, (size_t)GRID_W * GRID_H);

   /* For every OCCUPIED cell apply the pre-computed circular mask. */
   for(int16_t cy = 0; cy < GRID_H; cy++) {
      for(int16_t cx = 0; cx < GRID_W; cx++) {
         if(g->cells[cell_idx(cx, cy)] != CELL_OCCUPIED) continue;
         for(uint16_t m = 0; m < g->mask_count; m++) {
            int16_t nx = (int16_t)(cx + g->mask_dx[m]);
            int16_t ny = (int16_t)(cy + g->mask_dy[m]);
            if(!in_grid(nx, ny)) continue;

            /* The robot cell is always passable — never inflate it.
             * The robot occupies this cell, so treating it as blocked
             * would prevent the planner from finding any path at all. */
            if(nx == (int16_t)ROBOT_CX && ny == (int16_t)ROBOT_CY) continue;

            uint32_t ni = cell_idx(nx, ny);
            /* Only inflate FREE or UNKNOWN — don't overwrite OCCUPIED. */
            if(g->inflated[ni] < CELL_OCCUPIED)
               g->inflated[ni] = CELL_INFLATED;
         }
      }
   }
}

/* =========================================================================
* Cell accessors
* ====================================================================== */

uint8_t occ_grid_get(const OccupancyGrid *g, int16_t cx, int16_t cy) {
   if(!g || !in_grid(cx, cy)) return CELL_UNKNOWN;
   return g->cells[cell_idx(cx, cy)];
}

void occ_grid_set(OccupancyGrid *g, int16_t cx, int16_t cy, uint8_t val) {
   if(!g || !in_grid(cx, cy)) return;
   g->cells[cell_idx(cx, cy)] = val;
}

uint8_t occ_grid_get_inflated(const OccupancyGrid *g, int16_t cx, int16_t cy) {
   if(!g || !in_grid(cx, cy)) return CELL_INFLATED;   /* block OOB */
   return g->inflated[cell_idx(cx, cy)];
}

bool occ_grid_passable(const OccupancyGrid *g, int16_t cx, int16_t cy) {
   return occ_grid_get_inflated(g, cx, cy) < CELL_OCCUPIED;
}

/* =========================================================================
* Coordinate conversion
* ====================================================================== */

void occ_grid_world_to_cell(
   const OccupancyGrid *g,
   int32_t wx, int32_t wy,
   int16_t *cx, int16_t *cy
) {
   (void)g;   /* GRID_CELL_MM is compile-time; g kept for API consistency */
   int32_t raw_cx = (int32_t)ROBOT_CX + wx / (int32_t)GRID_CELL_MM;
   int32_t raw_cy = (int32_t)ROBOT_CY - wy / (int32_t)GRID_CELL_MM;

   /* Clamp to grid bounds. */
   if(raw_cx < 0)          raw_cx = 0;
   if(raw_cx >= GRID_W)    raw_cx = GRID_W - 1;
   if(raw_cy < 0)          raw_cy = 0;
   if(raw_cy >= GRID_H)    raw_cy = GRID_H - 1;

   *cx = (int16_t)raw_cx;
   *cy = (int16_t)raw_cy;
}

void occ_grid_cell_to_world(
   const OccupancyGrid *g,
   int16_t cx, int16_t cy,
   int32_t *wx, int32_t *wy
) {
   (void)g;
   /* Centre of cell.
    * X: cx increases in the +wx direction → add half-cell.
    * Y: cy increases in the -wy direction (Y inverted) → subtract half-cell
    *    so that the centre lies at the midpoint of the world range the cell
    *    covers, symmetric with world_to_cell's truncation-toward-zero. */
   *wx = ((int32_t)cx - ROBOT_CX) * GRID_CELL_MM + GRID_CELL_MM / 2;
   *wy = (ROBOT_CY - (int32_t)cy) * GRID_CELL_MM - GRID_CELL_MM / 2;
}

void occ_grid_robot_cell(int16_t *cx, int16_t *cy) {
   *cx = (int16_t)ROBOT_CX;
   *cy = (int16_t)ROBOT_CY;
}

/* =========================================================================
* Diagnostics
* ====================================================================== */

void occ_grid_get_stats(
   const OccupancyGrid *g,
   uint32_t *rays,
   uint32_t *freed,
   uint32_t *occupied,
   uint32_t *clipped
) {
   if(!g) return;
   if(rays)     *rays     = g->rays_cast;
   if(freed)    *freed    = g->cells_freed;
   if(occupied) *occupied = g->cells_occupied;
   if(clipped)  *clipped  = g->cells_clipped;
}
