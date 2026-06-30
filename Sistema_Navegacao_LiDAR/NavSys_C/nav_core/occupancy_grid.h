/**
 * @file occupancy_grid.h
 * @brief 2-D occupancy grid — public interface.
 *
 * Maintains a robot-centric local map built from LiDAR scans.
 *
 * ── Coordinate systems ────────────────────────────────────────────────────
 *
 *  World frame (mm)  : right-handed, X = forward (LiDAR 0°), Y = left.
 *  Grid frame (cells): integer column cx ∈ [0, GRID_W) and
 *                      integer row    cy ∈ [0, GRID_H).
 *  Robot position    : always cell (GRID_W/2, GRID_H/2) = grid centre.
 *
 *  World origin is the robot.  Positive X points to LiDAR angle 0°.
 *  Positive Y is 90° counter-clockwise from X (left of the robot).
 *
 *  Conversion (world → cell):
 *    cx = GRID_W/2 + wx_mm / GRID_CELL_MM
 *    cy = GRID_H/2 - wy_mm / GRID_CELL_MM   (Y-axis flipped for row order)
 *
 * ── Cell states ───────────────────────────────────────────────────────────
 *
 *  CELL_FREE      (0)   : confirmed clear by a ray that passed through.
 *  CELL_UNKNOWN (128)   : never observed (initial state).
 *  CELL_OCCUPIED (200)  : ray endpoint — obstacle measured here.
 *  CELL_INFLATED (255)  : within robot-radius of an obstacle; impassable.
 *
 * ── Typical usage per scan cycle ─────────────────────────────────────────
 *
 *   occ_grid_clear(&g);           // reset to CELL_UNKNOWN
 *   occ_grid_update(&g, &scan);   // Bresenham ray-cast
 *   occ_grid_inflate(&g);         // expand obstacles by robot radius
 *   // → grid ready for A*
 *
 * ── A* interface ─────────────────────────────────────────────────────────
 *
 *  A cell is traversable by the planner if and only if
 *    occ_grid_passable(&g, cx, cy) == true
 *  i.e. cell value < CELL_OCCUPIED.  CELL_INFLATED is therefore blocked.
**/

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include "nav_types.h"
#include "nav_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Grid struct
 *
 * Flat row-major array: index = cy * GRID_W + cx.
 * Declared with fixed-size arrays so the struct can be placed in BSS
 * (global) or on a static pool — no heap allocation required.
 * ====================================================================== */

typedef struct {
    uint8_t  cells[GRID_W * GRID_H];  /**< Raw cell values.                */
    uint8_t  inflated[GRID_W * GRID_H]; /**< Inflated copy for planning.   */

    /* Inflation mask — precomputed once at init.
     * Stores (dx, dy) offsets for all cells within INFLATE_RADIUS_CELLS. */
    int8_t   mask_dx[INFLATE_MASK_MAX];
    int8_t   mask_dy[INFLATE_MASK_MAX];
    uint16_t mask_count;

    /* Diagnostic counters (last occ_grid_update() call). */
    uint32_t rays_cast;        /**< Total rays traced.                      */
    uint32_t cells_freed;      /**< Cells marked FREE by ray-cast.          */
    uint32_t cells_occupied;   /**< Cells marked OCCUPIED by ray-cast.      */
    uint32_t cells_clipped;    /**< Ray endpoints outside grid (clipped).   */
} OccupancyGrid;

/* =========================================================================
 * Lifecycle
 * ====================================================================== */

/**
 * @brief Initialise all fields and pre-compute the inflation mask.
 *
 * Must be called once before any other function.
 * Safe to call again to hard-reset the grid.
 *
 * @param g  Non-NULL grid pointer (stack, BSS, or heap — all fine).
**/
void occ_grid_init(OccupancyGrid *g);

/**
 * @brief Reset all cells to CELL_UNKNOWN.
 *
 * Faster than occ_grid_init(); does not recompute the inflation mask.
 * Call once per scan cycle before occ_grid_update().
 *
 * @param g  Initialised grid.
**/
void occ_grid_clear(OccupancyGrid *g);

/* =========================================================================
 * Update (ray-cast)
 * ====================================================================== */

/**
 * @brief Update the grid from a filtered LiDAR scan.
 *
 * For each point in @p scan:
 *   – Traces a Bresenham line from the robot cell to the endpoint cell.
 *   – Marks every cell on the line (exclusive of endpoint) as CELL_FREE.
 *   – Marks the endpoint cell as CELL_OCCUPIED.
 *   – Points whose endpoint falls outside the grid are clipped:
 *     the free cells along the ray (within the grid) are still marked.
 *
 * Calls occ_grid_clear() internally — no need to call it separately.
 * Does NOT inflate; call occ_grid_inflate() afterwards.
 *
 * Uses the Q15 integer trig LUT; no floating-point arithmetic.
 *
 * @param g     Initialised grid.
 * @param scan  Filtered scan (run scan_filter_apply() first).
**/
void occ_grid_update(OccupancyGrid *g, const LidarScan *scan);

/* =========================================================================
 * Inflation
 * ====================================================================== */

/**
 * @brief Build the inflated grid from the raw cells.
 *
 * Copies cells[] to inflated[], then expands every CELL_OCCUPIED cell by
 * INFLATE_RADIUS_CELLS using the precomputed mask.  Expanded cells are
 * written as CELL_INFLATED.
 *
 * Must be called after occ_grid_update() and before planning.
 * The raw cells[] array is unchanged.
 *
 * @param g  Updated grid (occ_grid_update() already called).
**/
void occ_grid_inflate(OccupancyGrid *g);

/* =========================================================================
 * Cell accessors
 * ====================================================================== */

/**
 * @brief Read a raw (pre-inflation) cell value.
 *
 * Returns CELL_UNKNOWN for out-of-bounds coordinates.
**/
uint8_t occ_grid_get(const OccupancyGrid *g, int16_t cx, int16_t cy);

/**
 * @brief Write a raw cell value.
 *
 * Out-of-bounds writes are silently ignored.
**/
void occ_grid_set(OccupancyGrid *g, int16_t cx, int16_t cy, uint8_t val);

/**
 * @brief Read an inflated cell value (for the A* planner).
 *
 * Returns CELL_INFLATED (blocked) for out-of-bounds coordinates, so the
 * planner naturally stays inside the grid without bounds checks.
**/
uint8_t occ_grid_get_inflated(const OccupancyGrid *g, int16_t cx, int16_t cy);

/**
 * @brief Return true if the planner may traverse this cell.
 *
 * A cell is passable if its inflated value is < CELL_OCCUPIED.
 * Equivalently: it is CELL_FREE or CELL_UNKNOWN.
**/
bool occ_grid_passable(const OccupancyGrid *g, int16_t cx, int16_t cy);

/* =========================================================================
 * Coordinate conversion
 * ====================================================================== */

/**
 * @brief Convert world coordinates (mm, robot-centred) to grid cell.
 *
 * Result is clamped to [0, GRID_W-1] × [0, GRID_H-1].
 *
 * @param g    Grid (needed for cell size).
 * @param wx   World X in mm (positive = LiDAR 0° direction).
 * @param wy   World Y in mm (positive = left of robot).
 * @param cx   Output cell column.
 * @param cy   Output cell row.
**/
void occ_grid_world_to_cell(const OccupancyGrid *g,
                            int32_t wx, int32_t wy,
                            int16_t *cx, int16_t *cy);

/**
 * @brief Convert a grid cell to world coordinates (mm, robot-centred).
 *
 * Returns the centre of the cell.
 *
 * @param g    Grid.
 * @param cx   Cell column.
 * @param cy   Cell row.
 * @param wx   Output world X in mm.
 * @param wy   Output world Y in mm.
**/
void occ_grid_cell_to_world(const OccupancyGrid *g,
                            int16_t cx, int16_t cy,
                            int32_t *wx, int32_t *wy);

/**
 * @brief Return the robot's cell position (always grid centre).
 *
 * Convenience for path planning: start cell = robot cell.
**/
void occ_grid_robot_cell(int16_t *cx, int16_t *cy);

/* =========================================================================
 * Diagnostics
 * ====================================================================== */

/**
 * @brief Copy update diagnostics from the last occ_grid_update() call.
 *
 * @param g          Grid.
 * @param rays       Optional: total rays traced (may be NULL).
 * @param freed      Optional: cells marked FREE.
 * @param occupied   Optional: cells marked OCCUPIED.
 * @param clipped    Optional: rays clipped at grid boundary.
**/
void occ_grid_get_stats(const OccupancyGrid *g,
                        uint32_t *rays,
                        uint32_t *freed,
                        uint32_t *occupied,
                        uint32_t *clipped);

#ifdef __cplusplus
}
#endif

#endif /* OCCUPANCY_GRID_H */