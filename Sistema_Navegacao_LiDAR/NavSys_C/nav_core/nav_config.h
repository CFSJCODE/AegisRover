/**
 * @file nav_config.h
 * @brief Compile-time configuration for the navigation core.
 *
 * All tuneable constants that affect RAM layout, grid resolution,
 * sensor trust, and robot geometry live here.  Change a value and
 * recompile; no other source file needs editing.
 *
 * ESP32-C6 DRAM budget guidance
 * ──────────────────────────────
 *  Grid cells (raw + inflated)  : 2 × W × H bytes
 *  Trig LUT (cos + sin, int16)  : 2 × 360 × 2  = 1440 bytes  (.rodata / flash)
 *  LidarScan (WIP + output)     : 2 × 3608      ≈ 7.2 KB
 *  LidarParser struct           : ≈ 3.8 KB
 *  A* arrays (Phase 3)         : ~W×H×4 bytes
 *
 *  Default 150×150 grid → 22 500 bytes raw + 22 500 inflated = 45 KB.
 *  Total Phase-2 footprint ≈ 60 KB — comfortably within 512 KB.
 */

#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

/* =========================================================================
 * Grid dimensions
 *
 * GRID_W × GRID_H cells.  Each cell is one uint8_t.
 * Robot is always placed at cell (GRID_W/2, GRID_H/2).
 *
 * Physical coverage = GRID_W × GRID_CELL_MM by GRID_H × GRID_CELL_MM.
 * Default: 150 × 50 mm = 7 500 mm = 7.5 m square.
 * ====================================================================== */
#ifndef GRID_W
#  define GRID_W          150
#endif
#ifndef GRID_H
#  define GRID_H          150
#endif

/** Cell edge length in millimetres. */
#ifndef GRID_CELL_MM
#  define GRID_CELL_MM     50
#endif

/* =========================================================================
 * Obstacle inflation
 *
 * INFLATE_RADIUS_CELLS: obstacles are grown by this many cells in every
 * direction, effectively treating the robot as a point.  Set to
 * ceil(robot_radius_mm / GRID_CELL_MM).
 *
 * Default robot radius ~150 mm / 50 mm per cell = 3 cells.
 * ====================================================================== */
#ifndef INFLATE_RADIUS_CELLS
#  define INFLATE_RADIUS_CELLS   3
#endif

/* Worst-case inflate mask entries: circle of radius R → π R² + buffer.
 * (2R+1)² is a safe upper bound.                                         */
#define INFLATE_MASK_MAX  ((2*INFLATE_RADIUS_CELLS+1)*(2*INFLATE_RADIUS_CELLS+1))

/* =========================================================================
 * Scan filter defaults (overridable at runtime via ScanFilterCfg)
 * ====================================================================== */
#define FILTER_MIN_DIST_MM       100u   /**< Sensor blind zone.           */
#define FILTER_MAX_DIST_MM      6000u   /**< Maximum reliable range.      */
#define FILTER_MIN_INTENSITY       8u   /**< Drop weak returns.           */
#define FILTER_MEDIAN_WINDOW       3u   /**< 0 = disabled, 3 or 5 typical.*/

/* =========================================================================
 * Robot position within the grid
 *
 * The robot is always placed at the exact centre of the grid.
 * These macros are used by occupancy_grid.c, grid_viz.c, and astar.c.
 * ====================================================================== */
#define ROBOT_CX  (GRID_W / 2)
#define ROBOT_CY  (GRID_H / 2)

/* =========================================================================
 * Cell state values
 * ====================================================================== */
#define CELL_UNKNOWN   128u
#define CELL_FREE        0u
#define CELL_OCCUPIED  200u
#define CELL_INFLATED  255u   /**< Occupied + inflation margin.           */

/* =========================================================================
 * Bresenham update hysteresis
 *
 * A cell transitions FREE → OCCUPIED only after being hit by a ray
 * endpoint HIT_THRESH times in a single scan pass.  Since we clear and
 * rebuild each scan, this is effectively 1 (single-scan binary).
 * Kept as a named constant for future probabilistic extension.
 * ====================================================================== */
#define CELL_HIT_THRESH    1u

/* =========================================================================
 * A* path planner
 *
 * Maximum number of cells in a returned path (start → goal inclusive).
 * The worst-case shortest path across the full grid diagonal is
 * approximately sqrt(GRID_W² + GRID_H²) ≈ 212 cells for a 150×150 grid.
 * 512 provides a 2.4× headroom for non-straight paths through obstacles.
 * ====================================================================== */
#ifndef ASTAR_MAX_PATH_CELLS
#  define ASTAR_MAX_PATH_CELLS  512
#endif

#endif /* NAV_CONFIG_H */