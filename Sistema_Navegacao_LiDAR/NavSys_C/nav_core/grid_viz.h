/**
 * @file grid_viz.h
 * @brief ASCII + ANSI occupancy grid visualiser — Linux debug only.
 *
 * Renders an OccupancyGrid to a terminal using ANSI colour codes.
 * Never included on ESP32 builds (guarded by HAL_PLATFORM).
 *
 * Characters used:
 *   '.'  CELL_FREE      (dim white)
 *   ' '  CELL_UNKNOWN   (background)
 *   '#'  CELL_OCCUPIED  (bright red)
 *   '+'  CELL_INFLATED  (yellow)
 *   'R'  Robot position (bright cyan)
 *   '*'  Path cell      (bright green)
 *
 * Downsamples the grid to fit the terminal if the grid is wider than
 * VIZ_MAX_COLS characters.
**/

#ifndef GRID_VIZ_H
#define GRID_VIZ_H

/* Only compiled on Linux — entire header is a no-op on embedded targets. */
#if !defined(EMBEDDED)

#include <stdint.h>
#include <stdbool.h>
#include "occupancy_grid.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum terminal columns the visualiser will use. */
#define VIZ_MAX_COLS  100
/** Maximum terminal rows the visualiser will use. */
#define VIZ_MAX_ROWS   50

/**
 * @brief Optional path overlay (may be NULL to suppress path drawing).
 *
 * Mirrors the NavPath layout from Phase 3; forward-declared here so
 * grid_viz.h has no dependency on astar.h before it exists.
**/
typedef struct {
    int16_t  cx;
    int16_t  cy;
} VizPathCell;

typedef struct {
    const VizPathCell *cells;
    uint16_t           count;
} VizPath;

/* =========================================================================
 * Visualiser configuration
 * ====================================================================== */

typedef struct {
    bool use_colour;     /**< false → plain ASCII (pipe-friendly).         */
    bool show_inflated;  /**< true → render g->inflated; false → g->cells. */
    bool show_stats;     /**< Print ray-cast stats below the grid.         */
    bool clear_screen;   /**< Emit ANSI clear-screen before each render.   */
    uint8_t downsample;  /**< 1 = full resolution, 2 = every 2nd cell, …  */
} VizCfg;

/**
 * @brief Fill @p cfg with reasonable defaults.
 *
 * Colour is enabled when stdout is a TTY.
**/
void viz_cfg_default(VizCfg *cfg);

/* =========================================================================
 * Render functions
 * ====================================================================== */

/**
 * @brief Render the occupancy grid to stdout.
 *
 * @param g     Grid to render (must have been updated).
 * @param path  Optional path overlay (may be NULL).
 * @param cfg   Render configuration (may be NULL for defaults).
**/
void viz_render(
    const OccupancyGrid *g,
    const VizPath      *path,
    const VizCfg       *cfg
);

/**
 * @brief Print a one-line statistics summary to stdout.
 *
 * Shows scan count, free/occupied/unknown cell percentages,
 * and the last occ_grid_update() ray-cast counters.
 *
 * @param g        Grid.
 * @param scan_no  Scan sequence number (caller-maintained).
**/
void viz_print_stats(const OccupancyGrid *g, uint32_t scan_no);

/**
 * @brief Dump the raw grid as a CSV file (one row per line, values 0–255).
 *
 * Useful for post-processing with Python tools.
 *
 * @param g     Grid.
 * @param path  File path to write.
 * @return      true on success.
**/
bool viz_dump_csv(const OccupancyGrid *g, const char *path);

#ifdef __cplusplus
}
#endif

#endif /* !EMBEDDED */
#endif /* GRID_VIZ_H */