/**
 * @file astar.h
 * @brief A* path planner on the occupancy grid — public interface.
 *
 * Plans a collision-free path from the robot cell to a goal cell using
 * the inflated occupancy grid produced by occ_grid_inflate().
 *
 * ── Algorithm ─────────────────────────────────────────────────────────────
 *
 *  Standard A* with an 8-connected grid and an octile-distance heuristic.
 *
 *  Move costs:
 *    Cardinal  (N/S/E/W) : 100
 *    Diagonal  (NE/…/SW) : 141  (≈ 100√2, rounded)
 *
 *  Heuristic (octile distance — admissible and consistent):
 *    h(n) = 100·max(|dx|,|dy|) + 41·min(|dx|,|dy|)
 *
 *  Passability:
 *    occ_grid_passable(g, cx, cy) — true iff inflated value < CELL_OCCUPIED.
 *    Out-of-bounds cells return CELL_INFLATED from occ_grid_get_inflated(),
 *    so the planner never needs explicit bounds checks.
 *
 * ── Memory ────────────────────────────────────────────────────────────────
 *
 *  All large arrays are file-scope statics inside astar.c (BSS).  The
 *  AStarCtx struct is opaque and tiny — it holds only a pointer to the
 *  static state.  This ensures zero heap usage and zero stack pressure on
 *  ESP32-C6, where both are scarce.
 *
 *  On Linux the same static approach is used for API symmetry.
 *  Only one AStarCtx may exist at a time (embedded constraint).
 *
 * ── Path output ───────────────────────────────────────────────────────────
 *
 *  NavPath.cells[] uses VizPathCell {int16_t cx, cy}, the same type that
 *  viz_render() expects via VizPath, so path overlay requires no conversion:
 *
 *    VizPath vp = { .cells = path.cells, .count = path.count };
 *    viz_render(&grid, &vp, &cfg);
 *
 *  The path is ordered start → goal.  cells[0] == start cell (robot),
 *  cells[count-1] == goal cell.
 *
 * ── Typical usage ─────────────────────────────────────────────────────────
 *
 * @code
 *   AStarCtx *ctx = astar_create();
 *
 *   // Every scan cycle:
 *   occ_grid_update(&grid, &scan);
 *   occ_grid_inflate(&grid);
 *
 *   int16_t sx, sy;
 *   occ_grid_robot_cell(&sx, &sy);          // always (ROBOT_CX, ROBOT_CY)
 *
 *   NavPath path;
 *   PlanResult r = astar_plan(ctx, &grid, sx, sy, goal_cx, goal_cy, &path);
 *   if (r == PLAN_OK) {
 *       // path.cells[0..path.count-1] holds the route
 *       VizPath vp = { .cells = path.cells, .count = path.count };
 *       viz_render(&grid, &vp, NULL);
 *   }
 *
 *   astar_destroy(ctx);
 * @endcode
 **/

#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include <stdbool.h>

#include "nav_config.h"
#include "occupancy_grid.h"
#include "grid_viz.h"        /* VizPathCell — reused to avoid duplicate type */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Path result types
 * ====================================================================== */

/**
 * @brief Ordered sequence of grid cells forming the planned path.
 *
 * cells[0]         == start cell (robot position).
 * cells[count - 1] == goal cell.
 *
 * VizPathCell is {int16_t cx, cy} — identical layout to what viz_render()
 * expects via VizPath, so no type conversion is required for display:
 *
 *   VizPath vp = { .cells = path.cells, .count = path.count };
 *   viz_render(&grid, &vp, NULL);
**/
typedef struct {
    VizPathCell cells[ASTAR_MAX_PATH_CELLS];  /**< Path cells, start→goal.  */
    uint16_t    count;                        /**< Number of cells in path. */
    bool        found;                        /**< true iff PLAN_OK.        */
} NavPath;

/**
 * @brief Return code from astar_plan().
**/
typedef enum {
    PLAN_OK            = 0,  /**< Path found; out_path is valid.             */
    PLAN_NO_PATH       = 1,  /**< Goal is unreachable from start.            */
    PLAN_INVALID_START = 2,  /**< Start cell is impassable or out-of-bounds. */
    PLAN_INVALID_GOAL  = 3,  /**< Goal cell is impassable or out-of-bounds.  */
} PlanResult;

/* =========================================================================
 * Opaque context handle
 * ====================================================================== */

/**
 * @brief Opaque A* planner context.
 *
 * Allocate with astar_create(); free with astar_destroy().
 * Only one instance may exist at a time (single static pool).
**/
typedef struct AStarCtx AStarCtx;

/* =========================================================================
 * Lifecycle
 * ====================================================================== */

/**
 * @brief Allocate and initialise the A* planner context.
 *
 * Returns a pointer to the single static planner instance.
 * Only one AStarCtx may exist at a time.  Calling astar_create() while
 * an instance already exists returns the same pointer (idempotent).
 *
 * @return  Non-NULL handle on success, NULL on internal error.
**/
AStarCtx *astar_create(void);

/**
 * @brief Release the planner context.
 *
 * Marks the static instance as free.  Safe to call with NULL (no-op).
 * The large BSS arrays are NOT zeroed — their contents are undefined
 * until the next call to astar_plan().
 *
 * @param ctx  Handle returned by astar_create(), or NULL.
**/
void astar_destroy(AStarCtx *ctx);

/* =========================================================================
 * Planning
 * ====================================================================== */

/**
 * @brief Plan a path from (sx, sy) to (gx, gy) on the inflated grid.
 *
 * Uses A* with 8-connectivity and an octile-distance heuristic.
 * Planning operates entirely on g->inflated[] via occ_grid_passable().
 *
 * The start cell is the robot cell; use occ_grid_robot_cell() to obtain it.
 * If the start equals the goal, PLAN_OK is returned with count=1.
 *
 * @param ctx       Non-NULL context from astar_create().
 * @param g         Occupancy grid after occ_grid_inflate() has been called.
 * @param sx, sy    Start cell (robot position).
 * @param gx, gy    Goal cell.
 * @param out_path  Output path; written only when PLAN_OK is returned.
 *                  The caller owns this struct; astar_plan() retains no
 *                  reference after returning.
 *
 * @return  PLAN_OK            — out_path contains a valid route.
 *          PLAN_NO_PATH       — no passable route exists.
 *          PLAN_INVALID_START — start cell is blocked or out-of-bounds.
 *          PLAN_INVALID_GOAL  — goal cell is blocked or out-of-bounds.
**/
PlanResult astar_plan(
    AStarCtx            *ctx,
    const OccupancyGrid *g,
    int16_t sx, int16_t sy,
    int16_t gx, int16_t gy,
    NavPath             *out_path
);

/* =========================================================================
 * Diagnostics
 * ====================================================================== */

/**
 * @brief Counters from the most recent astar_plan() call.
**/
typedef struct {
    uint32_t nodes_expanded;   /**< Cells popped from the open set.         */
    uint32_t nodes_generated;  /**< Neighbours pushed onto the open set.    */
    uint32_t path_length;      /**< Cells in the returned path (0 if none). */
    uint32_t path_cost;        /**< Total g-score of the path (move units). */
} AStarStats;

/**
 * @brief Copy diagnostics from the most recent astar_plan() call.
 *
 * @param ctx  Non-NULL context handle.
 * @param out  Destination; must not be NULL.
**/
void astar_get_stats(const AStarCtx *ctx, AStarStats *out);

#ifdef __cplusplus
}
#endif

#endif /* ASTAR_H */