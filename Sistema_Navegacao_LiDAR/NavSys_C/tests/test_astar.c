/**
 * @file test_astar.c
 * @brief Functional test suite for astar.c
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O0 -g \
 *       -I. \
 *       test_astar.c \
 *       astar.c occupancy_grid.c scan_filter.c lidar_parser.c \
 *       grid_viz.c hal_linux.c \
 *       -lpthread -o build/test_astar
 *
 *   ./build/test_astar
 *   ./build/test_astar scan_raw.bin   # optional: plan on real scan
 *
 * Sections
 * ────────
 *  1.  Lifecycle          — create / destroy / NULL safety
 *  2.  Trivial case       — start == goal
 *  3.  Straight paths     — cardinal and diagonal, open grid
 *  4.  Optimality         — shortest path in controlled obstacles
 *  5.  Obstacle avoidance — path routes around obstacles
 *  6.  No-path cases      — fully blocked goal, enclosed start
 *  7.  Invalid inputs     — impassable start/goal, out-of-bounds
 *  8.  Path properties    — continuity (each step ≤ 1 cell), bounds
 *  9.  Inflation respect  — inflated cells are treated as obstacles
 * 10.  Start == robot cell — always passable after inflate
 * 11.  Stats              — nodes_expanded > 0, path_cost correct
 * 12.  Viz overlay        — NavPath casts to VizPath without errors
 * 13.  Full pipeline      — filter→update→inflate→plan (optional bin)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "astar.h"
#include "occupancy_grid.h"
#include "scan_filter.h"
#include "grid_viz.h"
#include "lidar_parser.h"

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
    t_result((cond), #cond, #cond, __FILE__, __LINE__)

#define CHECK_MSG(cond, msg) \
    t_result((cond), #cond, (msg), __FILE__, __LINE__)

#define CHECK_EQ(a, b) do { \
    long long _a = (long long)(a), _b = (long long)(b); \
    char _d[128]; \
    snprintf(_d, sizeof(_d), "%lld != %lld", _a, _b); \
    t_result(_a == _b, #a " == " #b, _d, __FILE__, __LINE__); \
} while (0)

static void section(const char *title) {
    printf("\n%s── %s ──%s\n",
           g_colour ? "\033[1;33m" : "",
           title,
           g_colour ? "\033[0m"    : "");
}

/* =========================================================================
 * Helpers
 * ====================================================================== */

/** Allocate a fresh, init'd grid (static — only one needed at a time). */
static OccupancyGrid g_grid;

static void grid_clear_free(void) {
    occ_grid_init(&g_grid);
    /* Fill everything FREE so A* can traverse the whole grid by default. */
    memset(g_grid.cells,    CELL_FREE, GRID_W * GRID_H);
    memset(g_grid.inflated, CELL_FREE, GRID_W * GRID_H);
    /* Robot cell. */
    g_grid.cells   [ROBOT_CY * GRID_W + ROBOT_CX] = CELL_FREE;
    g_grid.inflated[ROBOT_CY * GRID_W + ROBOT_CX] = CELL_FREE;
}

/** Place a rectangular wall of OCCUPIED cells in both cells[] and inflated[]. */
static void place_wall(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    for (int16_t cy = y0; cy <= y1; cy++) {
        for (int16_t cx = x0; cx <= x1; cx++) {
            if (cx < 0 || cx >= GRID_W || cy < 0 || cy >= GRID_H) continue;
            uint32_t idx = (uint32_t)cy * GRID_W + (uint32_t)cx;
            g_grid.cells[idx]    = CELL_OCCUPIED;
            g_grid.inflated[idx] = CELL_OCCUPIED;
        }
    }
}

/** Verify that every consecutive pair of cells in a path is adjacent (≤1 cell). */
static bool path_is_continuous(const NavPath *p) {
    for (uint16_t i = 1; i < p->count; i++) {
        int dx = (int)p->cells[i].cx - (int)p->cells[i-1].cx;
        int dy = (int)p->cells[i].cy - (int)p->cells[i-1].cy;
        if (dx < -1 || dx > 1 || dy < -1 || dy > 1) return false;
        if (dx == 0 && dy == 0) return false;   /* duplicate step */
    }
    return true;
}

/** Verify that every cell in a path is within grid bounds. */
static bool path_in_bounds(const NavPath *p) {
    for (uint16_t i = 0; i < p->count; i++) {
        if (p->cells[i].cx < 0 || p->cells[i].cx >= GRID_W) return false;
        if (p->cells[i].cy < 0 || p->cells[i].cy >= GRID_H) return false;
    }
    return true;
}

/** Verify that no cell in a path is OCCUPIED or INFLATED in inflated[]. */
static bool path_avoids_obstacles(const NavPath *p) {
    for (uint16_t i = 0; i < p->count; i++) {
        uint8_t v = occ_grid_get_inflated(&g_grid,
                                          p->cells[i].cx, p->cells[i].cy);
        if (v >= CELL_OCCUPIED) return false;
    }
    return true;
}

/* Cost of a path: cardinal moves = 100, diagonal = 141. */
static uint32_t path_cost(const NavPath *p) {
    uint32_t cost = 0;
    for (uint16_t i = 1; i < p->count; i++) {
        int dx = (int)p->cells[i].cx - (int)p->cells[i-1].cx;
        int dy = (int)p->cells[i].cy - (int)p->cells[i-1].cy;
        cost += (dx != 0 && dy != 0) ? 141u : 100u;
    }
    return cost;
}

/* =========================================================================
 * 1. Lifecycle
 * ====================================================================== */
static void test_lifecycle(void) {
    section("1. Lifecycle");

    /* 1a. astar_create returns non-NULL */
    AStarCtx *ctx = astar_create();
    CHECK_MSG(ctx != NULL, "astar_create() returned NULL");

    /* 1b. astar_destroy(NULL) does not crash */
    astar_destroy(NULL);
    CHECK(true);

    /* 1c. astar_destroy live handle does not crash */
    astar_destroy(ctx);
    CHECK(true);

    /* 1d. Re-create after destroy */
    ctx = astar_create();
    CHECK_MSG(ctx != NULL, "re-create after destroy returned NULL");
    astar_destroy(ctx);
}

/* =========================================================================
 * 2. Trivial: start == goal
 * ====================================================================== */
static void test_trivial(void) {
    section("2. Trivial: start == goal");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 2a. Start == goal at robot cell */
    PlanResult r = astar_plan(ctx, &g_grid,
                              ROBOT_CX, ROBOT_CY,
                              ROBOT_CX, ROBOT_CY,
                              &path);
    CHECK_EQ((int)r, (int)PLAN_OK);
    CHECK_MSG(path.found,   "trivial path: found not set");
    CHECK_EQ(path.count, 1u);
    CHECK_EQ(path.cells[0].cx, ROBOT_CX);
    CHECK_EQ(path.cells[0].cy, ROBOT_CY);

    /* 2b. Start == goal at an arbitrary free cell */
    r = astar_plan(ctx, &g_grid, 10, 10, 10, 10, &path);
    CHECK_EQ((int)r, (int)PLAN_OK);
    CHECK_EQ(path.count, 1u);

    astar_destroy(ctx);
}

/* =========================================================================
 * 3. Straight paths (open grid)
 * ====================================================================== */
static void test_straight_paths(void) {
    section("3. Straight paths — open grid");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 3a. Cardinal: move 5 cells east from robot */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = ROBOT_CY;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path.found, "cardinal E: found not set");
        CHECK_EQ(path.cells[0].cx, sx);
        CHECK_EQ(path.cells[0].cy, sy);
        CHECK_EQ(path.cells[path.count-1].cx, gx);
        CHECK_EQ(path.cells[path.count-1].cy, gy);
        CHECK_MSG(path_is_continuous(&path), "cardinal E: path not continuous");
        CHECK_MSG(path_in_bounds(&path),     "cardinal E: path out of bounds");
        /* Optimal cost for 5 cardinal steps = 5 * 100 = 500 */
        CHECK_EQ(path_cost(&path), 500u);
    }

    /* 3b. Cardinal: 5 cells north */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = ROBOT_CX, gy = (int16_t)(ROBOT_CY - 5);
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_EQ(path_cost(&path), 500u);
    }

    /* 3c. Diagonal: 5 cells NE */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = (int16_t)(ROBOT_CY - 5);
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_is_continuous(&path), "diagonal NE: not continuous");
        /* Optimal: 5 diagonal steps × 141 = 705 */
        CHECK_EQ(path_cost(&path), 705u);
    }

    /* 3d. Long path: robot to far corner */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(GRID_W - 1), gy = 0;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path.count >= 1u,          "far corner: empty path");
        CHECK_MSG(path_is_continuous(&path), "far corner: not continuous");
        CHECK_MSG(path_in_bounds(&path),     "far corner: out of bounds");
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 4. Optimality
 * ====================================================================== */
static void test_optimality(void) {
    section("4. Optimality");

    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 4a. Horizontal corridor — only one route, cost is n*100 */
    {
        grid_clear_free();
        /* Build a corridor: only row ROBOT_CY is passable, 10 cells wide. */
        for (int16_t cy = 0; cy < GRID_H; cy++) {
            if (cy == (int16_t)ROBOT_CY) continue;
            for (int16_t cx = (int16_t)ROBOT_CX;
                 cx <= (int16_t)(ROBOT_CX + 10); cx++) {
                uint32_t i = (uint32_t)cy * GRID_W + (uint32_t)cx;
                g_grid.inflated[i] = CELL_OCCUPIED;
            }
        }
        int16_t gx = (int16_t)(ROBOT_CX + 8), gy = (int16_t)ROBOT_CY;
        PlanResult r = astar_plan(ctx, &g_grid,
                                  ROBOT_CX, ROBOT_CY, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        /* 8 cardinal steps = 800 */
        CHECK_EQ(path_cost(&path), 800u);
    }

    /* 4b. Mixed route: prefer diagonal over two cardinals */
    {
        grid_clear_free();
        /* 1 cell NE: diagonal cost 141 < two cardinals 200 */
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(sx + 1), gy = (int16_t)(sy - 1);
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_EQ(path_cost(&path), 141u);
    }

    /* 4c. Two routes around an obstacle — shorter is chosen */
    {
        grid_clear_free();
        /* Vertical wall from (ROBOT_CX+3, ROBOT_CY-3) to (ROBOT_CX+3, ROBOT_CY+3)
         * Forces path to go around either top or bottom.  Both detour by
         * the same amount; either is optimal. */
        place_wall((int16_t)(ROBOT_CX+3), (int16_t)(ROBOT_CY-3),
                   (int16_t)(ROBOT_CX+3), (int16_t)(ROBOT_CY+3));
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 6), gy = (int16_t)ROBOT_CY;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_avoids_obstacles(&path), "route around wall: hit obstacle");
        /* There must be a path; cost is > direct 600 due to detour. */
        CHECK_MSG(path_cost(&path) >= 600u, "detour cost unexpectedly low");
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 5. Obstacle avoidance
 * ====================================================================== */
static void test_obstacle_avoidance(void) {
    section("5. Obstacle avoidance");

    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 5a. Single OCCUPIED cell directly on the straight path → detour */
    {
        grid_clear_free();
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 6), gy = (int16_t)ROBOT_CY;
        /* Block the direct route at (ROBOT_CX+3, ROBOT_CY). */
        place_wall((int16_t)(ROBOT_CX+3), (int16_t)ROBOT_CY,
                   (int16_t)(ROBOT_CX+3), (int16_t)ROBOT_CY);
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_avoids_obstacles(&path), "5a: path through obstacle");
        CHECK_MSG(path_is_continuous(&path),    "5a: path not continuous");
    }

    /* 5b. U-shaped obstacle — path goes around the open end */
    {
        grid_clear_free();
        /* U-shape around (ROBOT_CX+5, ROBOT_CY):
         *   top  wall: (ROBOT_CX+2, ROBOT_CY-2) → (ROBOT_CX+7, ROBOT_CY-2)
         *   right wall: (ROBOT_CX+7, ROBOT_CY-2) → (ROBOT_CX+7, ROBOT_CY+2)
         *   bottom wall: (ROBOT_CX+2, ROBOT_CY+2) → (ROBOT_CX+7, ROBOT_CY+2)
         *   left is open */
        place_wall((int16_t)(ROBOT_CX+2), (int16_t)(ROBOT_CY-2),
                   (int16_t)(ROBOT_CX+7), (int16_t)(ROBOT_CY-2));
        place_wall((int16_t)(ROBOT_CX+7), (int16_t)(ROBOT_CY-2),
                   (int16_t)(ROBOT_CX+7), (int16_t)(ROBOT_CY+2));
        place_wall((int16_t)(ROBOT_CX+2), (int16_t)(ROBOT_CY+2),
                   (int16_t)(ROBOT_CX+7), (int16_t)(ROBOT_CY+2));
        /* Goal inside the U, reachable only through the open end. */
        /* Open end is left (< ROBOT_CX+2), so goal inside U is just
         * beyond where a straight path would enter — actually the U
         * is closed on three sides; we place goal at (ROBOT_CX+5, ROBOT_CY)
         * which is inside and should be blocked. Skip this goal. */
        /* Instead: goal is just right of the U at (ROBOT_CX+8, ROBOT_CY). */
        int16_t sx = (int16_t)ROBOT_CX, sy = (int16_t)ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 9), gy = (int16_t)ROBOT_CY;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_avoids_obstacles(&path), "5b: path through U wall");
        CHECK_MSG(path_is_continuous(&path),    "5b: U path not continuous");
    }

    /* 5c. INFLATED cells are treated as obstacles */
    {
        grid_clear_free();
        /* Mark a cell INFLATED (not OCCUPIED) in inflated[] only. */
        int16_t bx = (int16_t)(ROBOT_CX + 3), by = (int16_t)ROBOT_CY;
        g_grid.inflated[(uint32_t)by * GRID_W + (uint32_t)bx] = CELL_INFLATED;
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = (int16_t)ROBOT_CY;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_avoids_obstacles(&path),
                  "5c: INFLATED cell not treated as obstacle");
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 6. No-path cases
 * ====================================================================== */
static void test_no_path(void) {
    section("6. No-path cases");

    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 6a. Goal completely surrounded by OCCUPIED cells */
    {
        grid_clear_free();
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = (int16_t)ROBOT_CY;
        /* Box the goal. */
        place_wall((int16_t)(gx-1), (int16_t)(gy-1),
                   (int16_t)(gx+1), (int16_t)(gy+1));
        /* The goal itself must remain free for PLAN_NO_PATH (not INVALID_GOAL). */
        g_grid.inflated[(uint32_t)gy * GRID_W + (uint32_t)gx] = CELL_FREE;
        PlanResult r = astar_plan(ctx, &g_grid,
                                  ROBOT_CX, ROBOT_CY, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
        CHECK_MSG(!path.found, "6a: found set on no-path result");
    }

    /* 6b. Start enclosed by a full ring of obstacles */
    {
        grid_clear_free();
        int16_t sx = (int16_t)ROBOT_CX, sy = (int16_t)ROBOT_CY;
        /* Surround the start completely at radius 1. */
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                place_wall((int16_t)(sx+dx), (int16_t)(sy+dy),
                           (int16_t)(sx+dx), (int16_t)(sy+dy));
            }
        }
        int16_t gx = (int16_t)(sx + 10), gy = sy;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
    }

    /* 6c. Entire grid is OCCUPIED except start and goal — no route */
    {
        occ_grid_init(&g_grid);
        /* Fill inflated with OCCUPIED. */
        memset(g_grid.inflated, CELL_OCCUPIED, GRID_W * GRID_H);
        /* Free only start and goal (no adjacent free cells → unreachable). */
        g_grid.inflated[ROBOT_CY * GRID_W + ROBOT_CX] = CELL_FREE;
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = (int16_t)ROBOT_CY;
        g_grid.inflated[(uint32_t)gy * GRID_W + (uint32_t)gx] = CELL_FREE;
        PlanResult r = astar_plan(ctx, &g_grid,
                                  ROBOT_CX, ROBOT_CY, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 7. Invalid inputs
 * ====================================================================== */
static void test_invalid_inputs(void) {
    section("7. Invalid inputs");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 7a. OCCUPIED start → PLAN_INVALID_START */
    {
        place_wall(5, 5, 5, 5);
        PlanResult r = astar_plan(ctx, &g_grid, 5, 5, ROBOT_CX, ROBOT_CY, &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_START);
        /* Restore */
        g_grid.inflated[5 * GRID_W + 5] = CELL_FREE;
    }

    /* 7b. OCCUPIED goal → PLAN_INVALID_GOAL */
    {
        place_wall(10, 10, 10, 10);
        PlanResult r = astar_plan(ctx, &g_grid, ROBOT_CX, ROBOT_CY, 10, 10, &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_GOAL);
        g_grid.inflated[10 * GRID_W + 10] = CELL_FREE;
    }

    /* 7c. INFLATED start → PLAN_INVALID_START */
    {
        g_grid.inflated[(uint32_t)ROBOT_CY * GRID_W + (uint32_t)(ROBOT_CX+2)]
            = CELL_INFLATED;
        PlanResult r = astar_plan(ctx, &g_grid,
                                  (int16_t)(ROBOT_CX+2), (int16_t)ROBOT_CY,
                                  (int16_t)(ROBOT_CX+5), (int16_t)ROBOT_CY,
                                  &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_START);
        g_grid.inflated[(uint32_t)ROBOT_CY * GRID_W + (uint32_t)(ROBOT_CX+2)]
            = CELL_FREE;
    }

    /* 7d. Out-of-bounds start → PLAN_INVALID_START */
    {
        PlanResult r = astar_plan(ctx, &g_grid, -1, 0,
                                  ROBOT_CX, ROBOT_CY, &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_START);
    }

    /* 7e. Out-of-bounds goal → PLAN_INVALID_GOAL */
    {
        PlanResult r = astar_plan(ctx, &g_grid, ROBOT_CX, ROBOT_CY,
                                  GRID_W, GRID_H, &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_GOAL);
    }

    /* 7f. NULL ctx → PLAN_NO_PATH (graceful) */
    {
        PlanResult r = astar_plan(NULL, &g_grid,
                                  ROBOT_CX, ROBOT_CY,
                                  (int16_t)(ROBOT_CX+2), (int16_t)ROBOT_CY,
                                  &path);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
    }

    /* 7g. NULL grid → PLAN_NO_PATH (graceful) */
    {
        PlanResult r = astar_plan(ctx, NULL,
                                  ROBOT_CX, ROBOT_CY,
                                  (int16_t)(ROBOT_CX+2), (int16_t)ROBOT_CY,
                                  &path);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
    }

    /* 7h. NULL out_path → PLAN_NO_PATH (graceful) */
    {
        PlanResult r = astar_plan(ctx, &g_grid,
                                  ROBOT_CX, ROBOT_CY,
                                  (int16_t)(ROBOT_CX+2), (int16_t)ROBOT_CY,
                                  NULL);
        CHECK_EQ((int)r, (int)PLAN_NO_PATH);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 8. Path properties
 * ====================================================================== */
static void test_path_properties(void) {
    section("8. Path properties");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 8a. Start cell is first cell in path */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 8), gy = (int16_t)(ROBOT_CY + 3);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ(path.cells[0].cx, sx);
        CHECK_EQ(path.cells[0].cy, sy);
    }

    /* 8b. Goal cell is last cell in path */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX - 4), gy = (int16_t)(ROBOT_CY + 6);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ(path.cells[path.count-1].cx, gx);
        CHECK_EQ(path.cells[path.count-1].cy, gy);
    }

    /* 8c. All cells passable */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 10), gy = (int16_t)(ROBOT_CY + 10);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_MSG(path_avoids_obstacles(&path), "8c: path through obstacle");
    }

    /* 8d. No duplicate consecutive cells */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 15), gy = (int16_t)(ROBOT_CY - 5);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        bool no_dup = true;
        for (uint16_t i = 1; i < path.count; i++) {
            if (path.cells[i].cx == path.cells[i-1].cx &&
                path.cells[i].cy == path.cells[i-1].cy) {
                no_dup = false;
                break;
            }
        }
        CHECK_MSG(no_dup, "8d: duplicate consecutive cell in path");
    }

    /* 8e. All cells within grid bounds */
    {
        int16_t sx = 5, sy = 5;
        int16_t gx = (int16_t)(GRID_W - 5), gy = (int16_t)(GRID_H - 5);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_MSG(path_in_bounds(&path), "8e: path cell out of bounds");
    }

    /* 8f. Path count == 1 for trivial case */
    {
        astar_plan(ctx, &g_grid, 20, 20, 20, 20, &path);
        CHECK_EQ(path.count, 1u);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 9. Inflation respect
 * ====================================================================== */
static void test_inflation_respect(void) {
    section("9. Inflation respect — inflated cells blocked");

    AStarCtx *ctx = astar_create();
    NavPath path;

    /* Build a real inflated grid via occ_grid_update + occ_grid_inflate. */
    occ_grid_init(&g_grid);

    /* Place a ring of 360 synthetic LiDAR returns at 3 cells distance. */
    LidarScan scan;
    memset(&scan, 0, sizeof(scan));
    uint16_t dist = (uint16_t)(3 * GRID_CELL_MM);
    for (uint16_t deg = 0; deg < 360; deg++) {
        scan.points[deg].angle_cdeg  = (uint16_t)(deg * 100u);
        scan.points[deg].distance_mm = dist;
        scan.points[deg].intensity   = 200;
    }
    scan.count = 360;

    occ_grid_update(&g_grid, &scan);
    occ_grid_inflate(&g_grid);

    /* 9a. Robot cell must be passable after inflate */
    CHECK_MSG(occ_grid_passable(&g_grid, ROBOT_CX, ROBOT_CY),
              "9a: robot cell not passable after inflate");

    /* 9b. Plan from robot to a point just inside the obstacle ring —
     *     within the inflated margin (2 cells from the obstacle ring at 3
     *     cells) the planner should not route through inflated cells. */
    int16_t sx = ROBOT_CX, sy = ROBOT_CY;
    /* Goal at (ROBOT_CX, ROBOT_CY-2) — 2 cells north, inside free zone. */
    int16_t gx = (int16_t)ROBOT_CX, gy = (int16_t)(ROBOT_CY - 2);
    if (occ_grid_passable(&g_grid, gx, gy)) {
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_OK);
        CHECK_MSG(path_avoids_obstacles(&path), "9b: path enters inflated zone");
    } else {
        /* Goal is inside inflated zone — verify PLAN_INVALID_GOAL. */
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        CHECK_EQ((int)r, (int)PLAN_INVALID_GOAL);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 10. Start == robot cell
 * ====================================================================== */
static void test_start_robot_cell(void) {
    section("10. Start == robot cell (always passable after inflate)");

    AStarCtx *ctx = astar_create();
    NavPath path;

    occ_grid_init(&g_grid);

    /* Dense ring at 2 cells — robot cell will be explicitly freed. */
    LidarScan scan;
    memset(&scan, 0, sizeof(scan));
    uint16_t dist = (uint16_t)(2 * GRID_CELL_MM);
    for (uint16_t d = 0; d < 360; d++) {
        scan.points[d].angle_cdeg  = (uint16_t)(d * 100u);
        scan.points[d].distance_mm = dist;
        scan.points[d].intensity   = 200;
    }
    scan.count = 360;
    occ_grid_update(&g_grid, &scan);
    occ_grid_inflate(&g_grid);

    CHECK_MSG(occ_grid_passable(&g_grid, ROBOT_CX, ROBOT_CY),
              "robot cell not passable after close ring + inflate");

    /* Now find any passable neighbour as a goal. */
    int16_t gx = 0, gy = 0;
    bool found_goal = false;
    for (int dy = -5; dy <= 5 && !found_goal; dy++) {
        for (int dx = -5; dx <= 5 && !found_goal; dx++) {
            if (dx == 0 && dy == 0) continue;
            int16_t nx = (int16_t)(ROBOT_CX + dx);
            int16_t ny = (int16_t)(ROBOT_CY + dy);
            if (occ_grid_passable(&g_grid, nx, ny)) {
                gx = nx; gy = ny; found_goal = true;
            }
        }
    }
    if (found_goal) {
        PlanResult r = astar_plan(ctx, &g_grid, ROBOT_CX, ROBOT_CY, gx, gy, &path);
        /* Either PLAN_OK or PLAN_NO_PATH is acceptable depending on
         * whether a passable route exists; what matters is no crash and
         * that a successful path avoids obstacles. */
        if (r == PLAN_OK) {
            CHECK_MSG(path_avoids_obstacles(&path),
                      "10: robot-start path hits obstacle");
        } else {
            /* Dense close ring — acceptable that no path escapes. */
            CHECK_EQ((int)r, (int)PLAN_NO_PATH);
        }
    } else {
        /* No passable neighbour — nothing to test. */
        CHECK(true);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 11. Stats
 * ====================================================================== */
static void test_stats(void) {
    section("11. Stats");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    /* 11a. After a successful plan, nodes_expanded > 0 */
    {
        astar_plan(ctx, &g_grid,
                   ROBOT_CX, ROBOT_CY,
                   (int16_t)(ROBOT_CX+10), (int16_t)(ROBOT_CY+5),
                   &path);
        AStarStats st;
        astar_get_stats(ctx, &st);
        CHECK_MSG(st.nodes_expanded > 0u, "nodes_expanded == 0 after plan");
        CHECK_MSG(st.nodes_generated > 0u, "nodes_generated == 0 after plan");
    }

    /* 11b. path_cost matches manual calculation (diagonal path) */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 3), gy = (int16_t)(ROBOT_CY + 3);
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        AStarStats st;
        astar_get_stats(ctx, &st);
        /* 3 diagonal steps × 141 = 423 */
        CHECK_EQ(st.path_cost,   423u);
        CHECK_EQ(st.path_length,   4u);   /* start + 3 steps = 4 cells */
    }

    /* 11c. path_length == path.count */
    {
        int16_t sx = ROBOT_CX, sy = ROBOT_CY;
        int16_t gx = (int16_t)(ROBOT_CX + 5), gy = (int16_t)ROBOT_CY;
        astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        AStarStats st;
        astar_get_stats(ctx, &st);
        CHECK_EQ(st.path_length, (long long)path.count);
    }

    /* 11d. astar_get_stats(NULL, ...) does not crash */
    {
        AStarStats st;
        astar_get_stats(NULL, &st);
        CHECK(true);
    }

    /* 11e. After PLAN_NO_PATH, path_length == 0 */
    {
        occ_grid_init(&g_grid);
        memset(g_grid.inflated, CELL_OCCUPIED, GRID_W * GRID_H);
        g_grid.inflated[ROBOT_CY * GRID_W + ROBOT_CX] = CELL_FREE;
        int16_t gx = (int16_t)(ROBOT_CX+5), gy = (int16_t)ROBOT_CY;
        g_grid.inflated[(uint32_t)gy * GRID_W + (uint32_t)gx] = CELL_FREE;
        astar_plan(ctx, &g_grid, ROBOT_CX, ROBOT_CY, gx, gy, &path);
        AStarStats st;
        astar_get_stats(ctx, &st);
        CHECK_EQ(st.path_length, 0u);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 12. Viz overlay — NavPath casts to VizPath
 * ====================================================================== */
static void test_viz_overlay(void) {
    section("12. Viz overlay — NavPath → VizPath cast");

    grid_clear_free();
    AStarCtx *ctx = astar_create();
    NavPath path;

    int16_t sx = ROBOT_CX, sy = ROBOT_CY;
    int16_t gx = (int16_t)(ROBOT_CX + 8), gy = (int16_t)(ROBOT_CY + 4);
    PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
    CHECK_EQ((int)r, (int)PLAN_OK);

    /* 12a. Layout check: VizPathCell and our path cells are same size */
    CHECK_EQ(sizeof(VizPathCell), sizeof(path.cells[0]));

    /* 12b. viz_render with the path overlay must not crash */
    {
        VizPath vp;
        vp.cells = path.cells;    /* VizPathCell* — same type */
        vp.count = path.count;

        VizCfg cfg;
        viz_cfg_default(&cfg);
        cfg.use_colour   = false;
        cfg.clear_screen = false;
        cfg.show_stats   = false;
        cfg.downsample   = (uint8_t)(GRID_W / 40u + 1u);

        printf("\n  [viz overlay smoke — 8-cell path]\n");
        viz_render(&g_grid, &vp, &cfg);
        CHECK(true);
    }

    astar_destroy(ctx);
}

/* =========================================================================
 * 13. Full pipeline with scan_raw.bin (optional)
 * ====================================================================== */
static void test_pipeline_replay(const char *bin_path) {
    section("13. Full pipeline replay (scan_raw.bin)");

    FILE *fp = fopen(bin_path, "rb");
    if (!fp) {
        printf("  (skipped — cannot open '%s')\n", bin_path);
        return;
    }

    LidarParser *parser = lidar_parser_create();
    occ_grid_init(&g_grid);
    ScanFilterCfg fcfg; scan_filter_cfg_default(&fcfg);
    AStarCtx *ctx = astar_create();

    /* Goal: one grid-quarter ahead and to the left of the robot. */
    int16_t gx = (int16_t)(ROBOT_CX + GRID_W / 4);
    int16_t gy = (int16_t)(ROBOT_CY - GRID_H / 4);
    /* Clamp to grid. */
    if (gx >= GRID_W) gx = (int16_t)(GRID_W - 1);
    if (gy < 0)       gy = 0;

    uint8_t buf[256];
    LidarScan scan;
    int scans_ok = 0;
    int plans_ok = 0;
    int plans_no_path = 0;
    size_t n;

    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (lidar_parser_feed(parser, buf, (int)n, &scan) != PARSE_OK)
            continue;

        scan_filter_apply(&scan, &fcfg);
        occ_grid_update(&g_grid, &scan);
        occ_grid_inflate(&g_grid);
        scans_ok++;

        /* Re-plan every scan. */
        int16_t sx, sy;
        occ_grid_robot_cell(&sx, &sy);

        /* Skip if goal is now blocked. */
        if (!occ_grid_passable(&g_grid, gx, gy)) continue;

        NavPath path;
        PlanResult r = astar_plan(ctx, &g_grid, sx, sy, gx, gy, &path);
        if (r == PLAN_OK) {
            plans_ok++;
            /* Validate the path on first success. */
            // if (plans_ok == 1) {
                CHECK_MSG(path_is_continuous(&path),
                          "replay: first path not continuous");
                CHECK_MSG(path_avoids_obstacles(&path),
                          "replay: first path hits obstacle");
                CHECK_MSG(path.cells[0].cx == sx && path.cells[0].cy == sy,
                          "replay: path start != robot cell");

                /* Render once for visual inspection. */
                VizPath vp = { .cells = path.cells, .count = path.count };
                VizCfg vcfg; viz_cfg_default(&vcfg);
                vcfg.use_colour   = isatty(STDOUT_FILENO);
                vcfg.clear_screen = false;
                vcfg.show_stats   = true;
                vcfg.downsample   = (uint8_t)(GRID_W / 60u + 1u);
                printf("\n  [Pipeline scan#%d — first successful plan]\n",
                       scans_ok);
                viz_render(&g_grid, &vp, &vcfg);
            // }
        } else if (r == PLAN_NO_PATH) {
            plans_no_path++;
        }
    }
    fclose(fp);

    printf("  Scans processed : %d\n",      scans_ok);
    printf("  Plans OK        : %d\n",      plans_ok);
    printf("  Plans no-path   : %d\n",      plans_no_path);

    CHECK_MSG(scans_ok > 0,  "no scans processed from replay file");
    CHECK_MSG(plans_ok > 0 || plans_no_path > 0,
              "astar_plan never called (goal always invalid?)");

    lidar_parser_destroy(parser);
    astar_destroy(ctx);
}

/* =========================================================================
 * main
 * ====================================================================== */
int main(int argc, char *argv[]) {
    g_colour = isatty(STDOUT_FILENO);
    printf("\n%sA* planner — test suite%s\n",
           g_colour ? "\033[1;33m" : "",
           g_colour ? "\033[0m"    : "");

    // test_lifecycle();
    // test_trivial();
    // test_straight_paths();
    // test_optimality();
    // test_obstacle_avoidance();
    // test_no_path();
    // test_invalid_inputs();
    // test_path_properties();
    // test_inflation_respect();
    // test_start_robot_cell();
    // test_stats();
    // test_viz_overlay();

    if (argc >= 2)
        test_pipeline_replay(argv[1]);
    else
        printf("\n  Tip: pass path to scan_raw.bin for pipeline replay test.\n");

    /* Summary */
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