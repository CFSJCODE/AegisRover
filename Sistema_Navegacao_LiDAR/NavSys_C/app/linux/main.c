/**
 * @file main.c
 * @brief Live LiDAR navigation loop — Linux / USB-UART, Phase 3.
 *
 * Runs the full pipeline once per completed 360° scan:
 *
 *   hal_serial_read → lidar_parser_feed → scan_filter_apply →
 *   occ_grid_update → occ_grid_inflate → astar_plan → viz_render
 *
 * The goal cell is set interactively at runtime.  The default goal is the
 * grid centre (robot's own cell) so the display is useful immediately on
 * startup even before a goal is given.
 *
 * ── Goal input ────────────────────────────────────────────────────────────
 *
 * The goal can be changed at any time without stopping the sensor loop.
 * A background thread reads stdin; the main loop checks for a new goal
 * at the start of each scan cycle.
 *
 * Accepted formats on stdin (one per line):
 *
 *   CELL  <cx> <cy>     — absolute grid cell coordinates
 *   WORLD <wx> <wy>     — world mm (robot-centred); converted to cell
 *   CLEAR               — clear the goal; planner is disabled
 *
 * Examples:
 *   CELL  90 60          → plan to cell (90, 60)
 *   WORLD 1000 500       → plan to the cell covering (+1 m, +0.5 m)
 *   CLEAR                → stop planning
 *
 * ── Usage ─────────────────────────────────────────────────────────────────
 *
 *   ./lidar_nav [OPTIONS] <port>
 *
 *   <port>           Serial device, e.g. /dev/ttyUSB0
 *
 * Options:
 *   -b <baud>        Baud rate                (default: 230400)
 *   -d <ds>          Grid downsample factor   (default: auto-fit terminal)
 *   -f               Plain ASCII — no ANSI colour
 *   -s               Suppress per-scan stats line
 *   -c <path>        CSV dump path            (overwritten each scan)
 *   -n <count>       Exit after <count> complete scans (default: unlimited)
 *   -g <cx>,<cy>     Set initial goal cell    (e.g. -g 90,60)
 *   -w <wx>,<wy>     Set initial goal in world mm (e.g. -w 1000,500)
 *   -r <file>        Replay from captured .bin instead of live port
 *   -h               Print this help and exit
 *
 * Build (from the directory containing all source files):
 *
 *   gcc -std=c11 -Wall -Wextra -O2 \
 *       -I. \
 *       main.c astar.c lidar_parser.c scan_filter.c \
 *       occupancy_grid.c grid_viz.c hal_linux.c \
 *       -lpthread -o lidar_nav
 *
 * Notes:
 *   - OccupancyGrid (~45 KB) and NavPath (~2 KB) are static globals.
 *   - Bytes are fed in ≤256-byte chunks to avoid the parser returning only
 *     the last scan in a multi-revolution buffer.
 *   - SIGINT (Ctrl-C) flushes the partial revolution and prints a summary.
 */

#define _GNU_SOURCE   /* nanosleep, sigaction, getopt */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>   /* getopt, isatty, STDOUT_FILENO */

#include "hal.h"
#include "hal_linux_ext.h"
#include "lidar_parser.h"
#include "scan_filter.h"
#include "occupancy_grid.h"
#include "grid_viz.h"
#include "astar.h"

/* =========================================================================
 * Static globals — too large for the stack
 * ====================================================================== */

static OccupancyGrid g_grid;   /* ~45 KB */
static NavPath       g_path;   /* ~2 KB  */

/* =========================================================================
 * Shared goal state — written by the goal-input thread,
 * read by the main scan loop.
 *
 * pthread_mutex guards the three goal fields.  The main loop takes the
 * lock only long enough to snapshot the current goal, so sensor latency
 * is unaffected.
 * ====================================================================== */

typedef struct {
    int16_t  cx;       /* grid cell column                  */
    int16_t  cy;       /* grid cell row                     */
    bool     active;   /* false → planner disabled          */
    bool     dirty;    /* true → goal changed since last use */
} Goal;

static Goal          g_goal;
static pthread_mutex_t g_goal_mutex = PTHREAD_MUTEX_INITIALIZER;

/* =========================================================================
 * Shutdown flag (set by SIGINT / SIGTERM)
 * ====================================================================== */

static volatile bool g_running = true;

static void handle_signal(int sig)
{
    (void)sig;
    g_running = false;
}

/* =========================================================================
 * Goal-input thread
 *
 * Reads lines from stdin in a tight loop.  Parses CELL/WORLD/CLEAR
 * commands and updates g_goal under the mutex.  Exits when g_running
 * is cleared or stdin reaches EOF.
 * ====================================================================== */

static void *goal_input_thread(void *arg)
{
    (void)arg;
    char line[128];

    while (g_running) {
        if (!fgets(line, sizeof(line), stdin)) break;  /* EOF or error */

        /* Strip trailing newline. */
        line[strcspn(line, "\r\n")] = '\0';

        /* Skip blank lines and comments. */
        if (line[0] == '\0' || line[0] == '#') continue;

        pthread_mutex_lock(&g_goal_mutex);

        if (strncmp(line, "CLEAR", 5) == 0) {
            g_goal.active = false;
            g_goal.dirty  = true;
            fprintf(stderr, "[GOAL] Cleared — planner disabled\n");

        } else if (strncmp(line, "CELL", 4) == 0) {
            int cx, cy;
            if (sscanf(line + 4, "%d %d", &cx, &cy) == 2) {
                g_goal.cx     = (int16_t)cx;
                g_goal.cy     = (int16_t)cy;
                g_goal.active = true;
                g_goal.dirty  = true;
                fprintf(stderr, "[GOAL] Cell (%d, %d)\n", cx, cy);
            } else {
                fprintf(stderr,
                    "[GOAL] Bad CELL format — expected: CELL <cx> <cy>\n");
            }

        } else if (strncmp(line, "WORLD", 5) == 0) {
            int wx, wy;
            if (sscanf(line + 5, "%d %d", &wx, &wy) == 2) {
                /* Convert world mm → grid cell (uses g_grid config). */
                int16_t cx, cy;
                occ_grid_world_to_cell(&g_grid,
                                       (int32_t)wx, (int32_t)wy,
                                       &cx, &cy);
                g_goal.cx     = cx;
                g_goal.cy     = cy;
                g_goal.active = true;
                g_goal.dirty  = true;
                fprintf(stderr,
                    "[GOAL] World (%d mm, %d mm) → cell (%d, %d)\n",
                    wx, wy, (int)cx, (int)cy);
            } else {
                fprintf(stderr,
                    "[GOAL] Bad WORLD format — expected: WORLD <wx_mm> <wy_mm>\n");
            }

        } else {
            fprintf(stderr,
                "[GOAL] Unknown command '%s'  "
                "(try: CELL <cx> <cy> | WORLD <wx> <wy> | CLEAR)\n",
                line);
        }

        pthread_mutex_unlock(&g_goal_mutex);
    }
    return NULL;
}

/* =========================================================================
 * Helper: human-readable PlanResult string
 * ====================================================================== */

static const char *plan_result_str(PlanResult r)
{
    switch (r) {
        case PLAN_OK:            return "OK";
        case PLAN_NO_PATH:       return "NO_PATH";
        case PLAN_INVALID_START: return "INVALID_START";
        case PLAN_INVALID_GOAL:  return "INVALID_GOAL";
        default:                 return "UNKNOWN";
    }
}

/* =========================================================================
 * Help text
 * ====================================================================== */

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [OPTIONS] <port>\n"
        "\n"
        "  <port>           Serial device (e.g. /dev/ttyUSB0)\n"
        "\n"
        "Options:\n"
        "  -b <baud>        Baud rate              (default: 230400)\n"
        "  -d <ds>          Downsample factor       (default: auto)\n"
        "  -f               Plain ASCII (no ANSI colour)\n"
        "  -s               Suppress per-scan stats line\n"
        "  -c <path>        CSV dump path           (overwritten each scan)\n"
        "  -n <count>       Stop after N complete scans\n"
        "  -g <cx>,<cy>     Initial goal cell       (e.g. -g 90,60)\n"
        "  -w <wx>,<wy>     Initial goal world mm   (e.g. -w 1000,500)\n"
        "  -r <file>        Replay from .bin file\n"
        "  -h               This help\n"
        "\n"
        "Goal commands on stdin (while running):\n"
        "  CELL  <cx> <cy>  — set goal to grid cell\n"
        "  WORLD <wx> <wy>  — set goal in world mm (robot-centred)\n"
        "  CLEAR            — disable planner\n",
        prog);
}

/* =========================================================================
 * main
 * ====================================================================== */

int main(int argc, char *argv[])
{
    /* ── Default options ── */
    uint32_t    baud          = 230400;
    uint8_t     downsample    = 0;      /* 0 = auto-fit */
    bool        no_colour     = false;
    bool        no_stats      = false;
    const char *csv_path      = NULL;
    int         max_scans     = 0;      /* 0 = unlimited */
    const char *replay_path   = NULL;
    bool        have_init_goal = false;
    int16_t     init_goal_cx  = 0;
    int16_t     init_goal_cy  = 0;

    /* ── Parse options ── */
    int opt;
    while ((opt = getopt(argc, argv, "b:d:fsc:n:g:w:r:h")) != -1) {
        switch (opt) {
            case 'b':
                baud = (uint32_t)atoi(optarg);
                break;
            case 'd':
                downsample = (uint8_t)atoi(optarg);
                break;
            case 'f':
                no_colour = true;
                break;
            case 's':
                no_stats = true;
                break;
            case 'c':
                csv_path = optarg;
                break;
            case 'n':
                max_scans = atoi(optarg);
                break;
            case 'g': {
                int cx, cy;
                if (sscanf(optarg, "%d,%d", &cx, &cy) == 2) {
                    init_goal_cx   = (int16_t)cx;
                    init_goal_cy   = (int16_t)cy;
                    have_init_goal = true;
                } else {
                    fprintf(stderr, "Error: -g expects <cx>,<cy> (e.g. -g 90,60)\n");
                    return 1;
                }
                break;
            }
            case 'w': {
                /* World mm goal — convert after grid is initialised; stash raw. */
                int wx, wy;
                if (sscanf(optarg, "%d,%d", &wx, &wy) == 2) {
                    /* Temporary: store world mm in cx/cy using int16_t casts.
                     * We'll convert properly once the grid is initialised. */
                    init_goal_cx   = (int16_t)wx;
                    init_goal_cy   = (int16_t)wy;
                    have_init_goal = true;
                    /* Flag that these are world coords by abusing a sentinel.
                     * Use a separate bool instead for clarity. */
                    /* Convert immediately after grid init using a flag. */
                    /* Re-use the 'w' case: stash as world and convert below. */
                    /* We signal by setting downsample = 0xFF temporarily?
                     * Cleaner: use a local flag variable. */
                    /* ── Actually: just remember the raw mm values in a pair
                     * of local ints and convert after occ_grid_init(). ── */
                    /* We'll handle this with dedicated locals below. */
                    /* Override: set a "world goal" flag. */
                    /* To keep it simple without an extra bool, encode:
                     *   init_goal_cx = -(wx+1), init_goal_cy = -(wy+1)
                     * and detect the sign.  But that is fragile.
                     *
                     * The cleanest solution: two local variables declared
                     * outside the switch.  But C99 doesn't allow jumping
                     * into a block with initialised variables.
                     *
                     * Solution: declare them before the while(getopt) loop.
                     * Since we already parsed argc/argv sequentially and
                     * 'w' is handled in this case, we will just do the
                     * world→cell conversion right here. The grid hasn't been
                     * initialised yet but occ_grid_world_to_cell only uses
                     * compile-time constants (GRID_W, GRID_H, GRID_CELL_MM,
                     * ROBOT_CX, ROBOT_CY) — no struct fields — so it is safe
                     * to call before occ_grid_init(). */
                    OccupancyGrid dummy;   /* no init needed — world_to_cell
                                           * only uses compile-time constants */
                    (void)memset(&dummy, 0, sizeof(dummy));
                    int16_t cx, cy;
                    occ_grid_world_to_cell(&dummy, (int32_t)wx, (int32_t)wy,
                                           &cx, &cy);
                    init_goal_cx = cx;
                    init_goal_cy = cy;
                    fprintf(stderr,
                        "[GOAL] -w %d,%d → cell (%d, %d)\n",
                        wx, wy, (int)cx, (int)cy);
                } else {
                    fprintf(stderr,
                        "Error: -w expects <wx_mm>,<wy_mm> (e.g. -w 1000,500)\n");
                    return 1;
                }
                break;
            }
            case 'r':
                replay_path = optarg;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: serial port argument required.\n");
        print_usage(argv[0]);
        return 1;
    }
    const char *port = argv[optind];

    /* ── Signal handling ── */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* ── Logging ── */
    hal_log_set_level(HAL_LOG_INFO);
    HAL_LOGI("MAIN", "Platform: %s", hal_platform_name());

    /* ── Open serial / replay ── */
    HalSerial *serial   = NULL;
    bool       is_replay = (replay_path != NULL);

    if (is_replay) {
        HAL_LOGI("MAIN", "Replay mode: %s", replay_path);
        serial = hal_serial_open_replay(replay_path, 0 /* max speed */);
        if (!serial) {
            fprintf(stderr, "Error: cannot open replay file '%s'\n",
                    replay_path);
            return 1;
        }
    } else {
        HAL_LOGI("MAIN", "Live mode: %s @ %u baud", port, baud);
        serial = hal_serial_open(port, baud);
        if (!serial) {
            fprintf(stderr,
                "Error: cannot open serial port '%s' at %u baud\n",
                port, baud);
            return 1;
        }
        hal_serial_flush_rx(serial);
    }

    /* ── Pipeline objects ── */
    LidarParser *parser = lidar_parser_create();
    if (!parser) {
        fprintf(stderr, "Error: lidar_parser_create() failed\n");
        return 1;
    }

    occ_grid_init(&g_grid);

    ScanFilterCfg filter_cfg;
    scan_filter_cfg_default(&filter_cfg);

    AStarCtx *astar = astar_create();
    if (!astar) {
        fprintf(stderr, "Error: astar_create() failed\n");
        return 1;
    }

    /* ── Initialise goal ── */
    pthread_mutex_lock(&g_goal_mutex);
    if (have_init_goal) {
        g_goal.cx     = init_goal_cx;
        g_goal.cy     = init_goal_cy;
        g_goal.active = true;
        g_goal.dirty  = true;
        HAL_LOGI("MAIN", "Initial goal: cell (%d, %d)",
                 (int)init_goal_cx, (int)init_goal_cy);
    } else {
        g_goal.active = false;
        g_goal.dirty  = false;
    }
    pthread_mutex_unlock(&g_goal_mutex);

    /* ── Visualiser configuration ── */
    VizCfg viz_cfg;
    viz_cfg_default(&viz_cfg);
    viz_cfg.use_colour    = no_colour ? false : isatty(STDOUT_FILENO);
    viz_cfg.show_stats    = !no_stats;
    viz_cfg.clear_screen  = viz_cfg.use_colour;
    viz_cfg.show_inflated = true;
    if (downsample > 0)
        viz_cfg.downsample = downsample;

    /* ── Goal-input thread ── */
    pthread_t input_tid;
    if (pthread_create(&input_tid, NULL, goal_input_thread, NULL) != 0) {
        HAL_LOGW("MAIN", "Could not start goal-input thread — "
                         "stdin goal changes unavailable");
    }

    /* ── Print startup hint ── */
    fprintf(stderr,
        "\nPhase 3 live — type goal commands on stdin:\n"
        "  CELL  <cx> <cy>   set goal to grid cell\n"
        "  WORLD <wx> <wy>   set goal in world mm\n"
        "  CLEAR             disable planner\n"
        "  Ctrl-C            quit\n\n");

    /* ── Main scan loop ── */
    uint8_t   buf[256];
    LidarScan scan;
    int       scan_count  = 0;

    /* Snapshot of the current goal taken once per scan cycle. */
    Goal      cur_goal    = { .active = false };

    /* Last plan result — used in the stats line. */
    PlanResult last_plan  = PLAN_NO_PATH;
    bool       planned_ok = false;      /* true if g_path is valid this cycle */

    HAL_LOGI("MAIN", "Scan loop started (Ctrl-C to stop)");

    while (g_running) {
        /* ── Read bytes ── */
        int n;
        if (is_replay) {
            n = hal_serial_read_replay(serial, buf, (int)sizeof(buf), 100);
        } else {
            n = hal_serial_read(serial, buf, (int)sizeof(buf), 100);
        }

        if (n < 0) {
            HAL_LOGE("MAIN", "Serial read error — aborting");
            break;
        }
        if (n == 0) {
            if (is_replay) {
                HAL_LOGI("MAIN", "Replay EOF");
                break;
            }
            continue;   /* live timeout — normal */
        }

        /* ── Parse ── */
        ParseResult parse_res = lidar_parser_feed(parser, buf, n, &scan);
        if (parse_res != PARSE_OK) continue;

        /* ── Complete scan available ── */
        scan_count++;

        /* Snapshot goal under lock — hold for the minimum possible time. */
        pthread_mutex_lock(&g_goal_mutex);
        cur_goal       = g_goal;
        g_goal.dirty   = false;
        pthread_mutex_unlock(&g_goal_mutex);

        /* 1. Filter */
        scan_filter_apply(&scan, &filter_cfg);

        /* 2. Ray-cast update */
        occ_grid_update(&g_grid, &scan);

        /* 3. Inflate */
        occ_grid_inflate(&g_grid);

        /* 4. Plan (if a goal is set) */
        planned_ok = false;
        if (cur_goal.active) {
            int16_t sx, sy;
            occ_grid_robot_cell(&sx, &sy);

            last_plan = astar_plan(astar, &g_grid,
                                   sx, sy,
                                   cur_goal.cx, cur_goal.cy,
                                   &g_path);
            planned_ok = (last_plan == PLAN_OK);

            /* Log notable plan events (not every scan — only on change). */
            if (last_plan == PLAN_INVALID_GOAL) {
                HAL_LOGW("MAIN",
                    "Goal (%d,%d) is blocked — waiting for it to clear",
                    (int)cur_goal.cx, (int)cur_goal.cy);
            } else if (last_plan == PLAN_NO_PATH) {
                HAL_LOGW("MAIN",
                    "No path to (%d,%d) — environment may be fully enclosed",
                    (int)cur_goal.cx, (int)cur_goal.cy);
            }
        }

        /* 5. Render — pass path overlay only when we have a valid plan */
        if (planned_ok) {
            VizPath vp = { .cells = g_path.cells, .count = g_path.count };
            viz_render(&g_grid, &vp, &viz_cfg);
        } else {
            viz_render(&g_grid, NULL, &viz_cfg);
        }

        /* 6. CSV dump */
        if (csv_path) {
            if (!viz_dump_csv(&g_grid, csv_path))
                HAL_LOGW("MAIN", "CSV dump to '%s' failed", csv_path);
        }

        /* 7. Per-scan diagnostics */
        if (!no_stats) {
            ParserStats ps;
            lidar_parser_get_stats(parser, &ps);
            ScanFilterStats fs;
            scan_filter_get_stats(&fs);

            if (cur_goal.active && planned_ok) {
                AStarStats as;
                astar_get_stats(astar, &as);
                HAL_LOGI("MAIN",
                    "scan#%d  pts=%u→%u  plan=%s  path=%u cells  "
                    "cost=%u  expanded=%u",
                    scan_count,
                    fs.in_count, fs.out_count,
                    plan_result_str(last_plan),
                    as.path_length, as.path_cost,
                    as.nodes_expanded);
            } else {
                HAL_LOGI("MAIN",
                    "scan#%d  pts=%u→%u  plan=%s  "
                    "parser_ok=%u  crc_err=%u",
                    scan_count,
                    fs.in_count, fs.out_count,
                    cur_goal.active
                        ? plan_result_str(last_plan)
                        : "disabled",
                    ps.packets_ok, ps.packets_crc_err);
            }
        }

        /* 8. Stop condition */
        if (max_scans > 0 && scan_count >= max_scans) {
            HAL_LOGI("MAIN", "Reached requested scan count (%d)", max_scans);
            break;
        }
    }

    /* ── Shut down goal-input thread ── */
    g_running = false;
    /* The input thread may be blocked in fgets(); it will exit at the
     * next newline or when the process exits.  We detach rather than join
     * to avoid hanging on a live terminal. */
    pthread_detach(input_tid);

    /* ── Flush partial revolution ── */
    if (lidar_parser_flush(parser, &scan)) {
        HAL_LOGI("MAIN", "Flushed partial revolution (%u pts)", scan.count);
        if (scan.count > 0) {
            scan_filter_apply(&scan, &filter_cfg);
            occ_grid_update(&g_grid, &scan);
            occ_grid_inflate(&g_grid);

            /* Re-plan one last time if goal is active. */
            planned_ok = false;
            if (cur_goal.active) {
                int16_t sx, sy;
                occ_grid_robot_cell(&sx, &sy);
                last_plan = astar_plan(astar, &g_grid,
                                       sx, sy,
                                       cur_goal.cx, cur_goal.cy,
                                       &g_path);
                planned_ok = (last_plan == PLAN_OK);
            }

            VizCfg final_cfg    = viz_cfg;
            final_cfg.clear_screen = false;
            if (planned_ok) {
                VizPath vp = { .cells = g_path.cells, .count = g_path.count };
                viz_render(&g_grid, &vp, &final_cfg);
            } else {
                viz_render(&g_grid, NULL, &final_cfg);
            }
        }
    }

    /* ── Session summary ── */
    ParserStats ps;
    lidar_parser_get_stats(parser, &ps);
    printf("\n── Session summary ──────────────────────────────────\n");
    printf("Complete scans   : %d\n", scan_count);
    printf("Parser: ok=%u  crc_err=%u  frame_err=%u  overflow=%u\n",
           ps.packets_ok, ps.packets_crc_err,
           ps.packets_frame_err, ps.points_overflow);
    if (cur_goal.active) {
        AStarStats as;
        astar_get_stats(astar, &as);
        printf("Last plan: %-16s  path=%u cells  cost=%u  expanded=%u\n",
               plan_result_str(last_plan),
               as.path_length, as.path_cost, as.nodes_expanded);
    }
    printf("─────────────────────────────────────────────────────\n\n");

    /* ── Cleanup ── */
    astar_destroy(astar);
    lidar_parser_destroy(parser);

    if (is_replay)
        hal_serial_close_replay(serial);
    else
        hal_serial_close(serial);

    return 0;
}