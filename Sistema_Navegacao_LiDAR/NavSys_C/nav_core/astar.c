/**
 * @file astar.c
 * @brief A* path planner on the occupancy grid — implementation.
 *
 * Portable C11.  No platform headers, no dynamic allocation, no float.
 * All large arrays are file-scope statics (BSS) — never stack-allocated.
 * Safe on ESP32-C6 with a default 8 KB task stack.
 *
 * ── Data structures ───────────────────────────────────────────────────────
 *
 *  Cell index  : uint16_t  idx = cy * GRID_W + cx   (row-major, 0-based)
 *  g_score[]   : uint16_t  g_score[idx] = cost from start to cell idx.
 *                          UINT16_MAX = "not yet visited" sentinel.
 *  parent[]    : uint16_t  parent[idx]  = idx of the cell we came from.
 *                          IDX_NONE (0xFFFF) = no parent (start cell).
 *  Open set    : binary min-heap of HeapNode {uint16_t f, uint16_t idx}.
 *                f = g + h, breaking ties by smaller f first (standard).
 *                Heap size is bounded by GRID_W × GRID_H (22 500 entries).
 *
 * ── Move costs ────────────────────────────────────────────────────────────
 *
 *  Cardinal  (N/S/E/W) : COST_CARD = 100
 *  Diagonal  (NE/…/SW) : COST_DIAG = 141  (≈ 100√2)
 *
 *  Maximum g_score across the full 150×150 diagonal:
 *    141 × 150 ≈ 21 150  — fits uint16_t (max 65 535).  ✓
 *
 * ── Heuristic ─────────────────────────────────────────────────────────────
 *
 *  Octile distance (admissible and consistent for 8-connected grids):
 *    h(n) = COST_CARD · max(|dx|,|dy|) + (COST_DIAG − COST_CARD) · min(|dx|,|dy|)
 *         = 100 · max + 41 · min
 *
 *  Because h ≤ true cost, A* is optimal.
 *  Because h is consistent (satisfies the triangle inequality), each cell
 *  is expanded at most once — no re-opening needed.
 *
 * ── Heap invariant ────────────────────────────────────────────────────────
 *
 *  Standard binary min-heap stored in g_heap[0..g_heap_size-1].
 *  Parent of node i : (i-1)/2   (integer division)
 *  Children of i    : 2i+1, 2i+2
 *  Ordering         : g_heap[i].f ≤ g_heap[child].f  for all children.
 *
 *  When a cell is re-discovered with a lower g-score it is pushed again
 *  ("lazy deletion").  When the cell is popped, if g_score[idx] is already
 *  lower than the popped f − h(idx), the entry is stale and discarded.
 *  This is correct because each cell is expanded at most once under a
 *  consistent heuristic.
 *
 * ── Path reconstruction ───────────────────────────────────────────────────
 *
 *  Walk parent[] from goal back to start, collecting cells in reverse.
 *  Reverse in-place before writing to out_path.
 *  If path length exceeds ASTAR_MAX_PATH_CELLS the path is truncated to
 *  that length (the goal end is kept, the start end is trimmed) and
 *  PLAN_OK is still returned — the caller can re-plan when needed.
 *
 * ── Memory footprint (ESP32-C6 DRAM) ─────────────────────────────────────
 *
 *  g_score[] : 150×150×2 = 45 000 B
 *  parent[]  : 150×150×2 = 45 000 B
 *  g_heap[]  : 150×150×4 = 90 000 B   (HeapNode = 2×uint16_t)
 *  g_ctx     :           ≈       8 B
 *  Total     :          ≈ 180 KB  (confirmed against §7 of the ledger)
 **/

#include "astar.h"

#include <string.h>   /* memset  */
#include <stddef.h>   /* NULL    */

/* =========================================================================
* Constants
* ====================================================================== */

#define COST_CARD   100u          /**< Cardinal move cost.                 */
#define COST_DIAG   141u          /**< Diagonal move cost (≈ 100√2).       */

#define IDX_NONE    0xFFFFu       /**< Sentinel: no parent / not visited.  */

/** Total number of cells in the grid. */
#define GRID_CELLS  ((uint32_t)GRID_W * (uint32_t)GRID_H)

/* =========================================================================
* Heap node
* ====================================================================== */

typedef struct {
   uint16_t f;    /**< f = g + h (priority key).                          */
   uint16_t idx;  /**< Flat cell index.                                   */
} HeapNode;

/* =========================================================================
* BSS arrays — never stack-allocated
*
* Declared static at file scope so the linker places them in BSS.
* On ESP32-C6 BSS is in DRAM.  The arrays are shared across all calls to
* astar_plan() and are re-initialised at the start of every call.
* ====================================================================== */

static uint16_t  g_score[GRID_CELLS];  /**< g-score for each cell.         */
static uint16_t  g_parent[GRID_CELLS]; /**< Parent index for path recovery. */
static HeapNode  g_heap[GRID_CELLS];   /**< Open-set min-heap.              */
static uint32_t  g_heap_size;          /**< Current heap occupancy.         */

/* =========================================================================
* AStarCtx — opaque handle (tiny; holds stats only)
* ====================================================================== */

struct AStarCtx {
   bool       in_use;
   AStarStats last_stats;
};

/** Single static instance — only one planner at a time (ESP32 constraint). */
static struct AStarCtx g_ctx_pool;

/* =========================================================================
* Lifecycle
* ====================================================================== */

AStarCtx *astar_create(void) {
   g_ctx_pool.in_use = true;
   memset(&g_ctx_pool.last_stats, 0, sizeof(g_ctx_pool.last_stats));
   return &g_ctx_pool;
}

void astar_destroy(AStarCtx *ctx) {
   if(!ctx) return;
   ctx->in_use = false;
}

/* =========================================================================
* Internal: cell index / coordinate helpers
* ====================================================================== */

static inline uint16_t make_idx(int16_t cx, int16_t cy) {
   return (uint16_t)((uint32_t)cy * (uint32_t)GRID_W + (uint32_t)cx);
}

static inline int16_t idx_cx(uint16_t idx) {
   return (int16_t)(idx % (uint16_t)GRID_W);
}

static inline int16_t idx_cy(uint16_t idx) {
   return (int16_t)(idx / (uint16_t)GRID_W);
}

static inline bool cell_in_grid(int16_t cx, int16_t cy) {
   return (cx >= 0) && (cx < GRID_W) && (cy >= 0) && (cy < GRID_H);
}

/* =========================================================================
* Internal: octile-distance heuristic
*
* h(n → goal) = 100·max(|dx|,|dy|) + 41·min(|dx|,|dy|)
*
* Returned as uint16_t.  Maximum value over a 150×150 grid:
*   100·150 + 41·150 = 21 150 — fits uint16_t.
* ====================================================================== */

static inline uint16_t heuristic(
   int16_t cx, int16_t cy,
   int16_t gx, int16_t gy
) {
   int16_t dx = (cx > gx) ? (int16_t)(cx - gx) : (int16_t)(gx - cx);
   int16_t dy = (cy > gy) ? (int16_t)(cy - gy) : (int16_t)(gy - cy);
   uint16_t hi = (dx > dy) ? (uint16_t)dx : (uint16_t)dy;
   uint16_t lo = (dx < dy) ? (uint16_t)dx : (uint16_t)dy;
   return (uint16_t)(COST_CARD * hi + (COST_DIAG - COST_CARD) * lo);
}

/* =========================================================================
* Internal: binary min-heap operations
* ====================================================================== */

static inline void heap_push(uint16_t f, uint16_t idx) {
   uint32_t i = g_heap_size++;
   g_heap[i].f   = f;
   g_heap[i].idx = idx;

   /* Sift up. */
   while(i > 0u) {
      uint32_t parent = (i - 1u) / 2u;
      if(g_heap[parent].f <= g_heap[i].f) break;
      /* Swap. */
      HeapNode tmp    = g_heap[parent];
      g_heap[parent]  = g_heap[i];
      g_heap[i]       = tmp;
      i = parent;
   }
}

static inline HeapNode heap_pop(void) {
   HeapNode top   = g_heap[0];
   g_heap_size--;

   if(g_heap_size > 0u) {
      g_heap[0] = g_heap[g_heap_size];

      /* Sift down. */
      uint32_t i = 0;
      for(;;) {
         uint32_t left  = 2u * i + 1u;
         uint32_t right = 2u * i + 2u;
         uint32_t smallest = i;

         if(left  < g_heap_size && g_heap[left].f  < g_heap[smallest].f)
            smallest = left;
         if(right < g_heap_size && g_heap[right].f < g_heap[smallest].f)
            smallest = right;

         if(smallest == i) break;

         HeapNode tmp      = g_heap[i];
         g_heap[i]         = g_heap[smallest];
         g_heap[smallest]  = tmp;
         i = smallest;
      }
   }
   return top;
}

/* =========================================================================
* 8-connected neighbour table
*
* (dx, dy, cost) for all 8 directions.
* Ordered: 4 cardinals first, then 4 diagonals.  This is the traversal
* order used when expanding each node.
* ====================================================================== */

typedef struct { int8_t dx; int8_t dy; uint16_t cost; } Move;

static const Move MOVES[8] = {
   {  1,  0, COST_CARD },   /* E  */
   { -1,  0, COST_CARD },   /* W  */
   {  0,  1, COST_CARD },   /* S  */
   {  0, -1, COST_CARD },   /* N  */
   {  1,  1, COST_DIAG },   /* SE */
   {  1, -1, COST_DIAG },   /* NE */
   { -1,  1, COST_DIAG },   /* SW */
   { -1, -1, COST_DIAG },   /* NW */
};

/* =========================================================================
* Core planner
* ====================================================================== */

PlanResult astar_plan(
   AStarCtx            *ctx,
   const OccupancyGrid *g,
   int16_t sx, int16_t sy,
   int16_t gx, int16_t gy,
   NavPath             *out_path
) {
   if(!ctx || !g || !out_path) return PLAN_NO_PATH;

   /* Reset stats. */
   AStarStats *st = &ctx->last_stats;
   memset(st, 0, sizeof(*st));

   /* ── Validate start ── */
   if(!cell_in_grid(sx, sy) || !occ_grid_passable(g, sx, sy))
      return PLAN_INVALID_START;

   /* ── Validate goal ── */
   if(!cell_in_grid(gx, gy) || !occ_grid_passable(g, gx, gy))
      return PLAN_INVALID_GOAL;

   /* ── Trivial case: start == goal ── */
   if(sx == gx && sy == gy) {
      out_path->cells[0].cx = sx;
      out_path->cells[0].cy = sy;
      out_path->count = 1;
      out_path->found = true;
      st->path_length = 1;
      st->path_cost   = 0;
      return PLAN_OK;
   }

   /* ── Initialise arrays ── */
   /*
   * memset g_score to 0xFF sets every uint16_t element to 0xFFFF
   * (UINT16_MAX), which serves as the "not visited" sentinel.
   */
   memset(g_score,  0xFF, sizeof(g_score));
   memset(g_parent, 0xFF, sizeof(g_parent));   /* IDX_NONE everywhere */
   g_heap_size = 0;

   /* ── Seed open set with start cell ── */
   uint16_t start_idx = make_idx(sx, sy);
   uint16_t goal_idx  = make_idx(gx, gy);

   g_score[start_idx] = 0;
   uint16_t h0 = heuristic(sx, sy, gx, gy);
   heap_push(h0, start_idx);

   /* ── Main A* loop ── */
   while(g_heap_size > 0u) {
      HeapNode cur = heap_pop();
      uint16_t cur_idx = cur.idx;

      /* Lazy-deletion: skip stale heap entries.
      *
      * A cell can appear multiple times in the heap (once per relaxation).
      * Under a consistent heuristic, the first time a cell is popped its
      * g_score is already optimal.  Any later pop with a higher f value
      * is stale and must be discarded.
      *
      * Stale test: cur.f > g_score[cur_idx] + h(cur)
      * i.e. the f stored in the heap exceeds the f we would compute now.
      * Equivalently: the g component stored is larger than g_score[].
      *
      * We compute: g_in_heap = cur.f - h(cur).  If g_in_heap > g_score[],
      * the entry is stale.
      */
      int16_t cur_cx = idx_cx(cur_idx);
      int16_t cur_cy = idx_cy(cur_idx);
      uint16_t h_cur  = heuristic(cur_cx, cur_cy, gx, gy);
      /* Arithmetic: cur.f = g + h; g_in_heap = cur.f - h_cur.
      * Use saturating subtraction to guard against underflow on
      * the very first node (f == h, g == 0). */
      uint16_t g_in_heap = (cur.f >= h_cur) ? (uint16_t)(cur.f - h_cur) : 0u;
      if(g_in_heap > g_score[cur_idx]) continue;   /* stale */

      st->nodes_expanded++;

      /* ── Goal reached ── */
      if(cur_idx == goal_idx) {
         /* Reconstruct path by walking parent[] backwards. */
         /* Use g_heap as a temporary stack (it is no longer needed). */
         uint32_t  depth = 0;
         uint16_t  node  = goal_idx;

         /* Count path length first to know if we must truncate. */
         {
            uint16_t n = goal_idx;
            while(n != IDX_NONE && depth < ASTAR_MAX_PATH_CELLS + 1u) {
               depth++;
               n = g_parent[n];
            }
         }

         /* Collect cells into the output array in reverse order.
            * If path is longer than ASTAR_MAX_PATH_CELLS, skip the
            * surplus start-end cells (keep goal end). */
         uint32_t skip = (depth > ASTAR_MAX_PATH_CELLS)
            ? (depth - ASTAR_MAX_PATH_CELLS)
            : 0u;

         /* Walk once more collecting only the cells we want. */
         /* temp[] on the heap (it is the BSS array g_heap cast as
            * a VizPathCell array — safe because we are done with the
            * heap and sizeof(VizPathCell) == 4 == sizeof(HeapNode)). */
         VizPathCell *tmp = (VizPathCell *)(void *)g_heap;

         uint32_t collected = 0;
         node = goal_idx;
         uint32_t step = 0;
         while(node != IDX_NONE && step < depth) {
            if(step >= skip && collected < ASTAR_MAX_PATH_CELLS) {
               tmp[collected].cx = idx_cx(node);
               tmp[collected].cy = idx_cy(node);
               collected++;
            }
            node = g_parent[node];
            step++;
         }

         /* tmp[] is goal-first; reverse into out_path->cells[]. */
         uint16_t cnt = (uint16_t)collected;
         for(uint16_t i = 0; i < cnt; i++) {
            out_path->cells[i] = tmp[cnt - 1u - i];
         }
         out_path->count = cnt;
         out_path->found = true;

         st->path_length = cnt;
         st->path_cost   = g_score[goal_idx];

         return PLAN_OK;
      }

      /* ── Expand neighbours ── */
      uint16_t g_cur = g_score[cur_idx];

      for(int m = 0; m < 8; m++) {
         int16_t nx = (int16_t)(cur_cx + MOVES[m].dx);
         int16_t ny = (int16_t)(cur_cy + MOVES[m].dy);

         /* occ_grid_passable returns false for OOB via get_inflated OOB→INFLATED */
         if(!occ_grid_passable(g, nx, ny)) continue;

         uint16_t ng = (uint16_t)(g_cur + MOVES[m].cost);
         uint16_t nb_idx = make_idx(nx, ny);

         /* Guard against uint16_t overflow (should not happen in
            * practice on a 150×150 grid, but belt-and-suspenders). */
         if(ng < g_cur) continue;   /* overflow sentinel */

         if(ng < g_score[nb_idx]) {
            g_score[nb_idx]  = ng;
            g_parent[nb_idx] = cur_idx;

            uint16_t h_nb = heuristic(nx, ny, gx, gy);
            uint16_t f_nb = (uint16_t)(ng + h_nb);
            /* Guard f overflow. */
            if(f_nb < ng) f_nb = 0xFFFEu;

            heap_push(f_nb, nb_idx);
            st->nodes_generated++;
         }
      }
   }

   /* Open set exhausted — no path found. */
   out_path->count = 0;
   out_path->found = false;
   return PLAN_NO_PATH;
}

/* =========================================================================
* Diagnostics
* ====================================================================== */

void astar_get_stats(const AStarCtx *ctx, AStarStats *out) {
   if(!ctx || !out) return;
   *out = ctx->last_stats;
}