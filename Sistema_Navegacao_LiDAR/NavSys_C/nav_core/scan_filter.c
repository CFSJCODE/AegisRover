/**
 * @file scan_filter.c
 * @brief LiDAR scan filter — implementation.
 *
 * Portable C11.  No platform headers, no dynamic allocation, no float.
 * The median filter uses a small fixed-size scratch buffer on the stack;
 * window size is bounded at 5, so the scratch costs at most 10 bytes.
**/

#include "scan_filter.h"
#include <string.h>   /* memcpy */
#include <stddef.h>   /* NULL   */

/* =========================================================================
* Module-level stats (last call only)
* ====================================================================== */

static ScanFilterStats g_stats;

/* =========================================================================
* Default configuration
* ====================================================================== */

void scan_filter_cfg_default(ScanFilterCfg *cfg) {
   if(!cfg) return;
   cfg->min_dist_mm   = FILTER_MIN_DIST_MM;
   cfg->max_dist_mm   = FILTER_MAX_DIST_MM;
   cfg->min_intensity = FILTER_MIN_INTENSITY;
   cfg->median_window = FILTER_MEDIAN_WINDOW;
}

/* =========================================================================
* Internal: insertion-sort median on a tiny window (w = 3 or 5)
*
* Works on uint16_t values.  No dynamic allocation; window is at most 5
* so the sort is ≤ 10 comparisons.
* ====================================================================== */

static uint16_t median_u16(const uint16_t *vals, uint8_t n) {
   /* Copy into scratch and insertion-sort. */
   uint16_t s[5];
   for(uint8_t i = 0; i < n; i++) s[i] = vals[i];
   for(uint8_t i = 1; i < n; i++) {
      uint16_t key = s[i];
      int8_t   j   = (int8_t)(i - 1);
      while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
      s[j+1] = key;
   }
   return s[n / 2];
}

/* =========================================================================
* Core pipeline
* ====================================================================== */

void scan_filter_apply(LidarScan *scan, const ScanFilterCfg *cfg) {
   if(!scan) return;

   /* Use caller config or fall back to defaults. */
   ScanFilterCfg local;
   if(!cfg) {
      scan_filter_cfg_default(&local);
      cfg = &local;
   }

   memset(&g_stats, 0, sizeof(g_stats));
   g_stats.in_count = scan->count;

   /* ------------------------------------------------------------------
   * Stage 1: zero-distance, intensity gate, range clamp.
   *
   * Compact surviving points to the front of the array in a single
   * pass.  Order (angle-sorted) is preserved.
   * ----------------------------------------------------------------*/
   uint16_t out = 0;
   for(uint16_t i = 0; i < scan->count; i++) {
      LidarPoint *pt = &scan->points[i];

      if(pt->distance_mm == 0u) {
         g_stats.dropped_zero++;
         continue;
      }
      if(pt->intensity < cfg->min_intensity) {
         g_stats.dropped_intens++;
         continue;
      }
      if(pt->distance_mm < cfg->min_dist_mm ||
         pt->distance_mm > cfg->max_dist_mm) {
         g_stats.dropped_range++;
         continue;
      }

      /* Compact: move to front if not already there. */
      if(out != i)
         scan->points[out] = scan->points[i];
      out++;
   }
   scan->count = out;

   /* ------------------------------------------------------------------
   * Stage 2: median filter (optional).
   *
   * For each surviving point, replace its distance with the median of
   * the window centred on it.  Points at the boundary of the scan
   * (first and last half-window) use the available neighbours only —
   * the window is clamped to the scan edges rather than wrapping, to
   * avoid introducing cross-0° artefacts.
   *
   * The filter runs on a copy of the distance values so that replaced
   * values do not influence their own neighbours.
   * ----------------------------------------------------------------*/
   uint8_t w = cfg->median_window;
   if(w >= 3u && scan->count >= w) {
      /* Clamp to supported window sizes. */
      if(w > 5u) w = 5u;
      if((w & 1u) == 0u) w--;   /* must be odd */

      uint8_t half = w / 2u;

      /* Scratch copy of distances — stack allocation is fine; at most
      * 720 × 2 = 1440 bytes, but we iterate one at a time so only
      * the small window array is on the stack simultaneously.
      * We need the pre-filter distances for the whole scan, so we
      * temporarily store them in a separate pass.
      *
      * For ESP32 RAM minimisation we avoid a full scratch copy by
      * doing the median in two passes: first collect originals into
      * a tiny ring of width w, then update in place going left-to-
      * right.  Because we only look backward into already-processed
      * positions by at most half, and the right half reads from
      * unmodified positions, this in-place approach is correct for
      * odd windows.
      *
      * However, to keep the logic unambiguous and easy to audit, we
      * do use a full scratch copy here.  scan->count ≤ 720, so the
      * maximum scratch is 720 × 2 = 1440 bytes on the stack — within
      * ESP32-C6's default 8 KB task stack.
      */
      uint16_t dist_orig[LIDAR_MAX_POINTS];
      for(uint16_t i = 0; i < scan->count; i++)
         dist_orig[i] = scan->points[i].distance_mm;

      for(uint16_t i = 0; i < scan->count; i++) {
         /* Build window around position i, clamped to array bounds. */
         uint16_t lo = (i >= half)
            ? (uint16_t)(i - half)
            : 0u;
         uint16_t hi = (i + half < scan->count)
            ? (uint16_t)(i + half)
            : (uint16_t)(scan->count - 1u);
         uint8_t  actual_w = (uint8_t)(hi - lo + 1u);

         uint16_t window[5];
         for(uint8_t k = 0; k < actual_w; k++) {
            window[k] = dist_orig[lo + k];
         }

         scan->points[i].distance_mm = median_u16(window, actual_w);
         g_stats.median_applied++;
      }
   }

   g_stats.out_count = scan->count;
}

/* =========================================================================
* Diagnostics
* ====================================================================== */

void scan_filter_get_stats(ScanFilterStats *out) {
   if(!out) return;
   *out = g_stats;
}