/**
 * @file scan_filter.h
 * @brief LiDAR scan filter — public interface.
 *
 * Cleans a raw LidarScan before it is fed to the occupancy grid:
 *
 *   1. Intensity gate   — drops returns weaker than min_intensity.
 *   2. Range clamp      — drops points outside [min_dist_mm, max_dist_mm].
 *   3. Zero-distance    — drops points reported as 0 mm (no-return marker).
 *   4. Median filter    — replaces each surviving point's distance with the
 *                         median of itself and its angular neighbours.
 *                         Removes salt-and-pepper spikes while preserving
 *                         real edges.  Window size must be 0 (disabled),
 *                         3, or 5.
 *
 * All operations are in-place.  The scan's `count` field is updated to
 * reflect the number of surviving points; rejected points are compacted
 * away.  The original scan is never enlarged.
 *
 * No dynamic allocation, no floating-point, no platform headers.
 * Safe on ESP32-C6 with any optimisation level.
**/

#ifndef SCAN_FILTER_H
#define SCAN_FILTER_H

#include <stdint.h>
#include "nav_types.h"
#include "nav_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Configuration
 * ====================================================================== */

/**
 * @brief Runtime filter parameters.
 *
 * Initialise with scan_filter_cfg_default() then override as needed.
**/
typedef struct {
    uint16_t min_dist_mm;    /**< Reject points closer than this (blind zone). */
    uint16_t max_dist_mm;    /**< Reject points farther than this.             */
    uint8_t  min_intensity;  /**< Reject returns weaker than this.             */
    uint8_t  median_window;  /**< Median filter window: 0 (off), 3, or 5.     */
} ScanFilterCfg;

/**
 * @brief Fill @p cfg with the project defaults from nav_config.h.
 *
 * @param cfg  Non-NULL pointer to a ScanFilterCfg to initialise.
**/
void scan_filter_cfg_default(ScanFilterCfg *cfg);

/* =========================================================================
 * Core operation
 * ====================================================================== */

/**
 * @brief Apply the filter pipeline to @p scan in place.
 *
 * Stages run in this fixed order:
 *   zero-distance → intensity gate → range clamp → median filter
 *
 * The function modifies scan->points[] and scan->count.  All other fields
 * (timestamp_ms, speed_dps) are unchanged.
 *
 * @param scan  Non-NULL scan to filter.  Modified in place.
 * @param cfg   Non-NULL filter configuration.  Pass NULL to use defaults.
**/
void scan_filter_apply(LidarScan *scan, const ScanFilterCfg *cfg);

/* =========================================================================
 * Diagnostics
 * ====================================================================== */

/**
 * @brief Counters for the most recent scan_filter_apply() call.
 *
 * Reset at the start of every call; read after the call.
**/
typedef struct {
    uint16_t in_count;        /**< Points entering the filter.              */
    uint16_t out_count;       /**< Points surviving to output.              */
    uint16_t dropped_zero;    /**< Dropped: distance_mm == 0.               */
    uint16_t dropped_intens;  /**< Dropped: intensity below threshold.      */
    uint16_t dropped_range;   /**< Dropped: outside [min,max] range.        */
    uint16_t median_applied;  /**< Points processed by the median stage.    */
} ScanFilterStats;

/**
 * @brief Retrieve statistics from the most recent scan_filter_apply() call.
 *
 * @param out  Non-NULL destination struct.
**/
void scan_filter_get_stats(ScanFilterStats *out);

#ifdef __cplusplus
}
#endif

#endif /* SCAN_FILTER_H */