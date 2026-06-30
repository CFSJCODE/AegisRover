/**
 * @file nav_types.h
 * @brief Shared data types for the navigation core.
 *
 * Included by every nav_core module.  No platform headers here.
 * All integer widths are explicit (stdint.h) so the same types work
 * identically on Linux/x86-64 and ESP32-C6/RISC-V.
**/

#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
* LiDAR scan types
* ====================================================================== */

/** Maximum points stored per 360° revolution.
 *
 *  Measured from the FHL-LD20 at 2160 °/s with 230400 baud UART:
 *    step ≈ 6.48° per packet  →  36000 / 648 ≈ 55.6 packets per revolution
 *    55–56 packets × 12 points = 660–672 points per revolution.
 *
 *  720 provides a safe ceiling (56 × 12 = 672, +7% headroom) while keeping
 *  the LidarScan struct at 720 × 5 bytes = 3600 bytes — well within the
 *  ESP32-C6 DRAM budget.
**/
#define LIDAR_MAX_POINTS 720

/**
 * @brief A single distance measurement from the LiDAR.
 *
 * Angles are stored in centidegrees (integer × 0.01°) so the full
 * 0–360° range fits in a uint16_t (0–35999) with no float arithmetic.
**/
typedef struct {
   uint16_t angle_cdeg;    /**< Bearing, 0–35999 (0.01° per LSB). 0 = forward (sensor +X axis). */
   uint16_t distance_mm;   /**< Range in mm.  0 = invalid / no return.                          */
   uint8_t  intensity;     /**< Signal strength, 0–255. Low value → unreliable measurement.     */
} LidarPoint;

/**
 * @brief One complete 360° scan.
 *
 * Assembled by lidar_parser from multiple 12-point packets.
 * Points are stored in the order they arrive from the sensor
 * (monotonically increasing angle, wrapping through 0°).
**/
typedef struct {
   LidarPoint points[LIDAR_MAX_POINTS]; /**< Measured points.                               */
   uint16_t   count;                    /**< Valid entries in points[].                     */
   uint32_t   timestamp_ms;             /**< HAL time at scan completion.                   */
   uint16_t   speed_dps;                /**< Rotation speed, °/s. Averaged across the scan. */
} LidarScan;

#ifdef __cplusplus
}
#endif

#endif /* NAV_TYPES_H */
