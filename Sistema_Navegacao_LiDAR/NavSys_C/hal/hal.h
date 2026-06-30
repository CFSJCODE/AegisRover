/**
 * @file hal.h
 * @brief Hardware Abstraction Layer — platform-agnostic interface.
 *
 * Every symbol in nav_core/ that touches hardware MUST go through this
 * header.  No platform headers (<termios.h>, <driver/uart.h>, etc.) may
 * be included by nav_core modules.
 *
 * Platform implementations:
 *   hal/hal_linux.c   — POSIX / termios  (development & testing)
 *   hal/hal_esp32.c   — ESP-IDF / FreeRTOS (deployment)
 *
 * Porting checklist for a new platform:
 *   1. Implement every function declared below.
 *   2. Define HAL_PLATFORM_NAME as a string literal.
 *   3. Link the new .c instead of the others — no other files change.
 */

#ifndef HAL_H
#define HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* size_t */
#include <stdarg.h>   /* va_list for hal_vlog */

/* -------------------------------------------------------------------------
* Compile-time platform tag (defined by the build system, not this header).
*
*   Linux CMake:     -DHAL_PLATFORM=HAL_PLATFORM_LINUX
*   ESP-IDF / PIO:   -DHAL_PLATFORM=HAL_PLATFORM_ESP32
* ---------------------------------------------------------------------- */
#define HAL_PLATFORM_LINUX  1
#define HAL_PLATFORM_ESP32  2

#ifndef HAL_PLATFORM
#  ifdef __linux__
#    define HAL_PLATFORM HAL_PLATFORM_LINUX
#  else
#    define HAL_PLATFORM HAL_PLATFORM_ESP32
#  endif
#endif

/* =========================================================================
* 1. SERIAL PORT
* ====================================================================== */

/**
 * @brief Opaque handle for a serial port.
 *
 * The internal layout is defined only inside the platform implementation.
 * Callers hold a pointer; they never dereference it directly.
 */
typedef struct HalSerial HalSerial;

/**
 * @brief Open and configure a UART serial port.
 *
 * On Linux  : @p port is a device path, e.g. "/dev/ttyUSB0".
 * On ESP32  : @p port is ignored; the UART number is fixed at build time
 *             via HAL_ESP32_UART_NUM in nav_config.h.
 *
 * The port is opened in raw (non-canonical) mode, 8N1, no flow control,
 * with reads returning as soon as any byte is available (VMIN=0, VTIME=0).
 * Timeout is managed by the caller through hal_serial_read().
 *
 * @param port     Device path string (Linux) or ignored (ESP32).
 * @param baud     Baud rate, e.g. 230400.  Must be an exact standard rate.
 * @return         Non-NULL handle on success, NULL on failure.
 */
HalSerial *hal_serial_open(const char *port, uint32_t baud);

/**
 * @brief Close and release a serial port.
 *
 * Safe to call with NULL (no-op).
 *
 * @param s  Handle returned by hal_serial_open().
 */
void hal_serial_close(HalSerial *s);

/**
 * @brief Read raw bytes from the serial port.
 *
 * Blocks until at least 1 byte is available or @p timeout_ms elapses.
 * Never blocks longer than @p timeout_ms regardless of @p max_len.
 * Does NOT guarantee that @p max_len bytes are returned in one call —
 * the caller must loop (which lidar_parser_feed() already does).
 *
 * @param s           Open serial handle.
 * @param buf         Destination buffer; must be at least @p max_len bytes.
 * @param max_len     Maximum bytes to read.
 * @param timeout_ms  Maximum wait time in milliseconds.  0 = non-blocking.
 * @return            Number of bytes actually read (>= 0), or -1 on error.
 *                    0 means timeout with no data.
 */
int hal_serial_read(HalSerial *s, uint8_t *buf, int max_len,
                  uint32_t timeout_ms);

/**
 * @brief Write raw bytes to the serial port (optional, for LiDAR commands).
 *
 * The FHL-LD20 does not require commands during normal operation, but the
 * function is provided for completeness and future use (e.g. motor speed
 * control over UART).
 *
 * @param s    Open serial handle.
 * @param buf  Data to transmit.
 * @param len  Number of bytes to write.
 * @return     Number of bytes written, or -1 on error.
 */
int hal_serial_write(HalSerial *s, const uint8_t *buf, int len);

/**
 * @brief Flush (discard) any unread bytes in the receive buffer.
 *
 * Useful after an error to resynchronise the parser.
 *
 * @param s  Open serial handle.
 */
void hal_serial_flush_rx(HalSerial *s);

/**
 * @brief Query how many bytes are waiting in the receive buffer.
 *
 * Returns 0 if none available or if the query is not supported.
 *
 * @param s  Open serial handle.
 * @return   Approximate number of bytes available, or -1 on error.
 */
int hal_serial_bytes_available(HalSerial *s);

/* =========================================================================
* 2. TIME
* ====================================================================== */

/**
 * @brief Monotonic millisecond counter.
 *
 * Wraps at UINT32_MAX (~49 days).  The epoch is undefined; only differences
 * between successive calls are meaningful.
 *
 * @return Current time in milliseconds.
 */
uint32_t hal_time_ms(void);

/**
 * @brief Monotonic microsecond counter.
 *
 * Wraps at UINT64_MAX (never in practice).  Used for profiling hot paths.
 *
 * @return Current time in microseconds.
 */
uint64_t hal_time_us(void);

/**
 * @brief Suspend the calling thread for at least @p ms milliseconds.
 *
 * On ESP32 this yields to the FreeRTOS scheduler.
 *
 * @param ms  Sleep duration in milliseconds.
 */
void hal_sleep_ms(uint32_t ms);

/* =========================================================================
* 3. LOGGING
* ====================================================================== */

/** Log severity levels — match conventional syslog ordering. */
typedef enum {
   HAL_LOG_DEBUG = 0,  /**< Verbose internal state dumps.              */
   HAL_LOG_INFO  = 1,  /**< Normal operational messages.               */
   HAL_LOG_WARN  = 2,  /**< Recoverable anomalies (e.g. CRC error).   */
   HAL_LOG_ERROR = 3,  /**< Unrecoverable errors; operation aborted.  */
} HalLogLevel;

/**
 * @brief Set the minimum log level that will be emitted.
 *
 * Messages below this level are discarded.  Default: HAL_LOG_INFO.
 *
 * @param level  Minimum level to emit.
 */
void hal_log_set_level(HalLogLevel level);

/**
 * @brief Emit a formatted log message.
 *
 * Thread-safe on all supported platforms.
 * Format string follows printf() semantics.
 *
 * On Linux  : writes to stderr with timestamp and level prefix.
 * On ESP32  : forwards to ESP_LOGx macros (routed to UART0).
 *
 * @param level  Severity of this message.
 * @param tag    Short module identifier, e.g. "PARSER", "GRID", "ASTAR".
 * @param fmt    printf-style format string.
 * @param ...    Format arguments.
 */
void hal_log(HalLogLevel level, const char *tag, const char *fmt, ...)
   __attribute__((format(printf, 3, 4)));

/**
 * @brief va_list variant of hal_log — for wrappers in other modules.
 */
void hal_vlog(HalLogLevel level, const char *tag, const char *fmt,
            va_list args);

/* Convenience macros — map to hal_log with literal tag strings. */
#define HAL_LOGD(tag, fmt, ...) hal_log(HAL_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
#define HAL_LOGI(tag, fmt, ...) hal_log(HAL_LOG_INFO,  tag, fmt, ##__VA_ARGS__)
#define HAL_LOGW(tag, fmt, ...) hal_log(HAL_LOG_WARN,  tag, fmt, ##__VA_ARGS__)
#define HAL_LOGE(tag, fmt, ...) hal_log(HAL_LOG_ERROR, tag, fmt, ##__VA_ARGS__)

/* =========================================================================
* 4. MEMORY  (thin wrappers — allow future heap instrumentation / IRAM)
* ====================================================================== */

/**
 * @brief Allocate zeroed memory from the system heap.
 *
 * On Linux  : wraps calloc().
 * On ESP32  : wraps heap_caps_calloc(MALLOC_CAP_8BIT) for DRAM placement.
 *
 * nav_core modules call this exactly once per object, during init.
 * No calls occur in hot paths (parser feed / grid update / A*).
 *
 * @param n     Number of elements.
 * @param size  Size of each element in bytes.
 * @return      Zeroed allocation, or NULL on failure.
 */
void *hal_calloc(size_t n, size_t size);

/**
 * @brief Free a hal_calloc() allocation.
 *
 * Safe to call with NULL (no-op).
 *
 * @param ptr  Pointer returned by hal_calloc(), or NULL.
 */
void hal_free(void *ptr);

/* =========================================================================
* 5. CRITICAL SECTION  (used by the logger on multi-threaded Linux builds)
* ====================================================================== */

/** Opaque mutex handle. */
typedef struct HalMutex HalMutex;

/**
 * @brief Create a non-recursive mutex.
 *
 * @return  Non-NULL handle on success, NULL on failure.
 */
HalMutex *hal_mutex_create(void);

/**
 * @brief Destroy a mutex created with hal_mutex_create().
 *
 * Must not be held by any thread at the time of destruction.
 *
 * @param m  Mutex handle, or NULL (no-op).
 */
void hal_mutex_destroy(HalMutex *m);

/**
 * @brief Acquire the mutex (blocking).
 *
 * @param m  Non-NULL mutex handle.
 */
void hal_mutex_lock(HalMutex *m);

/**
 * @brief Release the mutex.
 *
 * @param m  Non-NULL mutex handle.
 */
void hal_mutex_unlock(HalMutex *m);

/* =========================================================================
* 6. FILE I/O  (Linux only — replay / logging)
* ====================================================================== */

/**
 * @brief Opaque raw-binary file handle.
 *
 * Used by the replay tool to inject captured UART data as if it came from
 * the serial port.  Not implemented on ESP32 (returns NULL stub).
 */
typedef struct HalFile HalFile;

/** File open modes. */
typedef enum {
   HAL_FILE_READ  = 0,
   HAL_FILE_WRITE = 1,
   HAL_FILE_APPEND = 2,
} HalFileMode;

/**
 * @brief Open a file for binary read or write.
 *
 * On ESP32 always returns NULL (SPIFFS/LittleFS support is outside scope).
 *
 * @param path  Filesystem path.
 * @param mode  One of HAL_FILE_READ / WRITE / APPEND.
 * @return      Non-NULL handle on success, NULL on failure.
 */
HalFile *hal_file_open(const char *path, HalFileMode mode);

/**
 * @brief Close a file handle.
 *
 * Safe to call with NULL.
 *
 * @param f  File handle.
 */
void hal_file_close(HalFile *f);

/**
 * @brief Read binary data from a file.
 *
 * @param f    Open file handle.
 * @param buf  Destination buffer.
 * @param len  Bytes to read.
 * @return     Bytes actually read, 0 = EOF, -1 = error.
 */
int hal_file_read(HalFile *f, uint8_t *buf, int len);

/**
 * @brief Write binary data to a file.
 *
 * @param f    Open file handle.
 * @param buf  Source buffer.
 * @param len  Bytes to write.
 * @return     Bytes written, or -1 on error.
 */
int hal_file_write(HalFile *f, const uint8_t *buf, int len);

/* =========================================================================
* 7. PLATFORM INFO
* ====================================================================== */

/**
 * @brief Return a human-readable platform identifier string.
 *
 * e.g. "Linux/x86_64" or "ESP32-C6/RISC-V"
 *
 * @return  Null-terminated constant string; never NULL.
 */
const char *hal_platform_name(void);

/**
 * @brief Return true if the current build is for an embedded target.
 *
 * nav_core modules may use this to skip features inappropriate for MCUs
 * (e.g. file logging), but must not change core algorithmic behaviour.
 */
bool hal_is_embedded(void);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* HAL_H */
