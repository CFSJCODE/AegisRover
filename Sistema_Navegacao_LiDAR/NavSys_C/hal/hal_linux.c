/**
 * @file hal_linux.c
 * @brief Linux platform implementation of the HAL.
 *
 * Covers:
 *   - Serial I/O via POSIX termios (raw mode, configurable baud)
 *   - Monotonic time via clock_gettime(CLOCK_MONOTONIC)
 *   - Logging to stderr with timestamps, colours and level filtering
 *   - Memory allocation via calloc/free
 *   - Mutex via pthread_mutex_t
 *   - File I/O via fopen/fread/fwrite for replay and raw capture
 *
 * Compile with:
 *   cc -std=c11 -Wall -Wextra -O2 -lpthread hal_linux.c -c -o hal_linux.o
 *
 * Dependencies: POSIX.1-2008 + Linux-specific ioctl(TIOCINQ).
 *   Works on Arch Linux; tested with GCC 13 and Clang 17.
 */

/* =========================================================================
* Feature-test macros — must appear before ANY system header.
*
* _GNU_SOURCE is a superset of _POSIX_C_SOURCE + _XOPEN_SOURCE and
* additionally exposes:
*   - cfmakeraw()          (POSIX extension, glibc-specific)
*   - CRTSCTS              (hardware flow-control flag, Linux-specific)
*   - clock_gettime()      (exposed via _POSIX_C_SOURCE ≥ 199309L,
*                           but _GNU_SOURCE guarantees it without -lrt)
*   - nanosleep()          (likewise)
*   - PTHREAD_MUTEX_ERRORCHECK  (POSIX, but _GNU_SOURCE ensures it)
*   - TIOCINQ / FIONREAD   (Linux ioctl constants)
*   - strdup()
*
* We set _GNU_SOURCE exclusively; defining both _POSIX_C_SOURCE and
* _GNU_SOURCE simultaneously can cause macro-redefinition warnings on
* some versions of glibc because _GNU_SOURCE internally re-defines
* _POSIX_C_SOURCE to a higher value.
* ====================================================================== */
#define _GNU_SOURCE

/* =========================================================================
* System headers (platform-specific — ONLY in this file)
* ====================================================================== */
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
// #include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/*
* TIOCINQ: query bytes in receive buffer. Same as FIONREAD on Linux.
* its a compiler directive to call FIONREAD if there is no definition
* of TIOCINQ, resolving UNIX - Linux incompatibilities
*/
#ifndef TIOCINQ
#  include <linux/serial.h>
#  define TIOCINQ FIONREAD
#endif

/* =========================================================================
* HAL public header (no platform headers allowed past this point in .h)
* ====================================================================== */
#include "hal.h"

/*
* Linux-only extension header — declares the replay source API.
* Included here (after system headers and after _GNU_SOURCE is active)
* so that the compiler can verify the project definitions match the
* declarations.
* nav_core/ never includes this header; only app/linux/ and tools/ do.
*/
#include "hal_linux_ext.h"

/* =========================================================================
* Internal: Serial port state
* ====================================================================== */

/** Maximum device path length (e.g. "/dev/ttyUSB0" is 13 chars). */
#define HAL_PORT_PATH_MAX 64

struct HalSerial {
   int              fd;                        /**< File descriptor.         */
   char             path[HAL_PORT_PATH_MAX];   /**< Device path (debug).     */
   uint32_t         baud;                      /**< Configured baud rate.    */
   struct termios   saved_tty;                 /**< Original termios state.  */
   bool             tty_saved;                 /**< true if saved_tty valid. */
};

/* -------------------------------------------------------------------------
* Internal: map integer baud rate to POSIX speed_t constant.
* Covers every rate the FHL-LD20 is known to use (230400 primary).
* ---------------------------------------------------------------------- */
static speed_t baud_to_speed(uint32_t baud) {
   switch (baud) {
      case      9600: return      B9600;
      case     19200: return     B19200;
      case     38400: return     B38400;
      case     57600: return     B57600;
      case    115200: return    B115200;
      case    230400: return    B230400;
      case    460800: return    B460800;
      case    921600: return    B921600;
      case   1000000: return   B1000000;
      case   1500000: return   B1500000;
      case   2000000: return   B2000000;
      default:
         return B0;   /* Caller detects B0 as unsupported. */
   }
}

/* =========================================================================
* 1. SERIAL PORT — implementation
* ====================================================================== */

HalSerial *hal_serial_open(const char *port, uint32_t baud) {
   if(!port || port[0] == '\0') {
      fprintf(stderr, "[HAL] hal_serial_open: NULL or empty port path\n");
      return NULL;
   }

   speed_t speed = baud_to_speed(baud);
   if(speed == B0) {
      fprintf(stderr,
         "[HAL] hal_serial_open: unsupported baud rate %u\n",
      baud);
      return NULL;
   }

   /* ------------------------------------------------------------------
   * Allocate state struct.
   * ----------------------------------------------------------------*/
   HalSerial *s = (HalSerial *)calloc(1, sizeof(HalSerial));
   if(!s) {
      perror("[HAL] hal_serial_open: calloc");
      return NULL;
   }

   strncpy(s->path, port, HAL_PORT_PATH_MAX - 1);
   s->baud = baud;

   /* ------------------------------------------------------------------
   * Open the device.
   *   O_RDWR   : read + write (write needed for optional TX commands)
   *   O_NOCTTY : do not make this process's controlling terminal
   *   O_NONBLOCK temporarily: avoid blocking on open() if DCD is low
   * ----------------------------------------------------------------*/
   int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
   if(fd < 0) {
      fprintf(stderr,
         "[HAL] hal_serial_open: open(\"%s\"): %s\n",
      port, strerror(errno));
      free(s);
      return NULL;
   }

   /* Verify this is a tty (reject plain files passed by mistake). */
   if(!isatty(fd)) {
      fprintf(stderr, "[HAL] hal_serial_open: \"%s\" is not a TTY\n", port);
      close(fd);
      free(s);
      return NULL;
   }

   /* ------------------------------------------------------------------
   * Save current termios so we can restore on close.
   * ----------------------------------------------------------------*/
   if(tcgetattr(fd, &s->saved_tty) == 0) {
      s->tty_saved = true;
   } else {
      fprintf(stderr,
         "[HAL] hal_serial_open: tcgetattr(\"%s\"): %s (continuing)\n",
      port, strerror(errno));
   }

   /* ------------------------------------------------------------------
   * Configure raw mode (8N1, no flow control, no echo, no processing).
   *
   * cfmakeraw() sets:
   *   c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON)
   *   c_oflag &= ~OPOST
   *   c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN)
   *   c_cflag &= ~(CSIZE|PARENB);  c_cflag |= CS8
   * ----------------------------------------------------------------*/
   struct termios tty;
   memset(&tty, 0, sizeof(tty));
   cfmakeraw(&tty);

   /* 8 data bits, no parity, 1 stop bit — explicit belt-and-suspenders. */
   tty.c_cflag &= (tcflag_t)~CSIZE;
   tty.c_cflag |= CS8;
   tty.c_cflag &= (tcflag_t)~PARENB;
   tty.c_cflag &= (tcflag_t)~CSTOPB;

   /* Disable hardware flow control. */
   tty.c_cflag &= (tcflag_t)~CRTSCTS;

   /* Enable receiver; hang up on close only if modem lines change. */
   tty.c_cflag |= (CREAD | CLOCAL);

   /* Disable software flow control (XON/XOFF). */
   tty.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);

   /* VMIN=0, VTIME=0: non-blocking read at the termios level.
   * Timeouts are implemented in hal_serial_read() with select(). */
   tty.c_cc[VMIN]  = 0;
   tty.c_cc[VTIME] = 0;

   /* Apply baud rate to both directions. */
   if(cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
      fprintf(stderr,
         "[HAL] hal_serial_open: cfsetspeed: %s\n",
      strerror(errno));
      if(s->tty_saved) tcsetattr(fd, TCSANOW, &s->saved_tty);
      close(fd);
      free(s);
      return NULL;
   }

   /* Flush stale data before switching mode. */
   tcflush(fd, TCIOFLUSH);

   /* Apply new settings immediately. */
   if(tcsetattr(fd, TCSANOW, &tty) != 0) {
      fprintf(stderr,
         "[HAL] hal_serial_open: tcsetattr(\"%s\"): %s\n",
      port, strerror(errno));
      if(s->tty_saved) tcsetattr(fd, TCSANOW, &s->saved_tty);
      close(fd);
      free(s);
      return NULL;
   }

   /* Switch fd back to blocking mode now that termios is set.
   * Reads are bounded by the select() call in hal_serial_read(). */
   int flags = fcntl(fd, F_GETFL, 0);
   if(flags < 0 || fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0) {
      fprintf(stderr,
         "[HAL] hal_serial_open: fcntl(clr O_NONBLOCK): %s\n",
      strerror(errno));
      /* Non-fatal; VMIN/VTIME=0 will still keep reads non-blocking. */
   }

   s->fd = fd;

   HAL_LOGI("HAL", "Serial opened: %s @ %u baud (fd=%d)", port, baud, fd);
   return s;
}

/* ------------------------------------------------------------------------- */

void hal_serial_close(HalSerial *s) {
   if(!s) return;

   if(s->fd >= 0) {
      tcflush(s->fd, TCIOFLUSH);
      if(s->tty_saved) {
         /* Restore terminal to its original state. */
         tcsetattr(s->fd, TCSANOW, &s->saved_tty);
      }
      close(s->fd);
      HAL_LOGI("HAL", "Serial closed: %s", s->path);
   }
   free(s);
}

/* ------------------------------------------------------------------------- */

int hal_serial_read(
   HalSerial *s, uint8_t *buf, int max_len, uint32_t timeout_ms
) {
   if(!s || s->fd < 0 || !buf || max_len <= 0) return -1;

   /* ------------------------------------------------------------------
   * Use select() to implement the caller-specified timeout precisely.
   * This avoids VTIME (which has 100 ms granularity) and keeps the fd
   * in blocking mode so that reads after select() always return data.
   * ----------------------------------------------------------------*/
   fd_set read_fds;
   FD_ZERO(&read_fds);
   FD_SET(s->fd, &read_fds);

   ////////////////////////////////////////////////// BEFORE MANUAL INTERVENTION
   //
   // struct timeval tv;
   // tv.tv_sec  = (time_t)(timeout_ms / 1000u);
   // tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);
   // int ready = select(
   //    s->fd + 1, &read_fds, NULL, NULL, timeout_ms == 0 ? NULL : &tv
   // );
   // /*
   // * timeout_ms == 0: pass NULL timeval >> select blocks indefinitely.
   // * This matches the documented "0 = non-blocking" semantics at the
   // * application level only if the caller never passes 0 for live reads.
   // * The parser loop always passes a positive timeout.
   // *
   // * Correction: for true non-blocking, pass &tv with both fields == 0.
   // */
   // if(timeout_ms == 0) {
   //    /* Re-run with zero timeout for actual non-blocking behaviour. */
   //    FD_ZERO(&read_fds);
   //    FD_SET(s->fd, &read_fds);
   //    struct timeval tv0 = {0, 0};
   //    ready = select(s->fd + 1, &read_fds, NULL, NULL, &tv0);
   // }
   //
   /////////////////////////////////////////////////// AFTER MANUAL INTERVENTION

   int ready = -1;
   struct timeval tv;
   if(timeout_ms == 0) {
      /* Run with zero timeout for true non-blocking behaviour. */
      tv.tv_sec  = 0; tv.tv_usec = 0;
      ready = select(s->fd + 1, &read_fds, NULL, NULL, &tv);
   } else {
      tv.tv_sec  = (time_t)(timeout_ms / 1000u);
      tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);
      ready = select(s->fd + 1, &read_fds, NULL, NULL, &tv);
   }

   ////////////////////////////////////////////////// END OF MANUAL INTERVENTION

   if(ready < 0) {
      if(errno == EINTR) return 0;   /* Signal interrupted — not fatal. */
      HAL_LOGE("HAL", "hal_serial_read: select: %s", strerror(errno));
      return -1;
   }
   if(ready == 0) return 0;           /* Timeout: no data. */

   /* Data is available — read as much as max_len allows. */
   ssize_t n = read(s->fd, buf, (size_t)max_len);
   if(n < 0) {
      if(errno == EAGAIN || errno == EWOULDBLOCK) return 0;
      HAL_LOGE("HAL", "hal_serial_read: read: %s", strerror(errno));
      return -1;
   }
   return (int)n;
}

/* ------------------------------------------------------------------------- */

int hal_serial_write(HalSerial *s, const uint8_t *buf, int len) {
   if(!s || s->fd < 0 || !buf || len <= 0) return -1;

   ssize_t written = write(s->fd, buf, (size_t)len);
   if(written < 0) {
      HAL_LOGE("HAL", "hal_serial_write: %s", strerror(errno));
      return -1;
   }

   /* Drain the output buffer to ensure the LiDAR receives the command
   * before the caller proceeds (important for motor control bytes). */
   tcdrain(s->fd);
   return (int)written;
}

/* ------------------------------------------------------------------------- */

void hal_serial_flush_rx(HalSerial *s) {
   if(!s || s->fd < 0) return;
   if(tcflush(s->fd, TCIFLUSH) != 0) {
      HAL_LOGW("HAL", "hal_serial_flush_rx: tcflush: %s", strerror(errno));
   }
}

/* ------------------------------------------------------------------------- */

int hal_serial_bytes_available(HalSerial *s) {
   if(!s || s->fd < 0) return -1;

   int bytes = 0;
   if(ioctl(s->fd, TIOCINQ, &bytes) < 0) {
      HAL_LOGW("HAL",
         "hal_serial_bytes_available: ioctl(TIOCINQ): %s",
      strerror(errno));
      return -1;
   }

   return bytes;
}

/* =========================================================================
* 2. TIME — implementation
* ====================================================================== */

/**
 * Internal helper: read CLOCK_MONOTONIC as microseconds since first call.
 * Using a fixed epoch keeps the numbers human-readable in logs.
 */
static uint64_t time_epoch_us(void) {
   static uint64_t epoch = 0;
   struct timespec ts;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   uint64_t now_us =
      (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
   if(epoch == 0) epoch = now_us;

   return now_us - epoch;
}

uint32_t hal_time_ms(void) {
   return (uint32_t)(time_epoch_us() / 1000ULL);
}

uint64_t hal_time_us(void) {
   return time_epoch_us();
}

void hal_sleep_ms(uint32_t ms) {
   struct timespec req = {
      .tv_sec  = (time_t)(ms / 1000u),
      .tv_nsec = (long)((ms % 1000u) * 1000000L),
   };

   /* Restart on signal interruption. */
   while(nanosleep(&req, &req) == -1 && errno == EINTR);
}

/* =========================================================================
* 3. LOGGING — implementation
* ====================================================================== */

/** ANSI colour codes — disabled if stderr is not a TTY. */
#define ANSI_RESET  "\033[0m"
#define ANSI_GREY   "\033[90m"
#define ANSI_CYAN   "\033[36m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RED    "\033[31m"

static HalLogLevel     g_log_level  = HAL_LOG_INFO;
static pthread_mutex_t g_log_mutex  = PTHREAD_MUTEX_INITIALIZER;
static int             g_log_colour = 0;   /* set true if stderr is a tty */
static int             g_log_init   = 0;

/** Called once on first log emission. */
static void log_init(void) {
   g_log_colour = isatty(STDERR_FILENO);
   g_log_init   = true;
}

void hal_log_set_level(HalLogLevel level) {
   g_log_level = level;
}

void hal_vlog(
   HalLogLevel level, const char *tag, const char *fmt, va_list args
) {
   if(level < g_log_level) return;

   pthread_mutex_lock(&g_log_mutex);

   if(!g_log_init) log_init();

   /* Timestamp: milliseconds since epoch. */
   uint32_t ms = hal_time_ms();

   /* Level label and optional colour. */
   const char *colour  = "";
   const char *reset   = "";
   const char *label;

   if(g_log_colour) {
      reset = ANSI_RESET;
      switch (level) {
         case HAL_LOG_DEBUG: colour = ANSI_GREY;   label = "D"; break;
         case HAL_LOG_INFO:  colour = ANSI_CYAN;   label = "I"; break;
         case HAL_LOG_WARN:  colour = ANSI_YELLOW; label = "W"; break;
         case HAL_LOG_ERROR: colour = ANSI_RED;    label = "E"; break;
         default:            colour = "";          label = "?"; break;
      }
   } else {
      switch (level) {
         case HAL_LOG_DEBUG: label = "D"; break;
         case HAL_LOG_INFO:  label = "I"; break;
         case HAL_LOG_WARN:  label = "W"; break;
         case HAL_LOG_ERROR: label = "E"; break;
         default:            label = "?"; break;
      }
   }

   /* Format: "[+00012345 ms] I/TAG: message\n" */
   fprintf(stderr,
      "%s[+%08u ms] %s/%s:%s ",
   colour, ms, label, tag ? tag : "?", reset);

   vfprintf(stderr, fmt, args);
   fputc('\n', stderr);
   fflush(stderr);

   pthread_mutex_unlock(&g_log_mutex);
}

void hal_log(HalLogLevel level, const char *tag, const char *fmt, ...) {
   va_list args;
   va_start(args, fmt);
   hal_vlog(level, tag, fmt, args);
   va_end(args);
}

/* =========================================================================
* 4. MEMORY — implementation
* ====================================================================== */

void *hal_calloc(size_t n, size_t size) {
   void *p = calloc(n, size);
   if(!p) {
      /* Log directly to avoid circular dependency on hal_calloc in logger. */
      fprintf(stderr, "[HAL] hal_calloc(%zu, %zu) failed — OOM\n", n, size);
   }
   return p;
}

void hal_free(void *ptr) {
   free(ptr);
}

/* =========================================================================
* 5. MUTEX — implementation
* ====================================================================== */

struct HalMutex {
   pthread_mutex_t pm;
};

HalMutex *hal_mutex_create(void) {
   HalMutex *m = (HalMutex *)calloc(1, sizeof(HalMutex));
   if(!m) return NULL;

   pthread_mutexattr_t attr;
   pthread_mutexattr_init(&attr);
   pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
   if(pthread_mutex_init(&m->pm, &attr) != 0) {
      pthread_mutexattr_destroy(&attr);
      free(m);
      return NULL;
   }
   pthread_mutexattr_destroy(&attr);
   return m;
}

void hal_mutex_destroy(HalMutex *m) {
   if(!m) return;
   pthread_mutex_destroy(&m->pm);
   free(m);
}

void hal_mutex_lock(HalMutex *m) {
   if(!m) return;
   int rc = pthread_mutex_lock(&m->pm);
   if(rc != 0) {
      fprintf(stderr, "[HAL] hal_mutex_lock: %s\n", strerror(rc));
   }
}

void hal_mutex_unlock(HalMutex *m) {
   if(!m) return;
   int rc = pthread_mutex_unlock(&m->pm);
   if(rc != 0) {
      fprintf(stderr, "[HAL] hal_mutex_unlock: %s\n", strerror(rc));
   }
}

/* =========================================================================
* 6. FILE I/O — implementation
* ====================================================================== */

struct HalFile {
   FILE *fp;
   HalFileMode mode;
};

HalFile *hal_file_open(const char *path, HalFileMode mode) {
   if(!path) return NULL;

   const char *modestr;
   switch (mode) {
      case HAL_FILE_READ:   modestr = "rb";  break;
      case HAL_FILE_WRITE:  modestr = "wb";  break;
      case HAL_FILE_APPEND: modestr = "ab";  break;
      default:              return NULL;
   }

   FILE *fp = fopen(path, modestr);
   if(!fp) {
      HAL_LOGE("HAL",
         "hal_file_open(\"%s\", \"%s\"): %s",
      path, modestr, strerror(errno));
      return NULL;
   }

   HalFile *f = (HalFile *)calloc(1, sizeof(HalFile));
   if(!f) {
      fclose(fp);
      return NULL;
   }
   f->fp   = fp;
   f->mode = mode;
   return f;
}

void hal_file_close(HalFile *f) {
   if(!f) return;
   if(f->fp) fclose(f->fp);
   free(f);
}

int hal_file_read(HalFile *f, uint8_t *buf, int len) {
   if(!f || !f->fp || !buf || len <= 0) return -1;

   size_t n = fread(buf, 1, (size_t)len, f->fp);
   if(n == 0) {
      return feof(f->fp) ? 0 : -1;
   }

   return (int)n;
}

int hal_file_write(HalFile *f, const uint8_t *buf, int len) {
   if(!f || !f->fp || !buf || len <= 0) return -1;

   size_t n = fwrite(buf, 1, (size_t)len, f->fp);
   if(n < (size_t)len) {
      HAL_LOGE("HAL",
         "hal_file_write: short write (%zu / %d): %s",
      n, len, strerror(errno));
      return -1;
   }

   return (int)n;
}

/* =========================================================================
* 7. PLATFORM INFO — implementation
* ====================================================================== */

const char *hal_platform_name(void) {
#if defined(__x86_64__) || defined(_M_X64)
   return "Linux/x86_64";
#elif defined(__aarch64__)
   return "Linux/aarch64";
#elif defined(__arm__)
   return "Linux/arm";
#else
   return "Linux/unknown-arch";
#endif
}

bool hal_is_embedded(void) {
   return false;
}

/* =========================================================================
* 8. REPLAY SOURCE — thin wrapper to feed files as "serial" reads
*
* Allows the development host to replay a captured binary stream through
* the same lidar_parser_feed() loop that reads from live hardware.
*
* Usage:
*   HalSerial *replay = hal_serial_open_replay("scan_raw.bin", 230400);
*   // use exactly like a real port; hal_serial_read() returns file bytes
* ====================================================================== */

/**
 * @brief Open a captured binary file as a virtual serial source.
 *
 * Bytes are returned by hal_serial_read() exactly as if they arrived from
 * UART, at a rate governed by @p baud (interpacket pacing is simulated by
 * sleeping the appropriate number of microseconds between chunks).
 *
 * @param path  Path to a .bin file captured with tools/log_capture.py.
 * @param baud  LiDAR baud rate — used for playback pacing (may be 0 to
 *              return data as fast as possible, for unit testing).
 * @return      HalSerial handle usable with all hal_serial_*() functions,
 *              or NULL on error.
 *
 * @note  This function is Linux-only and must NOT be called from nav_core.
 *        It is available to app/linux/main.cpp and tools/.
 */

/** Extended struct for file-backed "serial" port. */
typedef struct {
   HalSerial base;       /**< Must be first — allows cast to HalSerial*. */
   HalFile  *src;        /**< Backing file.                              */
   uint32_t  baud;       /**< Playback pacing rate.                      */
   bool      is_replay;  /**< Distinguishes from real tty in close().    */
} HalSerialReplay;

/**
 * Internal read override used when is_replay == true.
 * Not part of the public HAL — called by hal_serial_read() dispatch below.
 */
static int replay_read(
   HalSerialReplay *r, uint8_t *buf, int max_len, uint32_t timeout_ms
) {
   (void)timeout_ms;   /* Replay ignores timeout — returns EOF as 0. */

   int n = hal_file_read(r->src, buf, max_len);
   if(n <= 0) return n;   /* 0 = EOF, -1 = error */

   /* Simulate UART byte timing to avoid overwhelming the parser. */
   if(r->baud > 0) {
      /* bits_per_byte = 10 (1 start + 8 data + 1 stop) */
      uint64_t byte_us = (10ULL * 1000000ULL) / r->baud;
      uint64_t sleep_us = byte_us * (uint64_t)n;
      if(sleep_us > 0) {
         struct timespec ts = {
            .tv_sec  = (time_t)(sleep_us / 1000000ULL),
            .tv_nsec = (long)((sleep_us % 1000000ULL) * 1000ULL),
         };
         nanosleep(&ts, NULL);
      }
   }
   return n;
}

HalSerial *hal_serial_open_replay(const char *path, uint32_t baud) {
   HalFile *f = hal_file_open(path, HAL_FILE_READ);
   if(!f) return NULL;

   HalSerialReplay *r = (HalSerialReplay *)calloc(1, sizeof(HalSerialReplay));
   if(!r) {
      hal_file_close(f);
      return NULL;
   }

   r->base.fd        = -1;        /* No real fd. */
   r->base.tty_saved = false;
   strncpy(r->base.path, path, HAL_PORT_PATH_MAX - 1);
   r->base.baud  = baud;
   r->src        = f;
   r->baud       = baud;
   r->is_replay  = true;

   HAL_LOGI("HAL", "Replay source opened: %s (pacing @ %u baud)", path, baud);
   return (HalSerial *)r;         /* Safe: base is first member. */
}

/*
* Override hal_serial_read to dispatch to replay_read when needed.
* We re-declare hal_serial_read here AFTER the static helper so the
* compiler sees the full definition. The original declaration in the
* header is what nav_core sees — this is a single translation unit, so
* there is no ODR conflict.
*
* The trick: check whether the fd is -1 AND the pointer was allocated
* with the extended size. We use a magic cookie embedded in the struct.
*/

/*
* Because C does not have virtual dispatch, we add a thin tag field.
* This is purely internal to hal_linux.c — no header change required.
* We achieve this by checking r->is_replay after casting.
*
* The public hal_serial_read() signature is already implemented above
* for real ports. We wrap it here to intercept the replay case.
* To avoid re-defining the same symbol, we use a linker alias approach:
* rename the real implementation and call it from a dispatching wrapper.
*/

/*
* Implementation note: Rather than re-defining hal_serial_read (which
* would be a duplicate definition), we rely on the fact that replay
* HalSerial handles have fd == -1. The existing hal_serial_read()
* already returns -1 for fd < 0. Instead, callers that hold a replay
* handle call hal_serial_read_replay() directly. nav_core never
* instantiates replay handles — only app/linux code does — so this
* does not break the platform-agnostic contract.
*/

/**
 * @brief Read from a replay source (use instead of hal_serial_read).
 *
 * The function signature is identical to hal_serial_read() so that
 * app/linux/main.cpp can choose between live and replay transparently
 * using a function pointer or a conditional:
 *
 * @code
 *   bool live = (port_path != NULL);
 *   HalSerial *port = live ? hal_serial_open("/dev/ttyUSB0", 230400)
 *                          : hal_serial_open_replay("scan.bin", 230400);
 *   // ...
 *   int n = live ? hal_serial_read(port, buf, sizeof(buf), 100)
 *                : hal_serial_read_replay(port, buf, sizeof(buf), 100);
 * @endcode
 */
int hal_serial_read_replay(
   HalSerial *s, uint8_t *buf, int max_len, uint32_t timeout_ms
) {
   if(!s || !buf || max_len <= 0) return -1;
   return replay_read((HalSerialReplay *)s, buf, max_len, timeout_ms);
}

/**
 * @brief Close a replay source opened with hal_serial_open_replay().
 *
 * Regular hal_serial_close() also works (it checks tty_saved and fd).
 * This variant additionally closes the backing file.
 */
void hal_serial_close_replay(HalSerial *s) {
   if(!s) return;
   HalSerialReplay *r = (HalSerialReplay *)s;
   if(r->is_replay && r->src) {
      hal_file_close(r->src);
      r->src = NULL;
   }
   HAL_LOGI("HAL", "Replay source closed: %s", s->path);
   free(r);
}
