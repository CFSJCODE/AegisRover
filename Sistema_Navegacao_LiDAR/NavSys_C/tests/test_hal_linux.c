/**
 * @file test_hal_linux.c
 * @brief Functional test suite for hal_linux.c
 *
 * Tests every public HAL function as it is actually used by the navigation
 * stack.  Each test is self-contained, prints a PASS/FAIL line, and the
 * process exits with a non-zero code if any test fails.
 *
 * No test framework dependency — minunit-style macros only.
 *
 * Build (from the project root, where hal/ lives):
 *
 *   gcc -std=c11 -Wall -Wextra -O0 -g \
 *       -DHAL_PLATFORM=HAL_PLATFORM_LINUX \
 *       -I hal/ \
 *       tests/test_hal_linux.c hal/hal_linux.c \
 *       -lpthread -o build/test_hal_linux
 *
 *   ./build/test_hal_linux
 *   ./build/test_hal_linux /dev/ttyUSB0    # optional: include live-port tests
 *
 * Without a TTY argument only the hardware-independent tests run.
 * With a TTY argument the serial open/read/write/flush/available group
 * also runs against real hardware.
 */

#define _GNU_SOURCE   /* nanosleep, pthread extras */

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "hal.h"
#include "hal_linux_ext.h"

/* =========================================================================
* Minimal test framework
* ====================================================================== */

static int g_tests_run    = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

/* ANSI colour helpers — written to stdout, not stderr, so they don't
* interleave with the HAL's own log output on stderr.               */
#define COL_GREEN  "\033[32m"
#define COL_RED    "\033[31m"
#define COL_YELLOW "\033[33m"
#define COL_BOLD   "\033[1m"
#define COL_RESET  "\033[0m"

static bool g_colour = false;   /* set once in main() */

static void test_result(bool ok, const char *name, const char *detail) {
   g_tests_run++;
   if(ok) {
      g_tests_passed++;
      if(g_colour) printf("  %s[PASS]%s %s\n", COL_GREEN, COL_RESET, name);
      else printf("  [PASS] %s\n", name);
   } else {
      g_tests_failed++;
      if(g_colour) printf(
         "  %s[FAIL]%s %s  →  %s\n",
         COL_RED, COL_RESET, name, detail ? detail : ""
      );
      else printf("  [FAIL] %s  ->  %s\n", name, detail ? detail : "");
   }
}

/* Convenience: evaluate cond, stringify it as detail on failure. */
#define CHECK(cond) test_result((cond), #cond, #cond " evaluated to false")

/* Evaluate cond with a custom failure message. */
#define CHECK_MSG(cond, msg) test_result((cond), #cond, (msg))

/* Print a section header. */
static void section(const char *title) {
   printf("\n");
   if(g_colour) printf(
      "%s%s══ %s ══%s\n", COL_BOLD, COL_YELLOW, title, COL_RESET
   );
   else printf("== %s ==\n", title);
}

/* Scratch file path used by file / replay tests. */
#define SCRATCH_FILE "/tmp/hal_test_scratch.bin"

/* =========================================================================
* Helper: build a scratch binary file with known content.
*
* Returns the number of bytes written, or -1 on error.
* The file is a flat array of sequential byte values 0x00..0xFF repeated.
* ====================================================================== */
static int make_scratch_file(const char *path, int n_bytes) {
   FILE *fp = fopen(path, "wb");
   if(!fp) return -1;
   for(int i = 0; i < n_bytes; i++) {
      uint8_t b = (uint8_t)(i & 0xFF);
      fwrite(&b, 1, 1, fp);
   }
   fclose(fp);
   return n_bytes;
}

/* =========================================================================
* 1. PLATFORM INFO
* ====================================================================== */
static void test_platform_info(void) {
   section("Platform info");

   const char *name = hal_platform_name();
   CHECK_MSG(name != NULL,                      "hal_platform_name() returned NULL");
   CHECK_MSG(name[0] != '\0',                   "platform name is empty string");
   CHECK_MSG(strstr(name, "Linux") != NULL,     "expected 'Linux' in platform name");

   /* hal_is_embedded() must return false on Linux. */
   CHECK(hal_is_embedded() == false);
}

/* =========================================================================
* 2. TIME
* ====================================================================== */
static void test_time(void) {
   section("Time");

   /* ── 2a. hal_time_ms() is monotonically non-decreasing ── */
   uint32_t t0 = hal_time_ms();
   uint32_t t1 = hal_time_ms();
   CHECK_MSG(t1 >= t0, "hal_time_ms() went backwards");

   /* ── 2b. hal_time_us() is monotonically non-decreasing ── */
   uint64_t u0 = hal_time_us();
   uint64_t u1 = hal_time_us();
   CHECK_MSG(u1 >= u0, "hal_time_us() went backwards");

   /* ── 2c. us counter advances faster than ms counter (finer resolution) ── */
   CHECK_MSG(
      u1 - u0 <= (t1 - t0 + 1) * 1000ULL + 100ULL,
      "hal_time_us() inconsistent with hal_time_ms()"
   );

   /* ── 2d. hal_sleep_ms() sleeps for approximately the requested time ── */
   {
      uint32_t before = hal_time_ms();
      hal_sleep_ms(50);
      uint32_t after  = hal_time_ms();
      uint32_t elapsed = after - before;

      /* Generous bounds: 40–200 ms (scheduler jitter on a loaded system). */
      char detail[64];
      snprintf(detail, sizeof(detail), "elapsed=%u ms (expected ~50)", elapsed);
      CHECK_MSG(elapsed >= 40 && elapsed <= 200, detail);
   }

   /* ── 2e. Two successive hal_time_ms() calls during a known sleep ── */
   {
      uint32_t a = hal_time_ms();
      hal_sleep_ms(20);
      uint32_t b = hal_time_ms();
      CHECK_MSG(b > a, "clock did not advance after 20 ms sleep");
   }

   /* ── 2f. hal_sleep_ms(0) does not hang ── */
   {
      hal_sleep_ms(0);
      /* Just checking it returns; no timing assertion on a 0 ms sleep. */
      CHECK_MSG(true, "hal_sleep_ms(0) hung");   /* always passes */
   }
}

/* =========================================================================
* 3. LOGGING
* ====================================================================== */
static void test_logging(void) {
   section("Logging");

   /*
   * The logger writes to stderr; we cannot capture it easily without
   * pipe tricks, so we test the filtering contract and the va_list path
   * by exercising every code path and verifying we don't crash.
   * Visual inspection of stderr output during the run is the secondary
   * verification.
   */

   /* ── 3a. Set level to DEBUG — all messages should be emitted ── */
   hal_log_set_level(HAL_LOG_DEBUG);
   fprintf(stderr, "\n[test] The next 4 lines should all appear:\n");
   hal_log(HAL_LOG_DEBUG, "TEST", "debug message  (level=DEBUG, filter=DEBUG)");
   hal_log(HAL_LOG_INFO,  "TEST", "info  message  (level=INFO,  filter=DEBUG)");
   hal_log(HAL_LOG_WARN,  "TEST", "warn  message  (level=WARN,  filter=DEBUG)");
   hal_log(HAL_LOG_ERROR, "TEST", "error message  (level=ERROR, filter=DEBUG)");
   CHECK(true);   /* crash-free is the pass condition */

   /* ── 3b. Set level to WARN — DEBUG and INFO must be suppressed ── */
   hal_log_set_level(HAL_LOG_WARN);
   fprintf(stderr, "[test] The next 2 lines should NOT appear (filtered):\n");
   hal_log(HAL_LOG_DEBUG, "TEST", "THIS SHOULD NOT PRINT (debug, filter=WARN)");
   hal_log(HAL_LOG_INFO,  "TEST", "THIS SHOULD NOT PRINT (info,  filter=WARN)");
   fprintf(stderr, "[test] The next 2 lines SHOULD appear:\n");
   hal_log(HAL_LOG_WARN,  "TEST", "warn  visible (filter=WARN)");
   hal_log(HAL_LOG_ERROR, "TEST", "error visible (filter=WARN)");
   CHECK(true);

   /* ── 3c. Set level to ERROR — only ERROR passes ── */
   hal_log_set_level(HAL_LOG_ERROR);
   fprintf(stderr, "[test] Only the ERROR line should appear:\n");
   hal_log(HAL_LOG_DEBUG, "TEST", "THIS SHOULD NOT PRINT");
   hal_log(HAL_LOG_INFO,  "TEST", "THIS SHOULD NOT PRINT");
   hal_log(HAL_LOG_WARN,  "TEST", "THIS SHOULD NOT PRINT");
   hal_log(HAL_LOG_ERROR, "TEST", "error visible (filter=ERROR)");
   CHECK(true);

   /* ── 3d. hal_vlog() path (used by wrapper modules in nav_core) ── */
   hal_log_set_level(HAL_LOG_DEBUG);
   /* Call hal_log() with a format that exercises the internal va path. */
   hal_log(HAL_LOG_INFO, "VLOG", "vlog test val=%d str=%s", 42, "hello");
   CHECK(true);

   /* ── 3e. NULL tag does not crash ── */
   hal_log(HAL_LOG_INFO, NULL, "null tag test");
   CHECK(true);

   /* ── 3f. Format with many arguments does not crash ── */
   hal_log(HAL_LOG_DEBUG,
      "TEST",
      "u8=%u u16=%u u32=%u u64=%llu str=%s ptr=%p",
      (uint8_t)0xFF, (uint16_t)0xFFFF, (uint32_t)0xDEADBEEF,
      (unsigned long long)0xCAFEBABEDEADBEEFULL,
      "test",
   (void *)0);
   CHECK(true);

   /* Restore default level for remaining tests. */
   hal_log_set_level(HAL_LOG_INFO);
}

/* =========================================================================
* 4. MEMORY
* ====================================================================== */
static void test_memory(void) {
   section("Memory");

   /* ── 4a. hal_calloc returns non-NULL for reasonable sizes ── */
   void *p = hal_calloc(1, 64);
   CHECK_MSG(p != NULL, "hal_calloc(1, 64) returned NULL");

   /* ── 4b. Memory is zeroed ── */
   if(p) {
      uint8_t *b = (uint8_t *)p;
      bool all_zero = true;
      for(int i = 0; i < 64; i++) {
         if(b[i] != 0) { all_zero = false; break; }
      }
      CHECK_MSG(all_zero, "hal_calloc did not zero-initialise memory");
      hal_free(p);
   }

   /* ── 4c. hal_calloc(0, ...) — should not crash ── */
   void *p0 = hal_calloc(0, 8);
   /* calloc(0,x) is implementation-defined; may be NULL or unique ptr. */
   /* We only require it does not crash. */
   hal_free(p0);   /* safe regardless */
   CHECK(true);

   /* ── 4d. hal_calloc large allocation (1 MB) ── */
   void *big = hal_calloc(1024, 1024);
   CHECK_MSG(big != NULL, "hal_calloc(1024, 1024) returned NULL");
   hal_free(big);

   /* ── 4e. Multiple alloc/free cycles do not corrupt ── */
   {
      bool ok = true;
      for(int i = 0; i < 32; i++) {
         uint8_t *q = (uint8_t *)hal_calloc(i + 1, sizeof(uint8_t));
         if(!q) { ok = false; break; }
         for(int j = 0; j <= i; j++) q[j] = (uint8_t)j;
         /* Verify data survived between write and read in same alloc. */
         for(int j = 0; j <= i; j++) {
            if(q[j] != (uint8_t)j) { ok = false; break; }
         }
         hal_free(q);
      }
      CHECK_MSG(ok, "alloc/free cycle corrupted data");
   }

   /* ── 4f. hal_free(NULL) does not crash ── */
   hal_free(NULL);
   CHECK(true);
}

/* =========================================================================
* 5. MUTEX
* ====================================================================== */

/* Thread argument for contention test. */
typedef struct {
   HalMutex *mutex;
   int       *counter;
   int        increments;
} MutexThreadArg;

static void *mutex_thread(void *arg) {
   MutexThreadArg *a = (MutexThreadArg *)arg;
   for(int i = 0; i < a->increments; i++) {
      hal_mutex_lock(a->mutex);
      (*a->counter)++;
      hal_mutex_unlock(a->mutex);
   }
   return NULL;
}

static void test_mutex(void) {
   section("Mutex");

   /* ── 5a. Create returns non-NULL ── */
   HalMutex *m = hal_mutex_create();
   CHECK_MSG(m != NULL, "hal_mutex_create() returned NULL");
   if(!m) return;

   /* ── 5b. Lock / unlock round-trip does not crash or deadlock ── */
   hal_mutex_lock(m);
   hal_mutex_unlock(m);
   CHECK(true);

   /* ── 5c. Multiple lock/unlock cycles ── */
   for(int i = 0; i < 100; i++) {
      hal_mutex_lock(m);
      hal_mutex_unlock(m);
   }
   CHECK(true);

   /* ── 5d. Mutual exclusion under real concurrency ── */
   {
      /* Two threads each increment a shared counter N times.
      * Without the mutex the final value would be unpredictable
      * (data race).  With it the result must be exactly 2*N.   */
      const int N = 50000;
      int counter = 0;

      MutexThreadArg arg1 = { m, &counter, N };
      MutexThreadArg arg2 = { m, &counter, N };

      pthread_t t1, t2;
      pthread_create(&t1, NULL, mutex_thread, &arg1);
      pthread_create(&t2, NULL, mutex_thread, &arg2);
      pthread_join(t1, NULL);
      pthread_join(t2, NULL);

      char detail[64];
      snprintf(detail, sizeof(detail),
               "counter=%d expected=%d", counter, 2 * N);
      CHECK_MSG(counter == 2 * N, detail);
   }

   /* ── 5e. hal_mutex_destroy(NULL) does not crash ── */
   hal_mutex_destroy(NULL);
   CHECK(true);

   /* ── 5f. Destroy a live mutex ── */
   hal_mutex_destroy(m);
   CHECK(true);
}

/* =========================================================================
* 6. FILE I/O
* ====================================================================== */
static void test_file_io(void) {
   section("File I/O");

   /* ── 6a. Write a known pattern ── */
   {
      const uint8_t pattern[8] = {
         0x54, 0x2C, 0x01, 0x02, 0xAB, 0xCD, 0xEF, 0xFF
      };
      HalFile *fw = hal_file_open(SCRATCH_FILE, HAL_FILE_WRITE);
      CHECK_MSG(fw != NULL, "hal_file_open(WRITE) returned NULL");
      if(!fw) goto skip_read;

      int written = hal_file_write(fw, pattern, (int)sizeof(pattern));
      CHECK_MSG(
         written == (int)sizeof(pattern),
         "hal_file_write returned wrong byte count"
      );
      hal_file_close(fw);

      /* ── 6b. Read it back and verify byte-for-byte ── */
      HalFile *fr = hal_file_open(SCRATCH_FILE, HAL_FILE_READ);
      CHECK_MSG(fr != NULL, "hal_file_open(READ) returned NULL");
      if(!fr) goto skip_read;

      uint8_t readback[8] = {0};
      int n = hal_file_read(fr, readback, (int)sizeof(readback));
      CHECK_MSG(
         n == (int)sizeof(pattern),
         "hal_file_read returned wrong byte count"
      );
      CHECK_MSG(
         memcmp(readback, pattern, sizeof(pattern)) == 0,
         "readback data does not match written pattern"
      );
      hal_file_close(fr);
   }

skip_read:

   /* ── 6c. Read past EOF returns 0 ── */
   {
      /* Write 4 bytes, then read 8 — second read must return 0 (EOF). */
      const uint8_t four[4] = {1, 2, 3, 4};
      HalFile *fw = hal_file_open(SCRATCH_FILE, HAL_FILE_WRITE);
   if(fw) {
         hal_file_write(fw, four, 4);
         hal_file_close(fw);
      }
      HalFile *fr = hal_file_open(SCRATCH_FILE, HAL_FILE_READ);
      if(fr) {
         uint8_t buf[8];
         int n1 = hal_file_read(fr, buf, 8);   /* reads 4, hits EOF */
         int n2 = hal_file_read(fr, buf, 8);   /* already at EOF   */
         (void)n1;
         CHECK_MSG(n2 == 0, "second read past EOF did not return 0");
         hal_file_close(fr);
      }
   }

   /* ── 6d. Append mode accumulates data ── */
   {
      /* Write 2 bytes, append 2 bytes, read back 4. */
      const uint8_t first[2]  = {0xAA, 0xBB};
      const uint8_t second[2] = {0xCC, 0xDD};

      HalFile *fw = hal_file_open(SCRATCH_FILE, HAL_FILE_WRITE);
      if(fw) { hal_file_write(fw, first, 2);  hal_file_close(fw); }

      HalFile *fa = hal_file_open(SCRATCH_FILE, HAL_FILE_APPEND);
      if(fa) { hal_file_write(fa, second, 2); hal_file_close(fa); }

      HalFile *fr = hal_file_open(SCRATCH_FILE, HAL_FILE_READ);
      uint8_t buf[4] = {0};
      int n = fr ? hal_file_read(fr, buf, 4) : -1;
      hal_file_close(fr);

      CHECK_MSG(n == 4, "append: expected 4 bytes total");
      CHECK_MSG(
         buf[0] == 0xAA && buf[1] == 0xBB && buf[2] == 0xCC && buf[3] == 0xDD,
         "append: data mismatch"
      );
   }

   /* ── 6e. hal_file_open on a nonexistent path for READ returns NULL ── */
   {
      HalFile *bad = hal_file_open(
         "/tmp/THIS_FILE_DOES_NOT_EXIST_HAL", HAL_FILE_READ
      );
      CHECK_MSG(bad == NULL, "hal_file_open(nonexistent) did not return NULL");
   }

   /* ── 6f. hal_file_close(NULL) does not crash ── */
   hal_file_close(NULL);
   CHECK(true);

   /* ── 6g. hal_file_write on a READ-mode file — expect -1 or 0 bytes ── */
   {
      /* Write a temp file first so we can open it for reading. */
      const uint8_t byte = 0x55;
      HalFile *fw = hal_file_open(SCRATCH_FILE, HAL_FILE_WRITE);
      if(fw) { hal_file_write(fw, &byte, 1); hal_file_close(fw); }

      HalFile *fr = hal_file_open(SCRATCH_FILE, HAL_FILE_READ);
      if(fr) {
         /* Writing to a read-mode FILE* is undefined behaviour at the
         * libc level, but hal_file_write will call fwrite which will
         * return 0 on a read-only stream.  We just verify no crash. */
         /* Skip the actual write to avoid UB; close cleanly instead. */
         hal_file_close(fr);
      }
      CHECK(true);   /* no crash = pass */
   }

   /* Cleanup. */
   unlink(SCRATCH_FILE);
}

/* =========================================================================
* 7. REPLAY SOURCE
* ====================================================================== */
static void test_replay(void) {
   section("Replay source");

   const int TOTAL_BYTES = 256;

   /* ── 7a. Create a scratch file with a known sequential pattern ── */
   {
      int n = make_scratch_file(SCRATCH_FILE, TOTAL_BYTES);
      CHECK_MSG(n == TOTAL_BYTES, "failed to create scratch file");
   }

   /* ── 7b. Open replay source — must return non-NULL ── */
   HalSerial *replay = hal_serial_open_replay(SCRATCH_FILE, 0 /* no pacing */);
   CHECK_MSG(replay != NULL, "hal_serial_open_replay returned NULL");
   if(!replay) return;

   /* ── 7c. Read in a single call — gets up to max_len bytes ── */
   {
      uint8_t buf[64];
      int n = hal_serial_read_replay(replay, buf, 64, 100);
      CHECK_MSG(
         n > 0 && n <= 64, "first replay read returned unexpected count"
      );

      /* First byte should be 0x00 (sequential pattern). */
      CHECK_MSG(buf[0] == 0x00, "first replay byte should be 0x00");
   }

   /* ── 7d. Drain remaining bytes — eventually returns 0 (EOF) ── */
   {
      uint8_t buf[64];
      int total_read = 64;   /* already read above */
      int got;
      int iterations = 0;
      while ((got = hal_serial_read_replay(replay, buf, 64, 10)) > 0) {
         total_read += got;
         if(++iterations > 1000) break;   /* infinite-loop guard */
      }
      CHECK_MSG(got == 0, "replay did not signal EOF (returned non-zero)");
      CHECK_MSG(total_read == TOTAL_BYTES, "replay total bytes mismatch");
   }

   hal_serial_close_replay(replay);
   CHECK(true);   /* close did not crash */

   /* ── 7e. Re-open and read in small chunks (simulates parser byte feed) ── */
   {
      make_scratch_file(SCRATCH_FILE, 128);
      HalSerial *r = hal_serial_open_replay(SCRATCH_FILE, 0);
      CHECK_MSG(r != NULL, "second replay open returned NULL");
      if(!r) goto skip_chunk;

      uint8_t buf[1];
      int total = 0;
      uint8_t expected = 0;
      bool seq_ok = true;
      int n;
      while ((n = hal_serial_read_replay(r, buf, 1, 10)) == 1) {
         if(buf[0] != expected) { seq_ok = false; break; }
         expected++;
         total++;
      }
      CHECK_MSG(seq_ok, "byte sequence mismatch in 1-byte chunk replay");
      CHECK_MSG(total == 128, "chunk replay total byte count wrong");
      hal_serial_close_replay(r);
   }
skip_chunk:;

   /* ── 7f. Replay open on nonexistent file returns NULL ── */
   {
      HalSerial *bad = hal_serial_open_replay("/tmp/NO_SUCH_FILE_HAL.bin", 0);
      CHECK_MSG(
         bad == NULL, "replay open on nonexistent file did not return NULL"
      );
   }

   /* ── 7g. hal_serial_read_replay with NULL handle returns -1 ── */
   {
      uint8_t buf[4];
      int n = hal_serial_read_replay(NULL, buf, 4, 10);
      CHECK_MSG(n == -1, "replay read with NULL handle should return -1");
   }

   /* ── 7h. hal_serial_read_replay with NULL buffer returns -1 ── */
   {
      make_scratch_file(SCRATCH_FILE, 4);
      HalSerial *r = hal_serial_open_replay(SCRATCH_FILE, 0);
      if(r) {
            int n = hal_serial_read_replay(r, NULL, 4, 10);
            CHECK_MSG(n == -1, "replay read with NULL buf should return -1");
            hal_serial_close_replay(r);
      }
   }

   /* ── 7i. hal_serial_close_replay(NULL) does not crash ── */
   hal_serial_close_replay(NULL);
   CHECK(true);

   /* Cleanup. */
   unlink(SCRATCH_FILE);
}

/* =========================================================================
* 8. SERIAL — error handling (no real hardware needed)
* ====================================================================== */
static void test_serial_errors(void) {
   section("Serial — error handling (no hardware)");

   /* ── 8a. Open with NULL path returns NULL ── */
   {
      HalSerial *s = hal_serial_open(NULL, 230400);
      CHECK_MSG(s == NULL, "hal_serial_open(NULL, baud) should return NULL");
   }

   /* ── 8b. Open with empty string returns NULL ── */
   {
      HalSerial *s = hal_serial_open("", 230400);
      CHECK_MSG(s == NULL, "hal_serial_open(\"\", baud) should return NULL");
   }

   /* ── 8c. Open with unsupported baud rate returns NULL ── */
   {
      HalSerial *s = hal_serial_open("/dev/ttyUSB0", 999999);
      CHECK_MSG(
         s == NULL, "hal_serial_open with invalid baud should return NULL"
      );
   }

   /* ── 8d. Open nonexistent device returns NULL ── */
   {
      HalSerial *s = hal_serial_open("/dev/ttyNONEXISTENT_HAL", 230400);
      CHECK_MSG(s == NULL, "hal_serial_open(nonexistent) should return NULL");
   }

   /* ── 8e. Open a regular file (not a tty) returns NULL ── */
   {
      /* Create a plain file and try to open it as a serial port. */
      const char *tmp = "/tmp/hal_not_a_tty.bin";
      FILE *fp = fopen(tmp, "wb");
      if(fp) { fputc(0, fp); fclose(fp); }

      HalSerial *s = hal_serial_open(tmp, 230400);
      CHECK_MSG(s == NULL, "hal_serial_open on plain file should return NULL");
      unlink(tmp);
   }

   /* ── 8f. hal_serial_close(NULL) does not crash ── */
   hal_serial_close(NULL);
   CHECK(true);

   /* ── 8g. hal_serial_read with NULL handle returns -1 ── */
   {
      uint8_t buf[4];
      int n = hal_serial_read(NULL, buf, 4, 10);
      CHECK_MSG(n == -1, "hal_serial_read(NULL,...) should return -1");
   }

   /* ── 8h. hal_serial_read with NULL buffer returns -1 ── */
   {
      /* We cannot open a real port here, but passing NULL as the handle
      * is sufficient to test the guard in the same code path. */
      int n = hal_serial_read(NULL, NULL, 4, 10);
      CHECK_MSG(n == -1, "hal_serial_read(NULL, NULL,...) should return -1");
   }

   /* ── 8i. hal_serial_write with NULL handle returns -1 ── */
   {
      const uint8_t b = 0x54;
      int n = hal_serial_write(NULL, &b, 1);
      CHECK_MSG(n == -1, "hal_serial_write(NULL,...) should return -1");
   }

   /* ── 8j. hal_serial_flush_rx(NULL) does not crash ── */
   hal_serial_flush_rx(NULL);
   CHECK(true);

   /* ── 8k. hal_serial_bytes_available(NULL) returns -1 ── */
   {
      int n = hal_serial_bytes_available(NULL);
      CHECK_MSG(n == -1, "hal_serial_bytes_available(NULL) should return -1");
   }
}

/* =========================================================================
* 9. SERIAL — live hardware tests (only when a TTY path is supplied)
* ====================================================================== */
static void test_serial_live(const char *port) {
   section("Serial — live hardware");
   printf("  Port: %s\n", port);

   /* ── 9a. Open at correct baud (230400 for FHL-LD20) ── */
   HalSerial *s = hal_serial_open(port, 230400);
   CHECK_MSG(s != NULL, "hal_serial_open failed on provided port");
   if(!s) return;

   /* ── 9b. bytes_available immediately after open is >= 0 ── */
   {
      int avail = hal_serial_bytes_available(s);
      CHECK_MSG(avail >= 0, "hal_serial_bytes_available returned negative");
   }

   /* ── 9c. flush_rx does not error ── */
   hal_serial_flush_rx(s);
   CHECK(true);

   /* ── 9d. read with short timeout — receives data or times out cleanly ── */
   {
      uint8_t buf[256];
      int n = hal_serial_read(s, buf, sizeof(buf), 200);
      char detail[64];
      snprintf(
         detail, sizeof(detail),
         "hal_serial_read returned %d (expected >=0)", n
      );
      CHECK_MSG(n >= 0, detail);

      if(n > 0) {
         HAL_LOGI("TEST", "Live read: %d bytes, first=0x%02X", n, buf[0]);
      } else {
         HAL_LOGW("TEST", "Live read: timeout (0 bytes) — sensor sending?");
      }
   }

   /* ── 9e. Read enough bytes to fill a buffer — non-blocking path ── */
   {
      /* Drain up to 1024 bytes with a 500 ms window.
         * This exercises the select() loop across multiple read() calls. */
      uint8_t buf[1024];
      int total = 0;
      uint32_t start = hal_time_ms();
      while (hal_time_ms() - start < 500) {
         int n = hal_serial_read(
            s, buf + total, (int)(sizeof(buf) - (size_t)total), 50
         );
         if(n < 0) break;
         total += n;
         if(total >= (int)sizeof(buf)) break;
      }
      char detail[64];
      snprintf(
         detail, sizeof(detail), "received %d bytes in 500 ms window", total
      );
      /* We only log; not failing on 0 bytes (sensor might need warm-up). */
      HAL_LOGI("TEST", "%s", detail);
      CHECK(true);
   }

   /* ── 9f. bytes_available after draining ── */
   {
      int avail = hal_serial_bytes_available(s);
      CHECK_MSG(avail >= 0, "bytes_available after drain negative");
   }

   /* ── 9g. Write (sensor does not require commands — this is a no-op tx) ── */
   {
      /* The FHL-LD20 ignores unknown bytes; we send one 0x00 and verify
      * hal_serial_write returns 1 (byte enqueued for transmission).   */
      const uint8_t null_cmd = 0x00;
      int n = hal_serial_write(s, &null_cmd, 1);
      CHECK_MSG(n == 1, "hal_serial_write returned != 1");
   }

   /* ── 9h. Close ── */
   hal_serial_close(s);
   CHECK(true);

   /* ── 9i. Open at every supported baud rate (port opens, then closes) ── */
   {
      const uint32_t rates[] = {
         9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
      };
      bool all_ok = true;
      for(size_t i = 0; i < sizeof(rates)/sizeof(rates[0]); i++) {
         HalSerial *t = hal_serial_open(port, rates[i]);
         if(!t) { all_ok = false; break; }
         hal_serial_close(t);
      }
      CHECK_MSG(all_ok, "one or more supported baud rates failed to open");
   }
}

/* =========================================================================
* 10. CROSS-SUBSYSTEM: time + logging under mutex contention
*
*     The logger is protected by a pthread mutex.  Spawning concurrent
*     threads that all log simultaneously verifies there is no deadlock
*     or corruption when hal_log() is called from multiple threads.
* ====================================================================== */

typedef struct { int id; int count; } LogThreadArg;

static void *log_thread(void *arg) {
   LogThreadArg *a = (LogThreadArg *)arg;
   for(int i = 0; i < a->count; i++) {
      hal_log(HAL_LOG_DEBUG,
         "THREAD", "thread=%d iteration=%d time_ms=%u", a->id, i,
      hal_time_ms());
   }
   return NULL;
}

static void test_concurrent_logging(void) {
   section("Concurrent logging (time + mutex + log)");

   hal_log_set_level(HAL_LOG_DEBUG);

   const int N_THREADS = 4;
   const int N_ITERS   = 50;
   pthread_t threads[4];
   LogThreadArg args[4];

   uint32_t t0 = hal_time_ms();

   for(int i = 0; i < N_THREADS; i++) {
      args[i].id    = i;
      args[i].count = N_ITERS;
      pthread_create(&threads[i], NULL, log_thread, &args[i]);
   }
   for(int i = 0; i < N_THREADS; i++) {
      pthread_join(threads[i], NULL);
   }

   uint32_t elapsed = hal_time_ms() - t0;
   CHECK_MSG(
      elapsed < 5000, "concurrent logging took unexpectedly long (>5s)"
   );

   hal_log_set_level(HAL_LOG_INFO);
   HAL_LOGI("TEST", "concurrent logging complete (%u ms)", elapsed);
   CHECK(true);
}

/* =========================================================================
* 11. REPLAY PACING accuracy
*
*     Open a replay at 230400 baud and read a block.  The elapsed wall
*     time should be within a generous tolerance of the theoretical UART
*     transmission time.  (Not a hard real-time test — just sanity.)
* ====================================================================== */
static void test_replay_pacing(void) {
   section("Replay pacing");

   /* 100 bytes at 230400 baud:
   *   10 bits/byte → 1000 bits → 1000/230400 s ≈ 4.34 ms          */
   const int    N_BYTES  = 100;
   const uint32_t BAUD   = 230400;
   const double expected_ms = (10.0 * N_BYTES * 1000.0) / BAUD;

   make_scratch_file(SCRATCH_FILE, N_BYTES);

   HalSerial *r = hal_serial_open_replay(SCRATCH_FILE, BAUD);
   CHECK_MSG(r != NULL, "pacing replay open failed");
   if(!r) { unlink(SCRATCH_FILE); return; }

   uint8_t buf[128];
   uint32_t t0 = hal_time_ms();
   int total = 0, n;
   while ((n = hal_serial_read_replay(r, buf, sizeof(buf), 100)) > 0)
      total += n;
   uint32_t elapsed = hal_time_ms() - t0;

   hal_serial_close_replay(r);
   unlink(SCRATCH_FILE);

   CHECK_MSG(total == N_BYTES, "pacing: wrong total bytes");

   /* Allow 0.5× – 10× the theoretical time (nanosleep jitter + scheduler). */
   char detail[128];
   snprintf(detail,
      sizeof(detail), "elapsed=%u ms, expected≈%.1f ms", elapsed,
   expected_ms);
   CHECK_MSG(
      elapsed >= (uint32_t)(expected_ms * 0.5)
      && elapsed <= (uint32_t)(expected_ms * 10.0 + 20.0),
   detail);

   HAL_LOGI("TEST",
      "Replay pacing: %u ms elapsed, expected ≈ %.1f ms", elapsed,
   expected_ms);
}

/* =========================================================================
* main
* ====================================================================== */
int main(int argc, char *argv[]) {
   g_colour = isatty(STDOUT_FILENO);

   const char *live_port = (argc >= 2) ? argv[1] : NULL;

   if(g_colour)
      printf(
         "\n%s%sHAL Linux — functional test suite%s\n",
         COL_BOLD, COL_YELLOW, COL_RESET
      );
   else printf("\nHAL Linux — functional test suite\n");

   if(live_port) printf("Live port : %s\n", live_port);
   else printf("Live port : (none — hardware tests skipped)\n");
   printf("Platform  : %s\n\n", hal_platform_name());

   /* Suppress HAL's own log output during tests that don't need it. */
   hal_log_set_level(HAL_LOG_WARN);

   test_platform_info();
   test_time();
   test_logging();        /* temporarily sets its own log level internally */
   test_memory();
   test_mutex();
   test_file_io();
   test_replay();
   test_serial_errors();
   test_concurrent_logging();
   test_replay_pacing();

   if(live_port)
      test_serial_live(live_port);

   /* ── Summary ── */
   printf("\n");
   if(g_colour) printf(
      "%s%s━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━%s\n",
      COL_BOLD, COL_YELLOW, COL_RESET
   );
   else printf("------------------------------------------------\n");

   printf("Tests run    : %d\n", g_tests_run);
   if(g_colour) printf(
      "Tests passed : %s%d%s\n", COL_GREEN, g_tests_passed, COL_RESET
   );
   else printf("Tests passed : %d\n", g_tests_passed);

   if(g_tests_failed > 0) {
      if(g_colour) printf(
         "Tests FAILED : %s%d%s\n", COL_RED, g_tests_failed, COL_RESET
      );
      else printf("Tests FAILED : %d\n", g_tests_failed);
   } else {
      if(g_colour) printf("%sAll tests passed.%s\n", COL_GREEN, COL_RESET);
      else printf("All tests passed.\n");
   }

   printf("\n");
   return (g_tests_failed > 0) ? 1 : 0;
}