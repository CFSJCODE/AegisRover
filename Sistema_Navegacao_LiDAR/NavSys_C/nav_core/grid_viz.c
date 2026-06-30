/**
 * @file grid_viz.c
 * @brief ASCII + ANSI occupancy grid visualiser — Linux only.
 *
 * Entire translation unit is compiled only when EMBEDDED is not defined.
 * On ESP32 builds this file is simply not added to the build.
**/

#if !defined(EMBEDDED)

#define _GNU_SOURCE   /* isatty on older glibc */

#include "grid_viz.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>   /* isatty, STDOUT_FILENO */

/* =========================================================================
* ANSI escape codes
* ====================================================================== */

#define ANSI_RESET     "\033[0m"
#define ANSI_BOLD      "\033[1m"
#define ANSI_DIM       "\033[2m"

/* Foreground colours */
#define ANSI_FG_RED    "\033[31m"
#define ANSI_FG_GREEN  "\033[32m"
#define ANSI_FG_YELLOW "\033[33m"
#define ANSI_FG_CYAN   "\033[36m"
#define ANSI_FG_WHITE  "\033[37m"

/* Bright variants */
#define ANSI_FG_BRED   "\033[91m"
#define ANSI_FG_BGRN   "\033[92m"
#define ANSI_FG_BYEL   "\033[93m"
#define ANSI_FG_BCYN   "\033[96m"

#define ANSI_CLEAR     "\033[2J\033[H"

// // testing
// #define ROBOT_CX 0
// #define ROBOT_CY 0

/* =========================================================================
* Default configuration
* ====================================================================== */

void viz_cfg_default(VizCfg *cfg) {
   if(!cfg) return;
   cfg->use_colour    = isatty(STDOUT_FILENO) ? true : false;
   cfg->show_inflated = true;
   cfg->show_stats    = true;
   cfg->clear_screen  = true;
   cfg->downsample    = 1;
}

/* =========================================================================
* Internal: check if a cell coordinate is on the path
* ====================================================================== */

static bool on_path(const VizPath *path, int16_t cx, int16_t cy) {
   if(!path || !path->cells) return false;
   for(uint16_t i = 0; i < path->count; i++)
      if(path->cells[i].cx == cx && path->cells[i].cy == cy) return true;
   return false;
}

/* =========================================================================
* Core render
* ====================================================================== */

void viz_render(
   const OccupancyGrid *g, const VizPath *path, const VizCfg *cfg
) {
   if(!g) return;

   VizCfg local;
   if(!cfg) { viz_cfg_default(&local); cfg = &local; }

   uint8_t ds = cfg->downsample;
   if(ds < 1) ds = 1;

   /* Compute display dimensions. */
   int cols = GRID_W / ds;
   int rows = GRID_H / ds;
   if(cols > VIZ_MAX_COLS) { ds = (uint8_t)(GRID_W / VIZ_MAX_COLS + 1); cols = GRID_W / ds; }
   if(rows > VIZ_MAX_ROWS) { /* increase ds for rows too */
      uint8_t ds2 = (uint8_t)(GRID_H / VIZ_MAX_ROWS + 1);
      if(ds2 > ds) { ds = ds2; cols = GRID_W / ds; rows = GRID_H / ds; }
   }

   if(cfg->clear_screen && cfg->use_colour)
      fputs(ANSI_CLEAR, stdout);

   /* Top border. */
   if(cfg->use_colour) fputs(ANSI_DIM, stdout);
   putchar('+');
   for(int c = 0; c < cols; c++) putchar('-');
   putchar('+');
   putchar('\n');
   if(cfg->use_colour) fputs(ANSI_RESET, stdout);

   for(int row = 0; row < rows; row++) {
      int16_t cy = (int16_t)(row * ds + ds / 2);   /* sample centre */

      if(cfg->use_colour) fputs(ANSI_DIM, stdout);
      putchar('|');
      if(cfg->use_colour) fputs(ANSI_RESET, stdout);

      for(int col = 0; col < cols; col++) {
         int16_t cx = (int16_t)(col * ds + ds / 2);

         /* Robot position? */
         bool is_robot = (cx == ROBOT_CX && cy == ROBOT_CY);
         bool is_path  = on_path(path, cx, cy);

         const uint8_t *arr = cfg->show_inflated ? g->inflated : g->cells;
         uint8_t val = (cx >= 0 && cx < GRID_W && cy >= 0 && cy < GRID_H)
                     ? arr[(uint32_t)cy * GRID_W + (uint32_t)cx]
                     : CELL_UNKNOWN;

         if(is_robot) {
               if(cfg->use_colour) fputs(ANSI_FG_BCYN ANSI_BOLD, stdout);
               putchar('R');
         } else if(is_path) {
               if(cfg->use_colour) fputs(ANSI_FG_BGRN ANSI_BOLD, stdout);
               putchar('*');
         } else if(val == CELL_OCCUPIED) {
               if(cfg->use_colour) fputs(ANSI_FG_BRED ANSI_BOLD, stdout);
               putchar('#');
         } else if(val == CELL_INFLATED) {
               if(cfg->use_colour) fputs(ANSI_FG_BYEL, stdout);
               putchar('+');
         } else if(val == CELL_FREE) {
               if(cfg->use_colour) fputs(ANSI_DIM ANSI_FG_WHITE, stdout);
               putchar('.');
         } else {
               /* CELL_UNKNOWN */
               putchar(' ');
         }

         if(cfg->use_colour) fputs(ANSI_RESET, stdout);
      }

      if(cfg->use_colour) fputs(ANSI_DIM, stdout);
      putchar('|');
      if(cfg->use_colour) fputs(ANSI_RESET, stdout);
      putchar('\n');
   }

   /* Bottom border. */
   if(cfg->use_colour) fputs(ANSI_DIM, stdout);
   putchar('+');
   for(int c = 0; c < cols; c++) putchar('-');
   putchar('+');
   putchar('\n');
   if(cfg->use_colour) fputs(ANSI_RESET, stdout);

   if(cfg->show_stats)
      viz_print_stats(g, 0);

   fflush(stdout);
}

/* =========================================================================
* Statistics summary
* ====================================================================== */

void viz_print_stats(const OccupancyGrid *g, uint32_t scan_no) {
   if(!g) return;

   uint32_t total = (uint32_t)GRID_W * GRID_H;
   uint32_t n_free = 0, n_occ = 0, n_inf = 0, n_unk = 0;

   for(uint32_t i = 0; i < total; i++) {
      switch (g->inflated[i]) {
      case CELL_FREE:     n_free++; break;
      case CELL_OCCUPIED: n_occ++;  break;
      case CELL_INFLATED: n_inf++;  break;
      default:            n_unk++;  break;
      }
   }

   printf("scan#%u  grid=%dx%d  cell=%dmm  "
         "free=%u(%.0f%%)  occ=%u  infl=%u  unk=%u  "
         "rays=%u  clipped=%u\n",
         scan_no, GRID_W, GRID_H, GRID_CELL_MM,
         n_free,  (double)n_free / total * 100.0,
         n_occ, n_inf, n_unk,
         g->rays_cast, g->cells_clipped);
}

/* =========================================================================
* CSV dump
* ====================================================================== */

bool viz_dump_csv(const OccupancyGrid *g, const char *path) {
   if(!g || !path) return false;

   FILE *fp = fopen(path, "w");
   if(!fp) return false;

   for(int cy = 0; cy < GRID_H; cy++) {
      for(int cx = 0; cx < GRID_W; cx++) {
         fprintf(fp, "%u", g->cells[(uint32_t)cy * GRID_W + (uint32_t)cx]);
         if(cx < GRID_W - 1) fputc(',', fp);
      }
      fputc('\n', fp);
   }

   fclose(fp);
   return true;
}

#endif /* !EMBEDDED */