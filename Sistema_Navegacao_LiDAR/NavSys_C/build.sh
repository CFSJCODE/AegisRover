#!/bin/sh

options="--test-hal --test-parser --test-grid --test-astar --main"

mkdir -p build

case "$1" in
  "--test-hal")
    gcc -std=c11 -Wall -Wextra -O0 -g -DHAL_PLATFORM=HAL_PLATFORM_LINUX \
      -Ihal \
      tests/test_hal_linux.c \
      hal/hal_linux.c \
      -lpthread -o build/test_hal_linux
    ;;
  "--test-parser")
    gcc -std=c11 -Wall -Wextra -O0 -g -DHAL_PLATFORM=HAL_PLATFORM_LINUX \
      -Ihal -Inav_core \
      tests/test_parser.c \
      hal/hal_linux.c nav_core/lidar_parser.c \
      -o build/test_parser
    ;;
  "--test-grid")
    gcc -std=c11 -Wall -Wextra -O0 -g \
      -Ihal -Inav_core \
      tests/test_grid.c \
      hal/hal_linux.c nav_core/lidar_parser.c nav_core/scan_filter.c nav_core/occupancy_grid.c nav_core/grid_viz.c \
      -o build/test_grid
    ;;
  "--test-astar")
    gcc -std=c11 -Wall -Wextra -O0 -g \
      -Ihal -Inav_core \
      tests/test_astar.c \
      hal/hal_linux.c \
      nav_core/lidar_parser.c nav_core/scan_filter.c \
      nav_core/occupancy_grid.c nav_core/grid_viz.c \
      nav_core/astar.c \
      -o build/test_astar
    ;;
  "--main")
    gcc -std=c11 -Wall -Wextra -O0 -g \
      -Ihal -Inav_core \
      app/linux/main.c \
      hal/hal_linux.c \
      nav_core/lidar_parser.c nav_core/scan_filter.c \
      nav_core/occupancy_grid.c nav_core/grid_viz.c \
      nav_core/astar.c \
      -lpthread -o build/aegis_nav_mod
    ;;
  *)
    echo "unknown option. try one of the following:"
    for i in $options; do
      echo "   $i"
    done
    exit 1
    ;;
esac
