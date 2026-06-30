#!/bin/sh

options="--test-hal --test-parser --test-grid --test-astar --test-main --main"

case "$1" in
  "--test-hal")
    ./build/test_hal_linux /dev/ttyUSB0
    ;;
  "--test-parser")
    ./build/test_parser scan_raw.bin
    ;;
  "--test-grid")
    ./build/test_grid scan_raw.bin
    cp /tmp/{test_grid,test_null,replay_grid}.csv tests/
    ;;
  "--test-astar")
    ./build/test_astar scan_raw.bin
    ;;
  "--test-main")
    ./build/aegis_nav_mod -r scan_raw.bin /dev/ttyUSB0
    ;;
  "--main")
    ./build/aegis_nav_mod /dev/ttyUSB0
    ;;
  *)
    echo "unknown option. try one of the following:"
    for i in $options; do
      echo "   $i"
    done
    exit 1
    ;;
esac
