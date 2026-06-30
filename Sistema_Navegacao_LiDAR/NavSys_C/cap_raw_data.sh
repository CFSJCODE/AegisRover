#!/bin/sh
# capture_lidar.sh
# Usage: ./capture_lidar.sh [duration_seconds] [port] [outfile]

BAUD="${1:-230400}"
DURATION="${2:-10}"
PORT="${3:-/dev/ttyUSB0}"
OUTFILE="${4:-scan_raw.bin}"

# ── 1. Configure port ────────────────────────────────────────────────────────
echo "Configuring $PORT at $BAUD baud (8N1, raw)..."
stty -F "$PORT" "$BAUD" raw \
    -echo -echoe -echok \
    -icrnl -onlcr \
    cs8 -cstopb -parenb \
    clocal cread
echo ""

# ── 2. Capture ───────────────────────────────────────────────────────────────
# timeout controls the read; cat streams bytes as they arrive;
# dd writes them to disk in 4096-byte chunks whenever the pipe flushes.
# No iflag=fullblock — we want every byte written as soon as it arrives.
echo "Capturing ${DURATION}s of data -> $OUTFILE ..."
timeout -s INT "${DURATION}s" dd if="$PORT" of="$OUTFILE" bs=4096 status=progress
echo ""

# ── 3. Report ────────────────────────────────────────────────────────────────
BYTES=$(wc -c < "$OUTFILE")
EXPECTED=$(( BYTES / 41 ))
echo "Captured : $BYTES bytes  (~$EXPECTED packets)"
echo ""

echo "First 80 bytes:"
xxd "$OUTFILE" | head -5
echo ""

echo "CRC validation:"
python3 tools/verify_capture.py "$(pwd)/$OUTFILE"
