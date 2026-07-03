#!/usr/bin/env sh

PYTHON_BIN="python3"
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

echo "=== Homing master robot (id=1) ==="
"$PYTHON_BIN" "$SCRIPT_DIR/homing.py" --id 1 || exit 1

echo "=== Homing slave robot (id=2) ==="
"$PYTHON_BIN" "$SCRIPT_DIR/homing.py" --id 2 || exit 1

echo "=== Starting data collection ==="
sh "$SCRIPT_DIR/run_dual_collect.sh"
