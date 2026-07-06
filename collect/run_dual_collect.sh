#!/usr/bin/env sh

# Edit parameters here, then run:
#   sh collect/run_dual_collect.sh
#
# Runtime keys:
#   r: activate teleoperation
#   s: deactivate teleoperation
#   c: start recording one trajectory
#   v: stop current trajectory recording
#   q: quit

PYTHON_BIN="python3"

# Robot serial numbers
FIRST_SN="Rizon4s-063652"
SECOND_SN="Rizon4s-063586"

# Data collection
SAVE_ROOT="/home/xense/MUSE_workspace/FlexivXense_Collect/data"
SESSION_NAME=""
FPS="30"

# Gripper collection: true or false.
# Master side uses Angler encoder, slave side uses Xense.
USE_GRIPPER="true"
SLAVE_GRIPPER_ID="1659f0e0dde0"

# Master Angler encoder settings.
ANGLER_ID="/dev/ttyUSB0"
ANGLER_INDEX="1"
ANGLER_BAUDRATE="1000000"
ANGLER_GAP="-1"
ANGLER_STRICT="true"
ANGLER_OPEN_ANGLE="51.68"
ANGLER_CLOSE_ANGLE="16.61"
SLAVE_OPEN_WIDTH="0.08"
SLAVE_CLOSE_WIDTH="0.0"

# Xense tactile sensors (end-effector cameras). Leave empty to disable.
# Set each to the device ID string reported by the Xense SDK.
XENSE_LEFT="OG001452"
XENSE_LEFT_MAC="1659f0e0dde0"
XENSE_RIGHT="OG001454"
XENSE_RIGHT_MAC="1659f0e0dde0"
XENSE_FPS="50"

# Optional LAN interface whitelist. Leave empty to let TDK try all interfaces.
# Multiple addresses can be separated by spaces, for example:
# NETWORK_INTERFACES="192.168.2.102 10.42.0.1"
NETWORK_INTERFACES=""

# Runtime tuning
GRIPPER_EPS="0.0001"
GRIPPER_WAIT_TIME="0.1"
NULL_SPACE_PERIOD="0.1"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

set -- \
  "$SCRIPT_DIR/dual_collect.py" \
  -1 "$FIRST_SN" \
  -2 "$SECOND_SN" \
  --save-root "$SAVE_ROOT" \
  --fps "$FPS" \
  --use-gripper "$USE_GRIPPER" \
  --gripper-eps "$GRIPPER_EPS" \
  --gripper-wait-time "$GRIPPER_WAIT_TIME" \
  --null-space-period "$NULL_SPACE_PERIOD"

if [ -n "$SESSION_NAME" ]; then
  set -- "$@" --session-name "$SESSION_NAME"
fi

if [ "$USE_GRIPPER" = "true" ]; then
  set -- "$@" \
    --slave-gripper-id "$SLAVE_GRIPPER_ID" \
    --angler-id "$ANGLER_ID" \
    --angler-index "$ANGLER_INDEX" \
    --angler-baudrate "$ANGLER_BAUDRATE" \
    --angler-gap="${ANGLER_GAP}" \
    --angler-strict "$ANGLER_STRICT" \
    --angler-open-angle "$ANGLER_OPEN_ANGLE" \
    --angler-close-angle "$ANGLER_CLOSE_ANGLE" \
    --slave-open-width "$SLAVE_OPEN_WIDTH" \
    --slave-close-width "$SLAVE_CLOSE_WIDTH"
fi

for interface in $NETWORK_INTERFACES; do
  set -- "$@" --network-interface "$interface"
done

if [ -n "$XENSE_LEFT" ]; then
  set -- "$@" --xense-left-id "$XENSE_LEFT"
fi
if [ -n "$XENSE_LEFT_MAC" ]; then
  set -- "$@" --xense-left-mac "$XENSE_LEFT_MAC"
fi
if [ -n "$XENSE_RIGHT" ]; then
  set -- "$@" --xense-right-id "$XENSE_RIGHT"
fi
if [ -n "$XENSE_RIGHT_MAC" ]; then
  set -- "$@" --xense-right-mac "$XENSE_RIGHT_MAC"
fi
if [ -n "$XENSE_LEFT" ] || [ -n "$XENSE_RIGHT" ]; then
  set -- "$@" --xense-fps "$XENSE_FPS"
fi

echo "Running:"
printf '  %s' "$PYTHON_BIN"
for arg in "$@"; do
  printf ' %s' "$arg"
done
printf '\n\n'

exec "$PYTHON_BIN" "$@"
