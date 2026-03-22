#!/bin/bash
# Lance le gestionnaire navigation (waypoints + patrol + naive) pilotable via /nav_control.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

TARGET_FRAME="${GOTO_TARGET_FRAME:-map}"
exec python3 nav_manager_node.py --ros-args -p target_frame:="${TARGET_FRAME}"

