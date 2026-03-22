#!/bin/bash
# Lance l'estimateur de trajectoire (LiDAR seul ou LIO).
# Usage: ./run_estimator.sh [lidar|lio]
#   lidar = odométrie LiDAR seule (ICP 2D)
#   lio   = LiDAR + IMU (position LiDAR, orientation IMU)
# Prérequis : bridge actif (./run_bridge.sh dans un autre terminal).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="${1:-lidar}"
if [ "$MODE" != "lidar" ] && [ "$MODE" != "lio" ]; then
  echo "Usage: $0 [lidar|lio]"
  echo "  lidar  = estimateur LiDAR seul (TF + /odom)"
  echo "  lio    = LIO LiDAR+IMU (TF + /odom)"
  exit 1
fi

if [ -d /opt/ros/humble ]; then
  source /opt/ros/humble/setup.bash
elif [ -d /opt/ros/jazzy ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -d /opt/ros/iron ]; then
  source /opt/ros/iron/setup.bash
else
  echo "Source ROS2 manuellement : source /opt/ros/<distro>/setup.bash"
  exit 1
fi

if ! python3 -c "import scipy.spatial" 2>/dev/null; then
  if [ -d .venv ]; then
    .venv/bin/pip install --quiet scipy
  else
    pip install --user scipy 2>/dev/null || pip install scipy
  fi
fi

if [ "$MODE" = "lidar" ]; then
  echo "Lancement estimateur LiDAR seul (TF + /odom)..."
  exec python3 lidar_odom_node.py
else
  echo "Lancement LIO (LiDAR+IMU, TF + /odom)..."
  exec python3 lio_odom_node.py
fi
