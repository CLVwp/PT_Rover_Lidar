#!/bin/bash
# Lance SLAM Toolbox en écoutant /lidar_scan (et base_link).
# Sans ça, le SLAM écoute /scan par défaut et ne reçoit rien → /map reste vide.
# Usage: ./run_slam.sh
# Prérequis : bridge + estimateur actifs (./run_bridge.sh, ./run_estimator.sh lidar ou lio).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PARAMS="$SCRIPT_DIR/config/slam_toolbox_rover.yaml"

if [ ! -f "$PARAMS" ]; then
  echo "Fichier de config introuvable: $PARAMS"
  exit 1
fi

if [ -d /opt/ros/humble ]; then
  source /opt/ros/humble/setup.bash
elif [ -d /opt/ros/jazzy ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -d /opt/ros/iron ]; then
  source /opt/ros/iron/setup.bash
else
  echo "Aucune distro ROS2 trouvée. Source manuelle : source /opt/ros/<distro>/setup.bash"
  exit 1
fi

echo "Lancement SLAM Toolbox (scan_topic=/lidar_scan, base_frame=base_link)..."
exec ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="$PARAMS"
