#!/usr/bin/env bash
# Note: on n'active pas `-u` (nounset) car ROS2 `setup.bash` peut référencer
# des variables non initialisées sous certaines images.
set -eo pipefail

cd /workspace

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

echo "[slam] Attente ${SLAM_START_DELAY_S:-3}s avant démarrage SLAM (pour /lidar_scan et /odom/TF)..."
sleep "${SLAM_START_DELAY_S:-3}"

echo "[slam] Lancement slam_toolbox (scan_topic=/lidar_scan, base_frame=base_link)..."
bash /workspace/ros2_bridge/run_slam.sh

