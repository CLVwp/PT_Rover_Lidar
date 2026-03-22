#!/bin/bash
# Navigation autonome : lit /lidar_scan et envoie les commandes de direction au rover.
# Prérequis : bridge actif (./run_bridge.sh). Arrêt avec Ctrl+C envoie stop au rover.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

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

echo "Lancement navigation autonome (LiDAR -> commandes rover)..."
exec python3 autonomous_nav_node.py
