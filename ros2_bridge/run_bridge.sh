#!/bin/bash
# Lance le bridge Rover -> ROS2 (LiDAR WebSocket + IMU HTTP).
# Prérequis : ROS2 installé. Rover doit être joignable sur le réseau.

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

echo "Lancement rover_bridge (LiDAR WS + IMU HTTP)..."
exec python3 rover_bridge.py

