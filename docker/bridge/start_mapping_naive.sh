#!/usr/bin/env bash
# Note: on n'active pas `-u` (nounset) car ROS2 `setup.bash` peut référencer
# des variables non initialisées sous certaines images.
set -eo pipefail

cd /workspace

# Démarrage en "mapping naif" :
# - Foxglove bridge (Foxglove -> ROS topics)
# - rover_bridge (publie /lidar_scan, /imu/* et écoute /cmd_vel)
# - estimateur (pub /odom + TF odom->base_link)
# - (optionnel) autonavigation (si START_AUTONOMOUS_NAV=1)

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

echo "[bridge] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ROVER_IP=${ROS_ROVER_IP:-unset}  START_AUTONOMOUS_NAV=${START_AUTONOMOUS_NAV:-0}"

# 1) Foxglove bridge (WebSocket)
# Bind explicite sur 0.0.0.0 pour être joignable depuis l’extérieur du conteneur.
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 &
FOXGLOVE_PID=$!

# 2) Bridge Rover -> ROS2 (LiDAR WS + IMU HTTP)
bash /workspace/ros2_bridge/run_bridge.sh &
ROVER_BRIDGE_PID=$!

# 3) Estimateur trajet (LiDAR ou LIO)
ESTIMATOR_MODE="${ESTIMATOR_MODE:-lidar}"
echo "[bridge] Estimateur: ${ESTIMATOR_MODE}"
bash /workspace/ros2_bridge/run_estimator.sh "${ESTIMATOR_MODE}" &
ESTIMATOR_PID=$!

# 3b) Manager navigation (waypoints/patrol/naive)
if [ "${START_NAV_MANAGER:-1}" = "1" ]; then
  echo "[bridge] Start nav manager (frame cible: ${GOTO_TARGET_FRAME:-map})"
  bash /workspace/ros2_bridge/run_nav_manager.sh &
  NAV_MGR_PID=$!
fi

# 3c) Monitoring IMU (angles + accel + resume stabilité)
if [ "${START_IMU_MONITOR:-1}" = "1" ]; then
  echo "[bridge] Start IMU monitor"
  bash /workspace/ros2_bridge/run_imu_monitor.sh &
  IMU_MON_PID=$!
fi

# 4) Autonome (optionnel). Sinon, tu pilotes depuis Foxglove via /cmd_vel.
if [ "${START_AUTONOMOUS_NAV:-0}" = "1" ]; then
  bash /workspace/ros2_bridge/run_autonomous_nav.sh &
  AUTO_PID=$!
fi

cleanup() {
  echo "[bridge] Arrêt demandé, nettoyage..."
  kill "${FOXGLOVE_PID}" 2>/dev/null || true
  kill "${ROVER_BRIDGE_PID}" 2>/dev/null || true
  kill "${ESTIMATOR_PID}" 2>/dev/null || true
  if [ "${NAV_MGR_PID:-}" != "" ]; then
    kill "${NAV_MGR_PID}" 2>/dev/null || true
  fi
  if [ "${IMU_MON_PID:-}" != "" ]; then
    kill "${IMU_MON_PID}" 2>/dev/null || true
  fi
  if [ "${AUTO_PID:-}" != "" ]; then
    kill "${AUTO_PID}" 2>/dev/null || true
  fi
  exit 0
}

trap cleanup SIGINT SIGTERM

echo "[bridge] Prêt. Foxglove: ws://localhost:8765  Topics: /lidar_scan /imu/rpy_deg /odom (après estimateur)"

# Reste vivant pour garder les sous-process actifs
while true; do
  sleep 2
done

