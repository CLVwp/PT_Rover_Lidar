## Docker (ROS2 Jazzy) : bridge + Foxglove + slam_toolbox + web_ui

Objectif :
- Conteneur `rover_bridge` : publie `/lidar_scan`, `/imu/*`, `/odom` (après estimateur) et permet à Foxglove d’émettre `/cmd_vel`.
- Conteneur `slam_toolbox` : publie `/map` (carte) en écoutant `/lidar_scan`.
- Conteneur `web_ui` : héberge une page locale de télémétrie + contrôle rover.

### Pré-requis
- Docker Desktop activé (containers Linux via WSL2).
- Accès réseau du conteneur au rover (IP `ROS_ROVER_IP`).

### Démarrage
1. Mapping naïf (bridge + estimateur + Foxglove bridge) :
   - Option A (pilotage depuis Foxglove) :
     - `docker compose up rover_bridge`
   - Option B (autonavigation “naïve” immédiate) :
     - `START_AUTONOMOUS_NAV=1 docker compose up rover_bridge`

2. Dans Foxglove Studio :
   - Connection -> `Foxglove WebSocket`
   - URL : `ws://localhost:8765`

3. Quand tu as roulé suffisamment pour couvrir la zone, lance SLAM :
   - `docker compose up slam_toolbox`

4. Lancer l'UI web locale :
   - `docker compose up web_ui`
   - Ouvrir `http://localhost:8080`

### Variables utiles
- `ROS_ROVER_IP` : IP du rover pour le pont HTTP/WS.
- `ROS_DOMAIN_ID` : DDS domain (garde la même valeur sur les deux services).
- `ESTIMATOR_MODE` : `lidar` (ICP 2D) ou `lio` (LiDAR + yaw IMU).
- `START_AUTONOMOUS_NAV` : 0 (pilotage manuel /cmd_vel) ou 1 (lancer l’autonome).
- `SLAM_START_DELAY_S` : délai avant démarrer slam_toolbox.

