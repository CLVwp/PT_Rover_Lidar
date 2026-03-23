docker compose down --remove-orphans
docker compose build --no-cache rover_bridge slam_toolbox web_ui
docker compose up -d --force-recreate rover_bridge slam_toolbox web_ui