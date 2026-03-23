# Docker Cheat Sheet

## Juste redemarrer
```powershell
docker compose restart
```

## Recharger `docker-compose.yml` / variables env
```powershell
docker compose up -d --force-recreate
```

## Rebuild des conteneurs sans `--no-cache`
```powershell
docker compose build
docker compose up -d --force-recreate
```

## Rebuild de certains services seulement
```powershell
docker compose build rover_bridge slam_toolbox web_ui
docker compose up -d --force-recreate rover_bridge slam_toolbox web_ui
```

## Rebuild complet sans cache
```powershell
docker compose down --remove-orphans
docker compose build --no-cache rover_bridge slam_toolbox web_ui
docker compose up -d --force-recreate rover_bridge slam_toolbox web_ui
```