#!/usr/bin/env bash
set -eo pipefail

cd /workspace

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

exec python3 /workspace/docker/webui/app.py

