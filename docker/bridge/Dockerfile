FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Dépendances système (Python + outils ROS + libs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-scipy \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Dépendances Python côté rover_bridge.py
RUN pip3 install --no-cache-dir --break-system-packages \
    websockets

WORKDIR /workspace

