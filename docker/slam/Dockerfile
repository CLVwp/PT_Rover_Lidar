FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

