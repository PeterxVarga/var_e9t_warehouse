# ROS 2 Humble desktop on Ubuntu 22.04 (Jammy).
FROM docker.io/osrf/ros:humble-desktop-jammy@sha256:1ee56d3ee5111f81e0a2a17b6ecd73447afb5c5b8a3c3574754f2646da9aee34

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]
