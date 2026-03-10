# Dockerfile — HP Robots Otto ROS2 host stack
#
# Builds the micro-ROS agent + Otto ROS2 workspace in a single image.
# Use with docker-compose.yml for easy startup.
#
# Build:
#   docker compose -f docker/docker-compose.yml build
#
# The firmware is NOT included — flash via PlatformIO on the host.

FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# ── 1. Install ROS2 dependencies ────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    python3-colcon-common-extensions \
    xterm \
    cmake \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Build micro-ROS agent from source ────────────────────────────────────
RUN git clone --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent && \
    cmake -B /tmp/agent/build /tmp/agent -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/agent/build -j"$(nproc)" && \
    cmake --install /tmp/agent/build && \
    ldconfig && \
    rm -rf /tmp/agent

# ── 3. Copy and build the ROS2 workspace ────────────────────────────────────
COPY ros2_ws/src /opt/otto/ros2_ws/src

RUN . /opt/ros/jazzy/setup.sh && \
    cd /opt/otto/ros2_ws && \
    colcon build --symlink-install \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# ── 4. Entrypoint ───────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
