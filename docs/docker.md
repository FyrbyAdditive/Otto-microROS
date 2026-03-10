# Docker — Running the ROS2 Stack in Containers

Run the Otto ROS2 host stack without installing ROS2 natively. You still need PlatformIO on the host to flash the ESP32 firmware.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) (v2+)
- The robot firmware flashed and WiFi configured (see [flashing.md](flashing.md))

## Build the image

From the project root:

```bash
docker compose -f docker/docker-compose.yml build
```

This builds a single image containing the micro-ROS agent and the full ROS2 workspace. First build takes a few minutes.

## Start the robot stack

```bash
docker compose -f docker/docker-compose.yml up agent bringup
```

This starts:
- **agent** — micro-ROS UDP bridge (port 8888) that connects to the robot
- **bringup** — robot state publisher, odometry, scan converter, and visualizer

When the robot connects, you'll see agent output showing the ESP32's topics being registered.

## Drive the robot

In a separate terminal:

```bash
docker compose -f docker/docker-compose.yml run teleop
```

This opens an interactive keyboard teleop session. Use `i`/`,`/`j`/`l` to drive, `k` to stop.

## Mapping demo

```bash
docker compose -f docker/docker-compose.yml up agent mapping
```

This replaces `bringup` with the mapping stack (includes slam_toolbox). Drive with the teleop service in another terminal.

## RViz (3D visualisation)

RViz requires X11 forwarding from the container to your display:

```bash
# Allow Docker to access X11
xhost +local:docker

# Start RViz
docker compose -f docker/docker-compose.yml up rviz
```

If you have an NVIDIA GPU, uncomment the `deploy` section in `docker-compose.yml` for GPU-accelerated rendering.

## Useful commands

```bash
# View logs from a specific service
docker compose -f docker/docker-compose.yml logs -f agent

# Stop everything
docker compose -f docker/docker-compose.yml down

# Rebuild after code changes
docker compose -f docker/docker-compose.yml build --no-cache

# Run any ROS2 command inside the container
docker compose -f docker/docker-compose.yml run --rm bringup ros2 topic list
```

## How it works

All services use `network_mode: host` so they share the host's network stack. This is required for:
- DDS multicast discovery (ROS2 nodes finding each other)
- UDP port 8888 (micro-ROS agent ↔ ESP32 communication)

The Dockerfile builds the Micro XRCE-DDS Agent from source (same agent the firmware talks to) and the Otto ROS2 workspace. The firmware is not included — you flash it via PlatformIO on the host machine.
