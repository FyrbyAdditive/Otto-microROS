# Docker

Run the Otto ROS2 host stack in containers without installing ROS2 natively. You still need PlatformIO on the host to flash the ESP32 firmware.

---

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) (v2+)
- The robot firmware flashed and WiFi configured (see [flashing.md](flashing.md))

---

## Build the image

From the project root:

```bash
docker compose -f docker/docker-compose.yml build
```

This builds a single image containing the Micro XRCE-DDS Agent and the full ROS2 workspace. First build takes a few minutes; subsequent builds use the Docker cache.

---

## Start the robot stack

```bash
docker compose -f docker/docker-compose.yml up agent bringup
```

This starts:

| Service | Purpose |
|---------|---------|
| **agent** | micro-ROS UDP bridge (port 8888) — connects to the robot |
| **bringup** | Robot state publisher, odometry, scan converter, and visualizer |

When the robot connects, you'll see agent output showing the ESP32's topics being registered.

---

## Drive the robot

In a separate terminal:

```bash
docker compose -f docker/docker-compose.yml run teleop
```

This opens an interactive keyboard teleop session. Use `i`/`,`/`j`/`l` to drive, `k` to stop.

---

## RViz (3D visualisation)

RViz requires X11 forwarding from the container to your display. Run it alongside the agent and bringup:

```bash
# Allow Docker to access your X11 display (one-time per session)
xhost +local:docker

# Start agent, bringup, and RViz together
docker compose -f docker/docker-compose.yml up agent bringup rviz
```

If you have an NVIDIA GPU, uncomment the `deploy` section in [`docker-compose.yml`](../docker/docker-compose.yml) for GPU-accelerated rendering.

---

## Mapping demo

The mapping stack replaces `bringup` with a version that includes slam_toolbox for occupancy grid mapping:

```bash
# Allow Docker to access your X11 display (if not already done)
xhost +local:docker

# Start agent, mapping stack, and RViz
docker compose -f docker/docker-compose.yml up agent mapping rviz
```

Drive with the teleop service in another terminal. Rotate in place first to seed the map, then drive around slowly. See [mapping_demo.md](mapping_demo.md) for tips.

---

## Useful commands

```bash
# View logs from a specific service
docker compose -f docker/docker-compose.yml logs -f agent

# Stop everything
docker compose -f docker/docker-compose.yml down

# Rebuild after code changes
docker compose -f docker/docker-compose.yml build --no-cache

# Run any ROS2 command inside a container
docker compose -f docker/docker-compose.yml run --rm bringup \
  ros2 topic list
```

---

## How it works

All services use `network_mode: host` so they share the host's network stack. This is required for:

- **DDS multicast discovery** — ROS2 nodes finding each other
- **UDP port 8888** — micro-ROS agent to ESP32 communication

A custom [FastDDS profile](../docker/fastdds.xml) disables shared-memory transport and forces UDP, which is necessary for DDS communication between separate containers.

The [Dockerfile](../docker/Dockerfile) builds the Micro XRCE-DDS Agent from source (the same agent the firmware talks to) and the Otto ROS2 workspace. The firmware is **not** included — you flash it via PlatformIO on the host machine.
