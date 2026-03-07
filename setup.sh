#!/bin/bash
# setup.sh — One-time setup for the HP Robots Otto micro-ROS project.
# Installs all ROS2 dependencies and builds the workspace.
# Safe to run again — it skips steps that are already done.
#
# Usage:
#   bash setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

# Colours for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()    { echo -e "${GREEN}[setup]${NC} $*"; }
warning() { echo -e "${YELLOW}[setup]${NC} $*"; }
error()   { echo -e "${RED}[setup] ERROR:${NC} $*"; exit 1; }

echo ""
echo "  HP Robots Otto — setup"
echo "  ======================"
echo ""

# ── 1. Check ROS2 Jazzy ───────────────────────────────────────────────────────
if [ ! -f "$ROS_SETUP" ]; then
    error "ROS2 Jazzy not found at $ROS_SETUP.\n  Install it from: https://docs.ros.org/en/jazzy/Installation.html"
fi
info "ROS2 Jazzy found."

# Source ROS2 for this script
source "$ROS_SETUP"

# ── 2. Install ROS2 packages ──────────────────────────────────────────────────
info "Checking sudo access (you may be asked for your password)..."
if ! sudo -v; then
    error "sudo access is required to install packages.\n  Ask an administrator to add you to the sudo group."
fi
# Keep the sudo timestamp alive for the duration of the script
while true; do sudo -n true; sleep 50; done 2>/dev/null &
SUDO_KEEPALIVE_PID=$!
trap 'kill $SUDO_KEEPALIVE_PID 2>/dev/null' EXIT

info "Installing ROS2 packages..."
sudo apt-get install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    python3-colcon-common-extensions

# micro-ROS agent: try apt first, then build the eProsima standalone agent
if apt-cache show ros-jazzy-micro-ros-agent >/dev/null 2>&1; then
    info "Installing micro-ROS agent via apt..."
    sudo apt-get install -y ros-jazzy-micro-ros-agent
elif ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    info "Building Micro XRCE-DDS Agent from source (one-time, takes ~5 min)..."
    sudo apt-get install -y cmake libasio-dev libtinyxml2-dev libssl-dev
    AGENT_SRC="$SCRIPT_DIR/.build/micro-xrce-dds-agent"
    git clone --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_SRC"
    cmake -B "$AGENT_SRC/build" "$AGENT_SRC" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$AGENT_SRC/build" -j"$(nproc)"
    sudo cmake --install "$AGENT_SRC/build"
    info "MicroXRCEAgent installed to /usr/local/bin."
else
    info "MicroXRCEAgent already installed."
fi

# ── 3. Build the ROS2 workspace ───────────────────────────────────────────────
info "Building the ROS2 workspace..."
cd "$WS_DIR"
# Prepend system Python to PATH so colcon/CMake don't pick up conda's Python,
# which lacks catkin_pkg and other ROS2 build dependencies.
PATH=/usr/bin:/usr/local/bin:$PATH colcon build --symlink-install
cd "$SCRIPT_DIR"

# ── 4. Patch ~/.bashrc ────────────────────────────────────────────────────────
WS_SETUP="$WS_DIR/install/setup.bash"
BASHRC="$HOME/.bashrc"

if ! grep -qF "source $ROS_SETUP" "$BASHRC"; then
    info "Adding ROS2 source to ~/.bashrc..."
    echo "" >> "$BASHRC"
    echo "# ROS2 Jazzy" >> "$BASHRC"
    echo "source $ROS_SETUP" >> "$BASHRC"
else
    info "ROS2 already sourced in ~/.bashrc."
fi

if ! grep -qF "source \"$WS_SETUP\"" "$BASHRC"; then
    info "Adding workspace source to ~/.bashrc..."
    echo "# micro-ROS Otto workspace" >> "$BASHRC"
    echo "source \"$WS_SETUP\"" >> "$BASHRC"
else
    info "Workspace already sourced in ~/.bashrc."
fi

# ── 5. Done ───────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}  Setup complete!${NC}"
echo ""
echo "  Next steps:"
echo "  1. Open a new terminal (or run: source ~/.bashrc)"
echo "  2. Power on the robot and wait for the blue LED to blink"
echo "  3. Run:  ./start.sh"
echo ""
