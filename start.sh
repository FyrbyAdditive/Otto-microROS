#!/bin/bash
# start.sh — Start the micro-ROS agent and robot stack.
# Run this every time you want to use the robot.
#
# Usage:
#   ./start.sh
#
# The robot must be powered on and connected to WiFi first.
# When the robot connects, its blue LED will go solid.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$SCRIPT_DIR/ros2_ws/install/setup.bash"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# ── Preflight checks ──────────────────────────────────────────────────────────
if [ ! -f "$ROS_SETUP" ]; then
    echo -e "${RED}ERROR:${NC} ROS2 Jazzy not found. Run ./setup.sh first."
    exit 1
fi

if [ ! -f "$WS_SETUP" ]; then
    echo -e "${RED}ERROR:${NC} Workspace not built. Run ./setup.sh first."
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

# Check the workspace built successfully
if ! ros2 pkg list 2>/dev/null | grep -q otto_bringup; then
    echo -e "${RED}ERROR:${NC} otto_bringup package not found. Run ./setup.sh first."
    exit 1
fi

# ── Launch ────────────────────────────────────────────────────────────────────
echo ""
echo "  HP Robots Otto — starting"
echo "  ========================="
echo ""
echo "  Power on the robot and wait for the blue LED to blink."
echo "  When connected, the blue LED will go solid and the ring will glow blue."
echo ""
echo "  To drive the robot, open a new terminal and run:"
echo -e "    ${GREEN}ros2 launch otto_bringup otto_teleop.launch.py${NC}"
echo ""
echo "  To visualise in RViz, open a new terminal and run:"
echo -e "    ${GREEN}ros2 launch otto_description display.launch.py${NC}"
echo ""
echo "  Press Ctrl+C to stop."
echo ""

# Use gnome-terminal tabs if available (desktop session),
# otherwise fall back to launching both in this terminal via tmux,
# otherwise print clear instructions.

if command -v gnome-terminal &>/dev/null && [ -n "$DISPLAY" ]; then
    # Open agent in a new tab, bringup in another
    gnome-terminal \
        --tab --title="micro-ROS Agent" \
            -- bash -c "source $ROS_SETUP && source $WS_SETUP && \
                        echo 'micro-ROS Agent — waiting for robot...' && \
                        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; \
                        echo 'Agent stopped. Press Enter to close.'; read" \
        --tab --title="Otto Bringup" \
            -- bash -c "source $ROS_SETUP && source $WS_SETUP && \
                        sleep 2 && \
                        ros2 launch otto_bringup otto_microros.launch.py agent:=none; \
                        echo 'Bringup stopped. Press Enter to close.'; read"
    echo "Opened two terminal tabs: 'micro-ROS Agent' and 'Otto Bringup'."
    echo "Close those tabs to stop the robot stack."

elif command -v tmux &>/dev/null; then
    SESSION="otto"
    tmux new-session -d -s "$SESSION" -x 220 -y 50
    tmux rename-window -t "$SESSION:0" "Agent"
    tmux send-keys -t "$SESSION:0" \
        "source $ROS_SETUP && source $WS_SETUP && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888" Enter
    tmux new-window -t "$SESSION" -n "Bringup"
    tmux send-keys -t "$SESSION:1" \
        "sleep 2 && source $ROS_SETUP && source $WS_SETUP && ros2 launch otto_bringup otto_microros.launch.py agent:=none" Enter
    echo "Started tmux session '$SESSION'."
    echo "Attach with:  tmux attach -t $SESSION"
    echo "Stop with:    tmux kill-session -t $SESSION"
    tmux attach -t "$SESSION"

else
    # No GUI or tmux — run agent in background, bringup in foreground
    echo -e "${YELLOW}Tip:${NC} Install tmux for a better experience: sudo apt install tmux"
    echo ""
    echo "Starting micro-ROS agent in the background..."
    bash -c "source $ROS_SETUP && source $WS_SETUP && \
             ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888" &
    AGENT_PID=$!
    trap "kill $AGENT_PID 2>/dev/null; exit" INT TERM

    sleep 2
    echo "Starting robot bringup (Ctrl+C to stop both)..."
    source "$ROS_SETUP" && source "$WS_SETUP" && \
        ros2 launch otto_bringup otto_microros.launch.py agent:=none

    kill $AGENT_PID 2>/dev/null
fi
