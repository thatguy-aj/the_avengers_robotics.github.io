#!/bin/bash
set -e

# Search-and-Rescue Drone Simulator Launcher
# For PX4 + Gazebo simulation on Mac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
cd "$PX4_DIR"

# Set up the working environment
source .venv/bin/activate
export GZ_IP=127.0.0.1
export SDKROOT="${SDKROOT:-$(xcrun --sdk macosx --show-sdk-path)}"
source build/px4_sitl_default/gz_env.sh

MANIFEST="Tools/setup/macos/pixi.toml"
WORLD="$SCRIPT_DIR/worlds/rescue_park.sdf"

cleanup() {
    echo "Stopping simulator..."
    for pid in "${GUI_PID:-}" "${PX4_PID:-}" "${SERVER_PID:-}"; do
        if [[ -n "$pid" ]]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT
trap 'exit 130' INT TERM

echo "Starting Gazebo simulation server..."

pixi run --manifest-path "$MANIFEST" -- \
    gz sim -s -r -v 3 "$WORLD" \
    > /tmp/rescue_gazebo.log 2>&1 &
SERVER_PID=$!

# Wait for the Gazebo world to become available
READY=0
for i in {1..60}; do
    if pixi run --manifest-path "$MANIFEST" -- \
        gz service -l 2>/dev/null |
        grep -q '/world/default/scene/info'; then
        READY=1
        break
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        break
    fi
    sleep 2
done

if [[ "$READY" != 1 ]]; then
    echo "Gazebo world did not start. Check /tmp/rescue_gazebo.log"
    exit 1
fi

echo "Gazebo world is ready!"
echo "Starting PX4 flight controller..."

PX4_GZ_STANDALONE=1 pixi run \
    --manifest-path "$MANIFEST" -- \
    make px4_sitl gz_x500 \
    > /tmp/rescue_px4.log 2>&1 &
PX4_PID=$!

# Wait for PX4 startup
PX4_READY=0
for i in {1..180}; do
    if grep -q 'Startup script returned successfully' \
        /tmp/rescue_px4.log; then
        PX4_READY=1
        break
    fi
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        break
    fi
    sleep 2
done

if [[ "$PX4_READY" != 1 ]]; then
    echo "PX4 did not start. Check /tmp/rescue_px4.log"
    exit 1
fi

echo "PX4 started!"
echo "Opening Gazebo 3D viewer..."

pixi run --manifest-path "$MANIFEST" -- \
    gz sim -g -v 3 \
    > /tmp/rescue_viewer.log 2>&1 &
GUI_PID=$!

echo "Opening QGroundControl..."
if pgrep -f 'QGroundControl.app/Contents/MacOS/QGroundControl' > /dev/null; then
    echo "QGroundControl is already running."
else
    open -a QGroundControl || true
fi
echo "Simulator launched. Check QGroundControl for Ready."
echo "Keep this Terminal open. Press Control+C to stop."

wait "$PX4_PID"
