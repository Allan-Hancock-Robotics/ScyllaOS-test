#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# start_rov_runtime.sh  –  Bring up the ROV software stack on the Pi
#
# Starts (in order):
#   1. mavros_node          – waits for FCU connection before continuing
#   2. joy_to_manual_control
#   3. offline camera tools installer  (blocking, must succeed)
#   4. v4l2_camera_node
#
# After startup the script monitors all three long-running processes and exits
# (triggering cleanup) as soon as any one of them dies.
# -----------------------------------------------------------------------------
set -eo pipefail

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PI_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MAVROS_PARAMS="${PI_WS}/mavros_ardusub.yaml"
JOY_PARAMS="${PI_WS}/src/controls/config/joy_to_manual_control.yaml"
LOG_DIR="${PI_WS}/logs"

# ---------------------------------------------------------------------------
# Camera config  (override via environment)
# ---------------------------------------------------------------------------
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1920}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1080}"
CAMERA_FRAME_ID="${CAMERA_FRAME_ID:-camera_link}"

# ---------------------------------------------------------------------------
# MAVROS connection poll tuning
# ---------------------------------------------------------------------------
MAVROS_POLL_ATTEMPTS="${MAVROS_POLL_ATTEMPTS:-120}"   # iterations
MAVROS_POLL_INTERVAL="${MAVROS_POLL_INTERVAL:-0.5}"   # seconds each → 60 s total

mkdir -p "${LOG_DIR}"

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Source the ROS 2 workspace (allow unbound vars momentarily)
set +u
# shellcheck source=/dev/null
source "${PI_WS}/install/setup.bash"
set -u

# ---------------------------------------------------------------------------
# Process bookkeeping
# ---------------------------------------------------------------------------
MAVROS_PID=""
JOY_PID=""
CAMERA_PID=""

# ---------------------------------------------------------------------------
# Cleanup – called on INT / TERM / EXIT
# ---------------------------------------------------------------------------
cleanup() {
    trap - INT TERM EXIT
    echo ""
    echo "Stopping ROV runtime..."

    # Send SIGTERM to every child that is still alive
    for pid_var in CAMERA_PID JOY_PID MAVROS_PID; do
        local pid="${!pid_var}"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            echo "  Sending SIGTERM to ${pid_var} (PID ${pid})..."
            kill "${pid}" 2>/dev/null || true
        fi
    done

    # Give processes up to 5 s to exit gracefully, then SIGKILL stragglers
    local deadline=$(( SECONDS + 5 ))
    for pid_var in CAMERA_PID JOY_PID MAVROS_PID; do
        local pid="${!pid_var}"
        if [[ -n "${pid}" ]]; then
            while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
                sleep 0.2
            done
            if kill -0 "${pid}" 2>/dev/null; then
                echo "  Force-killing ${pid_var} (PID ${pid})..."
                kill -9 "${pid}" 2>/dev/null || true
            fi
        fi
    done

    wait 2>/dev/null || true
    echo "Stopped."
}

trap cleanup INT TERM EXIT

# ---------------------------------------------------------------------------
# Preflight checks
# ---------------------------------------------------------------------------
if [[ ! -f "${MAVROS_PARAMS}" ]]; then
    echo "ERROR: MAVROS params file not found: ${MAVROS_PARAMS}"
    exit 1
fi

if [[ ! -f "${JOY_PARAMS}" ]]; then
    echo "ERROR: joy_to_manual_control params file not found: ${JOY_PARAMS}"
    exit 1
fi

# ---------------------------------------------------------------------------
# Helper: locate and run the offline camera tools installer
# ---------------------------------------------------------------------------
run_camera_tools_installer() {
    echo "Running offline camera tools installer..."

    if [[ -n "${CAMERA_INSTALLER_CMD:-}" ]]; then
        echo "  Using CAMERA_INSTALLER_CMD: ${CAMERA_INSTALLER_CMD}"
        bash -lc "${CAMERA_INSTALLER_CMD}"
        return
    fi

    local candidates=(
        "${PI_WS}/install_offline_camera_tools.sh"
        "${PI_WS}/scripts/install_offline_camera_tools.sh"
    )
    for script in "${candidates[@]}"; do
        if [[ -x "${script}" ]]; then
            echo "  Found installer: ${script}"
            "${script}"
            return
        fi
    done

    local cmds=(install_offline_camera_tools offline_camera_tools_installer)
    for cmd in "${cmds[@]}"; do
        if command -v "${cmd}" >/dev/null 2>&1; then
            echo "  Found installer command: ${cmd}"
            "${cmd}"
            return
        fi
    done

    echo "WARNING: No offline camera tools installer found."
    echo "  Set CAMERA_INSTALLER_CMD to the installer command if one is needed."
}

# ---------------------------------------------------------------------------
# Helper: assert a background process is still alive
# ---------------------------------------------------------------------------
assert_alive() {
    local name="$1" pid="$2"
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "ERROR: ${name} (PID ${pid}) exited unexpectedly."
        return 1
    fi
}

# ---------------------------------------------------------------------------
# 1. Start MAVROS
# ---------------------------------------------------------------------------
echo "Starting MAVROS..."
ros2 run mavros mavros_node --ros-args \
    --params-file "${MAVROS_PARAMS}" \
    > "${LOG_DIR}/mavros.log" 2>&1 &
MAVROS_PID=$!
echo "  PID: ${MAVROS_PID}  |  log: ${LOG_DIR}/mavros.log"

# Wait for FCU connection
echo "Waiting for MAVROS FCU connection (up to $(echo "${MAVROS_POLL_ATTEMPTS} * ${MAVROS_POLL_INTERVAL}" | bc -l | xargs printf '%.0f') s)..."
MAVROS_CONNECTED=0

for (( i=1; i<=MAVROS_POLL_ATTEMPTS; i++ )); do
    # Bail early if the node already died
    if ! kill -0 "${MAVROS_PID}" 2>/dev/null; then
        echo "ERROR: MAVROS exited before connecting."
        tail -n 50 "${LOG_DIR}/mavros.log" || true
        exit 1
    fi

    STATE="$(timeout 2s ros2 topic echo /mavros/state --once 2>/dev/null || true)"
    if echo "${STATE}" | grep -q "connected: true"; then
        MAVROS_CONNECTED=1
        echo "MAVROS connected to FCU."
        break
    fi

    printf "  waiting... %d/%d\r" "${i}" "${MAVROS_POLL_ATTEMPTS}"
    sleep "${MAVROS_POLL_INTERVAL}"
done

if [[ "${MAVROS_CONNECTED}" != "1" ]]; then
    echo ""
    echo "ERROR: Timed out waiting for MAVROS FCU connection."
    tail -n 50 "${LOG_DIR}/mavros.log" || true
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. Start joy_to_manual_control
# ---------------------------------------------------------------------------
echo "Starting joy_to_manual_control..."
ros2 run controls joy_to_manual_control --ros-args \
    --params-file "${JOY_PARAMS}" \
    > "${LOG_DIR}/joy_to_manual_control.log" 2>&1 &
JOY_PID=$!
echo "  PID: ${JOY_PID}  |  log: ${LOG_DIR}/joy_to_manual_control.log"

assert_alive "joy_to_manual_control" "${JOY_PID}" || exit 1

# ---------------------------------------------------------------------------
# 3. Offline camera tools installer  (blocking; must succeed)
# ---------------------------------------------------------------------------
run_camera_tools_installer > "${LOG_DIR}/camera_tools_installer.log" 2>&1 || {
    echo "ERROR: Camera tools installer failed."
    tail -n 50 "${LOG_DIR}/camera_tools_installer.log" || true
    exit 1
}
echo "  Camera tools installer succeeded."

# ---------------------------------------------------------------------------
# 4. Start v4l2_camera
# ---------------------------------------------------------------------------
echo "Starting v4l2_camera (${CAMERA_WIDTH}x${CAMERA_HEIGHT} on ${VIDEO_DEVICE})..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p "video_device:=${VIDEO_DEVICE}" \
    -p "image_size:=[${CAMERA_WIDTH},${CAMERA_HEIGHT}]" \
    -p "camera_frame_id:=${CAMERA_FRAME_ID}" \
    > "${LOG_DIR}/v4l2_camera.log" 2>&1 &
CAMERA_PID=$!
echo "  PID: ${CAMERA_PID}  |  log: ${LOG_DIR}/v4l2_camera.log"

assert_alive "v4l2_camera" "${CAMERA_PID}" || exit 1

# ---------------------------------------------------------------------------
# Status summary
# ---------------------------------------------------------------------------
echo ""
echo "ROV runtime running:"
printf "  %-30s PID %s\n" "mavros_node"            "${MAVROS_PID}"
printf "  %-30s PID %s\n" "joy_to_manual_control"  "${JOY_PID}"
printf "  %-30s PID %s\n" "v4l2_camera_node"       "${CAMERA_PID}"
echo ""
echo "Live logs:"
echo "  tail -f ${LOG_DIR}/mavros.log"
echo "  tail -f ${LOG_DIR}/joy_to_manual_control.log"
echo "  tail -f ${LOG_DIR}/camera_tools_installer.log"
echo "  tail -f ${LOG_DIR}/v4l2_camera.log"
echo ""
echo "Press Ctrl+C to stop."

# ---------------------------------------------------------------------------
# Monitor: exit (triggering cleanup) as soon as any process dies
# ---------------------------------------------------------------------------
while true; do
    for entry in "MAVROS:${MAVROS_PID}" "joy_to_manual_control:${JOY_PID}" "v4l2_camera:${CAMERA_PID}"; do
        name="${entry%%:*}"
        pid="${entry##*:}"
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo ""
            echo "ERROR: ${name} (PID ${pid}) died unexpectedly."
            echo "Last 20 lines of its log:"
            tail -n 20 "${LOG_DIR}/${name}.log" 2>/dev/null || true
            exit 1
        fi
    done
    sleep 1
done
