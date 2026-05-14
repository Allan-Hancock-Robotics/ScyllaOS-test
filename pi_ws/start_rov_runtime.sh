#!/usr/bin/env bash
set -eo pipefail

PI_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MAVROS_PARAMS="${PI_WS}/mavros_ardusub.yaml"
JOY_PARAMS="${PI_WS}/src/controls/config/joy_to_manual_control.yaml"
LOG_DIR="${PI_WS}/logs"

VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1920}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1080}"
CAMERA_FRAME_ID="${CAMERA_FRAME_ID:-camera_link}"

mkdir -p "${LOG_DIR}"

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

set +u
source "${PI_WS}/install/setup.bash"
set -u

MAVROS_PID=""
JOY_PID=""
CAMERA_PID=""

cleanup() {
    trap - INT TERM EXIT

    echo ""
    echo "Stopping ROV runtime..."

    if [[ -n "${CAMERA_PID}" ]]; then
        kill "${CAMERA_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${JOY_PID}" ]]; then
        kill "${JOY_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${MAVROS_PID}" ]]; then
        kill "${MAVROS_PID}" >/dev/null 2>&1 || true
    fi

    wait >/dev/null 2>&1 || true
    echo "Stopped."
}

trap cleanup INT TERM EXIT

if [[ ! -f "${MAVROS_PARAMS}" ]]; then
    echo "ERROR: MAVROS params file not found:"
    echo "  ${MAVROS_PARAMS}"
    exit 1
fi

if [[ ! -f "${JOY_PARAMS}" ]]; then
    echo "ERROR: joy_to_manual_control params file not found:"
    echo "  ${JOY_PARAMS}"
    exit 1
fi

run_camera_tools_installer() {
    echo "Running offline camera tools installer..."

    if [[ -n "${CAMERA_INSTALLER_CMD:-}" ]]; then
        echo "Using CAMERA_INSTALLER_CMD:"
        echo "  ${CAMERA_INSTALLER_CMD}"
        bash -lc "${CAMERA_INSTALLER_CMD}"
        return
    fi

    if [[ -x "${PI_WS}/install_offline_camera_tools.sh" ]]; then
        "${PI_WS}/install_offline_camera_tools.sh"
        return
    fi

    if [[ -x "${PI_WS}/scripts/install_offline_camera_tools.sh" ]]; then
        "${PI_WS}/scripts/install_offline_camera_tools.sh"
        return
    fi

    if command -v install_offline_camera_tools >/dev/null 2>&1; then
        install_offline_camera_tools
        return
    fi

    if command -v offline_camera_tools_installer >/dev/null 2>&1; then
        offline_camera_tools_installer
        return
    fi

    echo "WARNING: Could not find an offline camera tools installer."
    echo "Set CAMERA_INSTALLER_CMD to the exact installer command if needed."
}

echo "Starting MAVROS..."
ros2 run mavros mavros_node --ros-args \
    --params-file "${MAVROS_PARAMS}" \
    > "${LOG_DIR}/mavros.log" 2>&1 &
MAVROS_PID=$!

echo "MAVROS PID: ${MAVROS_PID}"
echo "MAVROS log: ${LOG_DIR}/mavros.log"

echo "Waiting for MAVROS connection..."
MAVROS_CONNECTED=0

for i in $(seq 1 60); do
    if grep -q "Got HEARTBEAT, connected" "${LOG_DIR}/mavros.log" 2>/dev/null; then
        MAVROS_CONNECTED=1
        echo "MAVROS connected."
        break
    fi

    if ! kill -0 "${MAVROS_PID}" >/dev/null 2>&1; then
        echo "ERROR: MAVROS exited early."
        tail -n 50 "${LOG_DIR}/mavros.log" || true
        exit 1
    fi

    echo "  waiting for MAVROS... ${i}/60"
    sleep 0.5
done

if [[ "${MAVROS_CONNECTED}" != "1" ]]; then
    echo "ERROR: Timed out waiting for MAVROS connected."
    echo "Last MAVROS log lines:"
    tail -n 80 "${LOG_DIR}/mavros.log" || true
    exit 1
fi

echo "Starting joy_to_manual_control..."
ros2 run controls joy_to_manual_control --ros-args \
    --params-file "${JOY_PARAMS}" \
    > "${LOG_DIR}/joy_to_manual_control.log" 2>&1 &
JOY_PID=$!

echo "joy_to_manual_control PID: ${JOY_PID}"
echo "joy_to_manual_control log: ${LOG_DIR}/joy_to_manual_control.log"

run_camera_tools_installer > "${LOG_DIR}/camera_tools_installer.log" 2>&1 || {
    echo "ERROR: Camera tools installer failed."
    tail -n 50 "${LOG_DIR}/camera_tools_installer.log" || true
    exit 1
}

echo "Starting v4l2_camera at ${CAMERA_WIDTH}x${CAMERA_HEIGHT} on ${VIDEO_DEVICE}..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p "video_device:=${VIDEO_DEVICE}" \
    -p "image_size:=[${CAMERA_WIDTH},${CAMERA_HEIGHT}]" \
    -p "camera_frame_id:=${CAMERA_FRAME_ID}" \
    > "${LOG_DIR}/v4l2_camera.log" 2>&1 &
CAMERA_PID=$!

echo "v4l2_camera PID: ${CAMERA_PID}"
echo "v4l2_camera log: ${LOG_DIR}/v4l2_camera.log"

echo ""
echo "ROV runtime is running:"
echo "  MAVROS PID:                ${MAVROS_PID}"
echo "  joy_to_manual_control PID: ${JOY_PID}"
echo "  v4l2_camera PID:           ${CAMERA_PID}"
echo ""
echo "Logs:"
echo "  tail -f ${LOG_DIR}/mavros.log"
echo "  tail -f ${LOG_DIR}/joy_to_manual_control.log"
echo "  tail -f ${LOG_DIR}/camera_tools_installer.log"
echo "  tail -f ${LOG_DIR}/v4l2_camera.log"
echo ""
echo "Press Ctrl+C to stop all processes."

wait