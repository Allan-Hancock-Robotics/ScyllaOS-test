#!/usr/bin/env bash
set -eo pipefail

PI_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MAVROS_PARAMS="${PI_WS}/mavros_ardusub.yaml"
JOY_PARAMS="${PI_WS}/src/controls/config/joy_to_manual_control.yaml"
LOG_DIR="${PI_WS}/logs"

QGC_CAMERA_SCRIPT="${QGC_CAMERA_SCRIPT:-${PI_WS}/start_qgc_camera.sh}"

mkdir -p "${LOG_DIR}"

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

source "${PI_WS}/install/setup.bash"

MAVROS_PID=""
JOY_PID=""
QGC_CAMERA_PID=""
MAVROS_READY=0

cleanup() {
    trap - INT TERM EXIT

    echo ""
    echo "Stopping control stack..."

    if [[ "${MAVROS_READY}" == "1" ]]; then
        echo "Disarming vehicle..."
        timeout 5s ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}" >/dev/null 2>&1 || true
    else
        echo "MAVROS was not ready; skipping disarm."
    fi

    if [[ -n "${QGC_CAMERA_PID}" ]]; then
        echo "Stopping QGC camera stream..."
        kill "${QGC_CAMERA_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${JOY_PID}" ]]; then
        echo "Stopping joy_to_manual_control..."
        kill "${JOY_PID}" >/dev/null 2>&1 || true
    fi

    if [[ -n "${MAVROS_PID}" ]]; then
        echo "Stopping MAVROS..."
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

echo "Starting MAVROS..."
ros2 run mavros mavros_node --ros-args \
    --params-file "${MAVROS_PARAMS}" \
    > "${LOG_DIR}/mavros.log" 2>&1 &
MAVROS_PID=$!

echo "MAVROS PID: ${MAVROS_PID}"
echo "MAVROS log: ${LOG_DIR}/mavros.log"

echo "Waiting for MAVROS connection..."
MAVROS_CONNECTED=0

for i in $(seq 1 90); do
    if ! kill -0 "${MAVROS_PID}" >/dev/null 2>&1; then
        echo "ERROR: MAVROS exited early."
        echo "Last MAVROS log lines:"
        tail -n 80 "${LOG_DIR}/mavros.log" || true
        exit 1
    fi

    CONNECTED_FIELD="$(ros2 topic echo /mavros/state --once --field connected)"
    echo "status: ${CONNECTED_FIELD}"
    if echo "${CONNECTED_FIELD}" | grep -qi "true"; then
        echo "MAVROS connected."
        MAVROS_CONNECTED=1
        break
    fi

    echo "  waiting... ${i}/90"
    sleep 1
done

if [[ "${MAVROS_CONNECTED}" != "1" ]]; then
    echo "ERROR: Timed out waiting for MAVROS connected."
    echo "Current /mavros/state:"
    timeout 5s ros2 topic echo /mavros/state --once || true
    echo "Last MAVROS log lines:"
    tail -n 80 "${LOG_DIR}/mavros.log" || true
    exit 1
fi

echo "Waiting for arming service..."
ARMING_SERVICE_READY=0

for i in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q "^/mavros/cmd/arming$"; then
        echo "Arming service available."
        ARMING_SERVICE_READY=1
        break
    fi

    echo "  waiting for arming service... ${i}/30"
    sleep 0.5
done

if [[ "${ARMING_SERVICE_READY}" != "1" ]]; then
    echo "ERROR: Timed out waiting for /mavros/cmd/arming."
    ros2 service list | grep mavros || true
    exit 1
fi

MAVROS_READY=1

echo "Starting QGC camera stream..."

if [[ -f "${QGC_CAMERA_SCRIPT}" ]]; then
    chmod +x "${QGC_CAMERA_SCRIPT}"

    "${QGC_CAMERA_SCRIPT}" > "${LOG_DIR}/qgc_camera.log" 2>&1 &
    QGC_CAMERA_PID=$!

    echo "QGC camera PID: ${QGC_CAMERA_PID}"
    echo "QGC camera log: ${LOG_DIR}/qgc_camera.log"
else
    echo "WARNING: QGC camera script not found:"
    echo "  ${QGC_CAMERA_SCRIPT}"
    echo "Skipping QGC camera stream."
fi

echo "Starting joy_to_manual_control..."
ros2 run controls joy_to_manual_control --ros-args \
    --params-file "${JOY_PARAMS}" \
    > "${LOG_DIR}/joy_to_manual_control.log" 2>&1 &
JOY_PID=$!

echo "joy_to_manual_control PID: ${JOY_PID}"
echo "joy_to_manual_control log: ${LOG_DIR}/joy_to_manual_control.log"

echo "Arming vehicle..."
ARM_RESPONSE="$(timeout 10s ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 2>&1 || true)"
echo "${ARM_RESPONSE}"

echo "State after arming attempt:"
timeout 3s ros2 topic echo /mavros/state --once || true

echo ""
echo "Control stack running."
echo "MAVROS PID:                ${MAVROS_PID}"
echo "joy_to_manual_control PID: ${JOY_PID}"

if [[ -n "${QGC_CAMERA_PID}" ]]; then
    echo "QGC camera PID:            ${QGC_CAMERA_PID}"
fi

echo ""
echo "Logs:"
echo "  tail -f ${LOG_DIR}/mavros.log"
echo "  tail -f ${LOG_DIR}/joy_to_manual_control.log"
echo "  tail -f ${LOG_DIR}/qgc_camera.log"
echo ""
echo "Press Ctrl+C to disarm and stop."

wait