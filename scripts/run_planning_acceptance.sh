#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOG_ROOT="${REPO_ROOT}/log/planning_acceptance"
PROFILE="${SCRIPT_DIR}/fastdds_acceptance.xml"
OUTER_TIMEOUT_SEC="${PLANNING_ACCEPTANCE_OUTER_TIMEOUT_SEC:-120}"

scenario="multi"
extra_arguments=()
for argument in "$@"; do
  case "${argument}" in
    scenario:=*)
      scenario="${argument#scenario:=}"
      ;;
    *)
      extra_arguments+=("${argument}")
      ;;
  esac
done

if [[ ! "${OUTER_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Invalid PLANNING_ACCEPTANCE_OUTER_TIMEOUT_SEC" >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2 Humble setup was not found" >&2
  exit 2
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "Workspace overlay missing; run colcon build first" >&2
  exit 2
fi
if [[ ! -f "${PROFILE}" ]]; then
  echo "Repository Fast DDS profile was not found" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

MAP_SHARE="$(ros2 pkg prefix --share drone_map)"
target_x="5.0"
target_y="0.0"
target_z="1.5"
case "${scenario}" in
  multi)
    map_file="${MAP_SHARE}/config/static_map.yaml"
    ;;
  single)
    map_file="${MAP_SHARE}/config/single_obstacle.yaml"
    ;;
  invalid_goal)
    map_file="${MAP_SHARE}/config/single_obstacle.yaml"
    target_x="2.5"
    ;;
  no_path)
    map_file="${MAP_SHARE}/config/no_path.yaml"
    ;;
  *)
    echo "Unknown scenario '${scenario}'" >&2
    exit 2
    ;;
esac

mkdir -p "${LOG_ROOT}"
timestamp="$(date '+%Y%m%d-%H%M%S')"
log_file="${LOG_ROOT}/${scenario}-${timestamp}.log"
fifo="${LOG_ROOT}/.${scenario}-${timestamp}.fifo"
mkfifo "${fifo}"

export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}"

launch_pid=""
tee_pid=""
interrupted=0

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  if [[ -n "${tee_pid}" ]]; then
    wait "${tee_pid}" 2>/dev/null || true
  fi
  rm -f -- "${fifo}"
  if [[ "${interrupted}" -eq 1 ]]; then
    exit 130
  fi
  exit "${code}"
}

on_interrupt() {
  interrupted=1
  exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

tee "${log_file}" <"${fifo}" &
tee_pid=$!

setsid ros2 launch drone_bringup planning_acceptance.launch.py \
  "scenario:=${scenario}" \
  "map_file:=${map_file}" \
  "target_x:=${target_x}" \
  "target_y:=${target_y}" \
  "target_z:=${target_z}" \
  "${extra_arguments[@]}" >"${fifo}" 2>&1 &
launch_pid=$!

deadline=$((SECONDS + OUTER_TIMEOUT_SEC))
timed_out=0
while kill -0 "${launch_pid}" 2>/dev/null; do
  if (( SECONDS >= deadline )); then
    timed_out=1
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    break
  fi
  sleep 0.2
done

set +e
wait "${launch_pid}"
launch_code=$?
set -e
launch_pid=""
wait "${tee_pid}" || true
tee_pid=""

if [[ "${timed_out}" -eq 1 ]]; then
  result=1
  summary="Wrapper result: timeout after ${OUTER_TIMEOUT_SEC} s"
elif grep -q "PLANNING ACCEPTANCE: FAIL" "${log_file}"; then
  result=1
  summary="Wrapper result: FAIL marker detected (launch=${launch_code})"
elif grep -q "process has died" "${log_file}"; then
  result=1
  summary="Wrapper result: node abnormal exit detected (launch=${launch_code})"
elif grep -q "PLANNING ACCEPTANCE: PASS" "${log_file}" \
    && [[ "${launch_code}" -eq 0 ]]; then
  result=0
  summary="Wrapper result: PASS (launch=${launch_code})"
else
  result=1
  summary="Wrapper result: no valid PASS result (launch=${launch_code})"
fi

{
  echo "${summary}"
  echo "Scenario: ${scenario}"
  echo "Shell exit code: ${result}"
  echo "Log: ${log_file}"
} | tee -a "${log_file}"

exit "${result}"
