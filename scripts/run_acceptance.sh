#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1
  pwd
)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LOG_DIRECTORY="${REPOSITORY_ROOT}/log/acceptance"
TIMESTAMP="$(date '+%Y%m%d-%H%M%S')"
LOG_PATH="${LOG_DIRECTORY}/acceptance-${TIMESTAMP}-$$.log"

mkdir -p -- "${LOG_DIRECTORY}"
touch -- "${LOG_PATH}"

print_summary() {
  local result="$1"
  local shell_exit_code="$2"

  printf '\nAcceptance wrapper summary\n'
  printf 'result: %s\n' "${result}"
  printf 'log path: %s\n' "${LOG_PATH}"
  printf 'shell exit code: %s\n' "${shell_exit_code}"
}

fail_before_launch() {
  local reason="$1"
  local exit_code="$2"

  printf 'ERROR: %s\n' "${reason}" | tee -a "${LOG_PATH}" >&2
  print_summary "FAIL" "${exit_code}"
  exit "${exit_code}"
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  fail_before_launch \
    "ROS2 Humble setup was not found at /opt/ros/humble/setup.bash." \
    2
fi

# Humble's generated setup files read optional variables that may be unset.
set +u

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ ! -f "${REPOSITORY_ROOT}/install/setup.bash" ]]; then
  fail_before_launch \
    "Workspace overlay is missing; run colcon build first." \
    2
fi

# shellcheck disable=SC1091
source "${REPOSITORY_ROOT}/install/setup.bash"

set -u

# The acceptance graph is intentionally self-contained on this machine.
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE=\
"${SCRIPT_DIR}/fastdds_acceptance.xml"

if [[ ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
  fail_before_launch \
    "Fast DDS acceptance profile is missing." \
    2
fi

printf 'ROS_LOCALHOST_ONLY: %s\n' \
  "${ROS_LOCALHOST_ONLY}" >>"${LOG_PATH}"
printf 'FASTRTPS_DEFAULT_PROFILES_FILE: %s\n' \
  "${FASTRTPS_DEFAULT_PROFILES_FILE}" >>"${LOG_PATH}"

discovery_timeout_sec="15.0"
convergence_timeout_sec="20.0"

for argument in "$@"; do
  case "${argument}" in
    discovery_timeout_sec:=*)
      discovery_timeout_sec="${argument#*:=}"
      ;;
    timeout_sec:=*)
      convergence_timeout_sec="${argument#*:=}"
      ;;
  esac
done

is_positive_number() {
  local value="$1"

  awk -v value="${value}" '
    BEGIN {
      valid = value ~ /^[0-9]+([.][0-9]*)?([eE][+-]?[0-9]+)?$/
      if (valid && (value + 0.0) > 0.0) {
        exit 0
      }
      exit 1
    }
  '
}

if ! is_positive_number "${discovery_timeout_sec}"; then
  fail_before_launch \
    "discovery_timeout_sec must be a positive number." \
    2
fi

if ! is_positive_number "${convergence_timeout_sec}"; then
  fail_before_launch \
    "timeout_sec must be a positive number." \
    2
fi

outer_timeout_sec="$(
  awk \
    -v discovery="${discovery_timeout_sec}" \
    -v convergence="${convergence_timeout_sec}" '
      BEGIN {
        margin = 10.0
        total = discovery + convergence + margin
        rounded_up = int(total)
        if (rounded_up < total) {
          rounded_up += 1
        }
        print rounded_up
      }
    '
)"

run_directory="$(
  mktemp -d "${LOG_DIRECTORY}/.acceptance-run.XXXXXX"
)"
output_fifo="${run_directory}/output.fifo"
mkfifo -- "${output_fifo}"

launch_pid=""
tee_pid=""

process_group_exists() {
  [[ -n "${launch_pid}" ]] \
    && kill -0 -- "-${launch_pid}" 2>/dev/null
}

launch_is_running() {
  local process_state

  if [[ -z "${launch_pid}" ]]; then
    return 1
  fi

  process_state="$(ps -o stat= -p "${launch_pid}" 2>/dev/null)"
  [[ -n "${process_state}" && "${process_state}" != Z* ]]
}

stop_launch_group() {
  local attempt

  if ! process_group_exists; then
    return
  fi

  kill -INT -- "-${launch_pid}" 2>/dev/null || true

  for attempt in $(seq 1 50); do
    if ! process_group_exists; then
      return
    fi
    sleep 0.1
  done

  kill -TERM -- "-${launch_pid}" 2>/dev/null || true

  for attempt in $(seq 1 20); do
    if ! process_group_exists; then
      return
    fi
    sleep 0.1
  done

  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
}

finish_output_capture() {
  if [[ -n "${launch_pid}" ]]; then
    wait "${launch_pid}" 2>/dev/null || true
  fi

  if [[ -n "${tee_pid}" ]]; then
    wait "${tee_pid}" 2>/dev/null || true
  fi

  rm -f -- "${output_fifo}"
  rmdir -- "${run_directory}" 2>/dev/null || true
}

handle_signal() {
  local signal_name="$1"
  local exit_code="$2"

  trap - INT TERM
  set +e
  printf 'Wrapper interrupted by %s.\n' \
    "${signal_name}" >>"${LOG_PATH}"
  stop_launch_group
  finish_output_capture
  print_summary "INTERRUPTED" "${exit_code}"
  exit "${exit_code}"
}

trap 'handle_signal INT 130' INT
trap 'handle_signal TERM 143' TERM

tee -a "${LOG_PATH}" <"${output_fifo}" &
tee_pid="$!"

setsid ros2 launch drone_bringup acceptance_test.launch.py \
  "$@" >"${output_fifo}" 2>&1 &
launch_pid="$!"

deadline=$((SECONDS + outer_timeout_sec))
outer_timeout_reached=0

while launch_is_running; do
  if ((SECONDS >= deadline)); then
    outer_timeout_reached=1
    stop_launch_group
    break
  fi
  sleep 0.2
done

set +e
wait "${launch_pid}"
launch_exit_code="$?"
set -e

residual_group_count=0
if process_group_exists; then
  residual_group_count=1
  stop_launch_group
fi

launch_pid=""

wait "${tee_pid}" 2>/dev/null || true
tee_pid=""

rm -f -- "${output_fifo}"
rmdir -- "${run_directory}" 2>/dev/null || true

pass_count="$(grep -c 'ACCEPTANCE TEST: PASS' "${LOG_PATH}" || true)"
fail_count="$(grep -c 'ACCEPTANCE TEST: FAIL' "${LOG_PATH}" || true)"
clean_exit_count="$(
  grep -c \
    'acceptance_test_node-.*process has finished cleanly' \
    "${LOG_PATH}" \
    || true
)"
abnormal_exit_count="$(
  grep -c \
    'acceptance_test_node-.*process has died' \
    "${LOG_PATH}" \
    || true
)"
discovery_time_count="$(
  grep -c 'Discovery time:' "${LOG_PATH}" || true
)"
convergence_time_count="$(
  grep -c 'Convergence time:' "${LOG_PATH}" || true
)"
position_metric_count="$(
  grep -c 'Position error:' "${LOG_PATH}" || true
)"
yaw_metric_count="$(
  grep -c 'Yaw error:' "${LOG_PATH}" || true
)"
linear_metric_count="$(
  grep -c 'Linear speed:' "${LOG_PATH}" || true
)"
angular_metric_count="$(
  grep -c 'Angular speed:' "${LOG_PATH}" || true
)"

result="FAIL"
shell_exit_code=1

if ((outer_timeout_reached == 1)); then
  result="OUTER_TIMEOUT"
elif [[ "${launch_exit_code}" -eq 0 \
  && "${pass_count}" -eq 1 \
  && "${fail_count}" -eq 0 \
  && "${clean_exit_count}" -ge 1 \
  && "${abnormal_exit_count}" -eq 0 \
  && "${residual_group_count}" -eq 0 \
  && "${discovery_time_count}" -ge 1 \
  && "${convergence_time_count}" -ge 1 \
  && "${position_metric_count}" -ge 1 \
  && "${yaw_metric_count}" -ge 1 \
  && "${linear_metric_count}" -ge 1 \
  && "${angular_metric_count}" -ge 1 ]]; then
  result="PASS"
  shell_exit_code=0
fi

{
  printf '\nWrapper checks\n'
  printf 'launch exit code: %s\n' "${launch_exit_code}"
  printf 'PASS record count: %s\n' "${pass_count}"
  printf 'FAIL record count: %s\n' "${fail_count}"
  printf 'clean acceptance exit count: %s\n' \
    "${clean_exit_count}"
  printf 'abnormal acceptance exit count: %s\n' \
    "${abnormal_exit_count}"
  printf 'outer timeout seconds: %s\n' \
    "${outer_timeout_sec}"
  printf 'outer timeout reached: %s\n' \
    "${outer_timeout_reached}"
  printf 'residual process group detected: %s\n' \
    "${residual_group_count}"
} >>"${LOG_PATH}"

print_summary "${result}" "${shell_exit_code}"
exit "${shell_exit_code}"
