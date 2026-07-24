#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
timestamp="$(date '+%Y%m%d-%H%M%S')"
output_directory="${REPO_ROOT}/output/showcase_experiments/${timestamp}"
scenarios=(default narrow random_seed_42 multi_segment)
logs=()

for scenario in "${scenarios[@]}"; do
  "${SCRIPT_DIR}/run_showcase_acceptance.sh" "scenario:=${scenario}"
  log_file="$(
    find "${REPO_ROOT}/log/showcase_acceptance" \
      -maxdepth 1 -type f -name "${scenario}-*.log" \
      -printf '%T@ %p\n' \
      | sort -nr \
      | head -n 1 \
      | cut -d' ' -f2-
  )"
  if [[ -z "${log_file}" ]]; then
    echo "No log found for ${scenario}" >&2
    exit 1
  fi
  logs+=("${log_file}")
done

python3 "${SCRIPT_DIR}/export_showcase_results.py" \
  --output "${output_directory}" \
  "${logs[@]}"
echo "Experiment artifacts: ${output_directory}"
