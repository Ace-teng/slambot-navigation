#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
colcon build --symlink-install --packages-select landerpi_gazebo_sim

echo "Built landerpi_gazebo_sim in ${WORKSPACE_DIR}"
