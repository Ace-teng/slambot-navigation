#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MODE="${1:-demo}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Software rendering avoids WSLg black screens on affected Windows GPU drivers.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"

case "${MODE}" in
  demo)
    exec ros2 launch landerpi_gazebo_sim simulation.launch.py \
      slam:=false nav2:=true demo:=true rviz:=true gui:=false
    ;;
  navigation)
    exec ros2 launch landerpi_gazebo_sim simulation.launch.py \
      slam:=false nav2:=true demo:=false rviz:=true gui:=false
    ;;
  mapping)
    exec ros2 launch landerpi_gazebo_sim simulation.launch.py \
      slam:=true nav2:=false demo:=false rviz:=true gui:=false
    ;;
  *)
    echo "Usage: $0 [demo|navigation|mapping]" >&2
    exit 2
    ;;
esac
