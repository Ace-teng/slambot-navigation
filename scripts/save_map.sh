#!/usr/bin/env bash
# =============================================================================
# 保存当前 /map 到仓库 slam 包的地图目录（src/slam/maps/map_01.{pgm,yaml}）
# 这样后续可直接用仓库 navigation.launch.py（默认 map_01）做 AMCL 导航，
# 也方便拷到真车/本地做 AMCL 复用。
#
# 注意：保存时小车请保持静止；.gitignore 已忽略 src/slam/maps/*，不影响使用。
# =============================================================================

# ------------------------- 单一配置块 -------------------------
ROBOT_WS=${ROBOT_WS:-$HOME/ros2_ws}
OUT_DIR=${OUT_DIR:-"$ROBOT_WS/src/slam/maps"}   # 存到 slam 包 maps 目录
MAP_NAME=${MAP_NAME:-map_01}
# -------------------------------------------------------------

set -e

source /opt/ros/humble/setup.bash
if [ -f "$ROBOT_WS/install/setup.bash" ]; then
    source "$ROBOT_WS/install/setup.bash"
fi

mkdir -p "$OUT_DIR"

echo "==============================================================="
echo " 开始保存 /map -> $OUT_DIR/${MAP_NAME}.{pgm,yaml}"
echo " 请确保小车保持静止（2~3 秒内无地图更新）再执行。Ctrl-C 可中止。"
echo "==============================================================="

ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$MAP_NAME"

echo "已保存："
ls -la "$OUT_DIR/${MAP_NAME}".*
