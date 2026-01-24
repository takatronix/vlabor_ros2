#!/bin/bash
#
# VR ROS2 Teleop foreground launch with logs
# Quest3 + SO101 VR teleoperation
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_ROS2_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$(dirname "$(dirname "$VR_ROS2_DIR")")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

setup_ros2() {
    print_status "ROS2環境をセットアップ中..."

    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        print_error "ROS2 Humbleが見つかりません"
        exit 1
    fi

    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash"
    else
        print_error "ワークスペースがビルドされていません"
        exit 1
    fi

    # Avoid mixing Python 3.13 site-packages with ROS2 Python 3.10
    if [ -n "$PYTHONPATH" ]; then
        PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v 'python3.13' | paste -sd ':' -)"
    fi
    export PYTHONPATH="/home/takatronix/ros2_ws/install/vr_haptic_msgs/local/lib/python3.10/dist-packages:/home/takatronix/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
}

LOG_FILE="${LOG_FILE:-/tmp/vr_ros2_teleop.log}"

# Defaults (override via env)
LEFT_SERIAL_PORT="${LEFT_SERIAL_PORT:-/dev/ttyACM0}"
RIGHT_SERIAL_PORT="${RIGHT_SERIAL_PORT:-/dev/ttyACM1}"
# ROS2用キャリブレーション（so101_descriptionパッケージ内）
# 環境変数で上書き可能
LEFT_CALIB_PATH="${LEFT_CALIB_PATH:-}"
RIGHT_CALIB_PATH="${RIGHT_CALIB_PATH:-}"
DRIVER_BACKEND="${DRIVER_BACKEND:-feetech}"
AUTO_ENABLE="${AUTO_ENABLE:-false}"
ROS_IP="${ROS_IP:-0.0.0.0}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
AUTO_DETECT_IP="${AUTO_DETECT_IP:-true}"
LEFT_WEBUI_PORT="${LEFT_WEBUI_PORT:-8080}"
RIGHT_WEBUI_PORT="${RIGHT_WEBUI_PORT:-8081}"
FOXGLOVE="${FOXGLOVE:-true}"
PROFILE="${PROFILE:-vr_teleop_so101}"
LAUNCH_CONFIG="${LAUNCH_CONFIG:-}"

setup_ros2

print_status "ログ出力: ${LOG_FILE}"
print_status "WebUI (左アーム): http://localhost:${LEFT_WEBUI_PORT}/"
print_status "WebUI (右アーム): http://localhost:${RIGHT_WEBUI_PORT}/"
print_status "フォアグラウンドで起動します (Ctrl+Cで停止)"

# キャリブレーションパスが指定されていればlaunch引数に追加
CALIB_ARGS=""
if [ -n "${LEFT_CALIB_PATH}" ]; then
    CALIB_ARGS="${CALIB_ARGS} left_calibration_path:=${LEFT_CALIB_PATH}"
fi
if [ -n "${RIGHT_CALIB_PATH}" ]; then
    CALIB_ARGS="${CALIB_ARGS} right_calibration_path:=${RIGHT_CALIB_PATH}"
fi

CONFIG_ARG=""
if [ -n "${LAUNCH_CONFIG}" ]; then
    CONFIG_ARG="config:=${LAUNCH_CONFIG}"
fi

ros2 launch vlabor_launch vlabor.launch.py \
  profile:=${PROFILE} \
  ${CONFIG_ARG} \
  ros_ip:=${ROS_IP} \
  ros_tcp_port:=${ROS_TCP_PORT} \
  auto_detect_ip:=${AUTO_DETECT_IP} \
  left_serial_port:=${LEFT_SERIAL_PORT} \
  right_serial_port:=${RIGHT_SERIAL_PORT} \
  driver_backend:=${DRIVER_BACKEND} \
  auto_enable:=${AUTO_ENABLE} \
  left_webui_port:=${LEFT_WEBUI_PORT} \
  right_webui_port:=${RIGHT_WEBUI_PORT} \
  enable_foxglove:=${FOXGLOVE} \
  ${CALIB_ARGS} \
  2>&1 | tee "${LOG_FILE}"
