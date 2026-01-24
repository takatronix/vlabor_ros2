#!/bin/bash
#
# Robot Remote Server (現場Robot側)
#
# 実機ロボット(SO101)を制御し、Tailscale + Zenoh Bridge経由で
# 遠隔地のVRと通信する
#
# 使い方:
#   ./start_robot_remote.sh                              # デフォルト設定で起動
#   LEFT_SERIAL_PORT=/dev/ttyACM0 ./start_robot_remote.sh  # シリアルポート指定
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_ROS2_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$(dirname "$(dirname "$VR_ROS2_DIR")")"
CONFIG_DIR="${VR_ROS2_DIR}/unity_robot_control/config"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn()   { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error()  { echo -e "${RED}[ERROR]${NC} $1"; }

# ==============================================================================
# 設定
# ==============================================================================
ZENOH_PORT="${ZENOH_PORT:-7447}"                       # Zenoh Bridgeポート

# Robot設定
LEFT_SERIAL_PORT="${LEFT_SERIAL_PORT:-/dev/ttyACM0}"
RIGHT_SERIAL_PORT="${RIGHT_SERIAL_PORT:-/dev/ttyACM1}"
LEFT_CALIB_PATH="${LEFT_CALIB_PATH:-}"
RIGHT_CALIB_PATH="${RIGHT_CALIB_PATH:-}"
DRIVER_BACKEND="${DRIVER_BACKEND:-feetech}"
AUTO_ENABLE="${AUTO_ENABLE:-false}"

# WebUI
LEFT_WEBUI_PORT="${LEFT_WEBUI_PORT:-8080}"
RIGHT_WEBUI_PORT="${RIGHT_WEBUI_PORT:-8081}"
FOXGLOVE="${FOXGLOVE:-true}"
PROFILE="${PROFILE:-dual_teleop_so101}"
LAUNCH_CONFIG="${LAUNCH_CONFIG:-}"

# ==============================================================================
# ROS2環境セットアップ
# ==============================================================================
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
        print_error "ワークスペースがビルドされていません: colcon build を実行してください"
        exit 1
    fi

    # Python path fix
    if [ -n "$PYTHONPATH" ]; then
        PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v 'python3.13' | paste -sd ':' -)"
    fi
    export PYTHONPATH="/home/takatronix/ros2_ws/install/vr_haptic_msgs/local/lib/python3.10/dist-packages:/home/takatronix/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
}

# ==============================================================================
# Tailscale確認
# ==============================================================================
check_tailscale() {
    print_status "Tailscale接続を確認中..."

    if ! command -v tailscale &> /dev/null; then
        print_error "Tailscaleがインストールされていません"
        print_status "インストール: curl -fsSL https://tailscale.com/install.sh | sh"
        exit 1
    fi

    if ! tailscale status &> /dev/null; then
        print_warn "Tailscaleが接続されていません"

        # OAuth認証を試みる
        if [ -f "${CONFIG_DIR}/tailscale_oauth.env" ]; then
            print_status "OAuth認証を試行中..."
            source "${CONFIG_DIR}/tailscale_oauth.env"
            sudo tailscale up --authkey="${TS_AUTHKEY}"
        else
            print_status "手動でログインしてください:"
            sudo tailscale up
        fi
    fi

    MY_TAILSCALE_IP=$(tailscale ip -4 2>/dev/null || echo "")
    if [ -z "$MY_TAILSCALE_IP" ]; then
        print_error "Tailscale IPを取得できません"
        exit 1
    fi

    print_status "Tailscale IP: ${MY_TAILSCALE_IP}"
}

# ==============================================================================
# Zenoh Bridge起動（サーバーモード）
# ==============================================================================
start_zenoh_bridge() {
    print_status "Zenoh Bridge (サーバーモード) を起動中..."

    # Zenoh Bridgeのパス
    ZENOH_BRIDGE=""
    if command -v zenoh-bridge-ros2dds &> /dev/null; then
        ZENOH_BRIDGE="zenoh-bridge-ros2dds"
    elif [ -f "$HOME/bin/zenoh-bridge-ros2dds" ]; then
        ZENOH_BRIDGE="$HOME/bin/zenoh-bridge-ros2dds"
    elif [ -f "/usr/local/bin/zenoh-bridge-ros2dds" ]; then
        ZENOH_BRIDGE="/usr/local/bin/zenoh-bridge-ros2dds"
    elif [ -f "$HOME/zenoh-bridge-ros2dds" ]; then
        ZENOH_BRIDGE="$HOME/zenoh-bridge-ros2dds"
    else
        print_error "zenoh-bridge-ros2dds が見つかりません"
        print_status "ダウンロード: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases"
        exit 1
    fi

    # Zenoh Bridge起動（リッスンモード）
    $ZENOH_BRIDGE -l "tcp/0.0.0.0:${ZENOH_PORT}" \
        2>&1 | tee /tmp/zenoh_bridge_robot.log &
    ZENOH_PID=$!

    sleep 2

    if ! kill -0 $ZENOH_PID 2>/dev/null; then
        print_error "Zenoh Bridgeの起動に失敗しました"
        cat /tmp/zenoh_bridge_robot.log
        exit 1
    fi

    print_status "Zenoh Bridge起動完了 (PID: ${ZENOH_PID})"
    print_status "リッスン中: tcp://0.0.0.0:${ZENOH_PORT}"
}

# ==============================================================================
# メイン
# ==============================================================================
main() {
    echo ""
    echo "=============================================="
    echo "  Robot Remote Server (現場Robot側)"
    echo "=============================================="
    echo ""

    setup_ros2
    check_tailscale
    start_zenoh_bridge
    echo ""
    print_status "=========================================="
    print_status "Robot側の準備完了!"
    print_status "=========================================="
    print_status ""
    print_status "このマシン (Robot側):"
    print_status "  Tailscale IP: ${MY_TAILSCALE_IP}"
    print_status "  Zenoh Port:   ${ZENOH_PORT}"
    print_status ""
    print_status "VR側から接続するには:"
    print_status "  ROBOT_TAILSCALE_IP=${MY_TAILSCALE_IP} ./start_vr_remote.sh"
    print_status ""
    print_status "ロボット制御を開始中..."
    echo ""

    # 終了時のクリーンアップ
    cleanup() {
        print_status "終了処理中..."
        kill $ZENOH_PID 2>/dev/null || true
        print_status "終了しました"
    }
    trap cleanup EXIT

    # キャリブレーション引数
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

    # メインのテレオペ起動（Unity TCP Endpointなし版）
    # Note: VR側からZenoh経由でトピックが届く
    ros2 launch vlabor_launch vlabor.launch.py \
        profile:=${PROFILE} \
        ${CONFIG_ARG} \
        left_serial_port:=${LEFT_SERIAL_PORT} \
        right_serial_port:=${RIGHT_SERIAL_PORT} \
        driver_backend:=${DRIVER_BACKEND} \
        auto_enable:=${AUTO_ENABLE} \
        left_webui_port:=${LEFT_WEBUI_PORT} \
        right_webui_port:=${RIGHT_WEBUI_PORT} \
        enable_foxglove:=${FOXGLOVE} \
        ${CALIB_ARGS} \
        2>&1 | tee /tmp/robot_teleop.log
}

main "$@"
