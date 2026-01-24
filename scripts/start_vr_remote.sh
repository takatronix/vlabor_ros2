#!/bin/bash
#
# VR Remote Server (リモートVR側)
#
# VRヘッドセット(Quest3)と接続し、Tailscale + Zenoh Bridge経由で
# 遠隔地のRobotと通信する
#
# 使い方:
#   ./start_vr_remote.sh                    # デフォルト設定で起動
#   ROBOT_TAILSCALE_IP=100.64.0.1 ./start_vr_remote.sh  # Robot側IP指定
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
ROBOT_TAILSCALE_IP="${ROBOT_TAILSCALE_IP:-}"          # Robot側のTailscale IP
ZENOH_PORT="${ZENOH_PORT:-7447}"                       # Zenoh Bridgeポート
ROS_IP="${ROS_IP:-0.0.0.0}"                           # Unity TCP Endpoint IP
ROS_TCP_PORT="${ROS_TCP_PORT:-42000}"                 # Unity TCP Endpoint Port

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
# Zenoh Bridge確認・起動
# ==============================================================================
start_zenoh_bridge() {
    print_status "Zenoh Bridgeを起動中..."

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

    if [ -z "$ROBOT_TAILSCALE_IP" ]; then
        print_error "ROBOT_TAILSCALE_IP が設定されていません"
        print_status "使い方: ROBOT_TAILSCALE_IP=100.64.0.1 $0"
        print_status ""
        print_status "接続可能なマシン一覧:"
        tailscale status
        exit 1
    fi

    print_status "Robot側 (${ROBOT_TAILSCALE_IP}:${ZENOH_PORT}) に接続中..."

    # Zenoh Bridge起動（Robot側に接続）
    $ZENOH_BRIDGE -e "tcp/${ROBOT_TAILSCALE_IP}:${ZENOH_PORT}" \
        2>&1 | tee /tmp/zenoh_bridge_vr.log &
    ZENOH_PID=$!

    sleep 2

    if ! kill -0 $ZENOH_PID 2>/dev/null; then
        print_error "Zenoh Bridgeの起動に失敗しました"
        cat /tmp/zenoh_bridge_vr.log
        exit 1
    fi

    print_status "Zenoh Bridge起動完了 (PID: ${ZENOH_PID})"
}

# ==============================================================================
# Unity TCP Endpoint起動
# ==============================================================================
start_unity_endpoint() {
    print_status "Unity TCP Endpoint を起動中..."
    print_status "  IP: ${ROS_IP}"
    print_status "  Port: ${ROS_TCP_PORT}"

    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
        -p ROS_IP:="${ROS_IP}" \
        -p ROS_TCP_PORT:=${ROS_TCP_PORT} \
        2>&1 | tee /tmp/unity_tcp_endpoint.log &
    UNITY_PID=$!

    sleep 1
    print_status "Unity TCP Endpoint起動完了 (PID: ${UNITY_PID})"
}

# ==============================================================================
# メイン
# ==============================================================================
main() {
    echo ""
    echo "=============================================="
    echo "  VR Remote Server (リモートVR側)"
    echo "=============================================="
    echo ""

    setup_ros2
    check_tailscale
    start_zenoh_bridge
    start_unity_endpoint

    echo ""
    print_status "=========================================="
    print_status "VR Remote Server 起動完了!"
    print_status "=========================================="
    print_status ""
    print_status "このマシン (VR側):"
    print_status "  Tailscale IP: ${MY_TAILSCALE_IP}"
    print_status "  Unity TCP:    ${ROS_IP}:${ROS_TCP_PORT}"
    print_status ""
    print_status "接続先 (Robot側):"
    print_status "  Tailscale IP: ${ROBOT_TAILSCALE_IP}"
    print_status "  Zenoh Port:   ${ZENOH_PORT}"
    print_status ""
    print_status "Quest3のUnityアプリで接続してください"
    print_status "Ctrl+Cで停止"
    echo ""

    # 終了時のクリーンアップ
    cleanup() {
        print_status "終了処理中..."
        kill $ZENOH_PID 2>/dev/null || true
        kill $UNITY_PID 2>/dev/null || true
        print_status "終了しました"
    }
    trap cleanup EXIT

    # 待機
    wait
}

main "$@"
