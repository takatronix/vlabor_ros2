#!/bin/bash
#
# VR Remote Entrypoint
#

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "  VLAbor - VR Container"
echo "=========================================="
echo ""

# ROS2環境
source /opt/ros/${ROS_DISTRO}/setup.bash

# ワークスペース（ros_tcp_endpoint）
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Tailscale起動（オプション）
MY_IP=$(hostname -I | awk '{print $1}')
if [ -n "$TS_AUTHKEY" ]; then
    echo -e "${GREEN}[INFO]${NC} Tailscale起動中..."
    tailscaled --state=/var/lib/tailscale/tailscaled.state &
    sleep 2
    echo -e "${GREEN}[INFO]${NC} Tailscale認証中..."
    tailscale up --authkey="$TS_AUTHKEY" --hostname="vr-remote-$(hostname)" || true
    MY_IP=$(tailscale ip -4 2>/dev/null || echo "$MY_IP")
    echo -e "${GREEN}[INFO]${NC} Tailscale IP: $MY_IP"
else
    echo -e "${GREEN}[INFO]${NC} Tailscale未使用 - ローカルネットワークモード"
    echo -e "${GREEN}[INFO]${NC} ローカル IP: $MY_IP"
fi

# ROBOT_TAILSCALE_IP チェック
if [ -z "$ROBOT_TAILSCALE_IP" ]; then
    echo -e "${RED}[ERROR]${NC} ROBOT_TAILSCALE_IP が設定されていません"
    echo ""
    echo "使い方:"
    echo "  docker run -e ROBOT_TAILSCALE_IP=100.64.0.1 ..."
    echo "  または -e ROBOT_TAILSCALE_IP=192.168.1.xxx（ローカルIP）"
    echo ""
    # Tailscale使用時のみステータス表示
    if [ -n "$TS_AUTHKEY" ]; then
        echo "接続可能なマシン:"
        tailscale status 2>/dev/null || true
    fi
    echo -e "${GREEN}[INFO]${NC} 待機モードで起動します..."
    ROBOT_TAILSCALE_IP="127.0.0.1"
fi

# Zenoh Bridge起動（クライアントモード）
echo -e "${GREEN}[INFO]${NC} Zenoh Bridge起動中..."
echo -e "${GREEN}[INFO]${NC}   接続先: tcp://${ROBOT_TAILSCALE_IP}:${ZENOH_PORT}"
# client モードで接続（リッスンしない）
zenoh-bridge-ros2dds client -e "tcp/${ROBOT_TAILSCALE_IP}:${ZENOH_PORT}" &
ZENOH_PID=$!
sleep 3

# Unity TCP Endpoint起動
echo -e "${GREEN}[INFO]${NC} Unity TCP Endpoint起動中..."
echo -e "${GREEN}[INFO]${NC}   ポート: ${ROS_TCP_PORT}"
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
    -p ROS_IP:=0.0.0.0 \
    -p ROS_TCP_PORT:=${ROS_TCP_PORT} &
UNITY_PID=$!

echo ""
echo "=========================================="
echo -e "${GREEN}[INFO]${NC} VR Remote 起動完了!"
echo "=========================================="
echo ""
echo "このコンテナ (VR側):"
echo "  Tailscale IP: $MY_IP"
echo "  Unity TCP:    0.0.0.0:${ROS_TCP_PORT}"
echo ""
echo "接続先 (Robot側):"
echo "  Tailscale IP: ${ROBOT_TAILSCALE_IP}"
echo "  Zenoh Port:   ${ZENOH_PORT}"
echo ""

# クリーンアップ
cleanup() {
    echo "終了処理中..."
    kill $ZENOH_PID $UNITY_PID 2>/dev/null || true
}
trap cleanup EXIT

# コマンド実行またはシェル
if [ "$1" = "bash" ]; then
    exec bash
else
    wait
fi
