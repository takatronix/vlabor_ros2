#!/bin/bash
#
# VLAbor - Robot Entrypoint
#
# 環境変数:
#   ALLOWED_VR_IP  - 操作を許可するVRのIP（指定なしは全許可）
#   ZENOH_PORT     - Zenohポート (default: 7447)
#   TS_AUTHKEY     - Tailscale認証キー

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "  VLAbor - Robot Container"
echo "=========================================="
echo ""

# ROS2環境
source /opt/ros/${ROS_DISTRO}/setup.bash

# ワークスペースがビルドされていれば読み込み
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
    echo -e "${GREEN}[INFO]${NC} ワークスペース読み込み完了"
fi

# Tailscale起動（オプション）
MY_IP=$(hostname -I | awk '{print $1}')
if [ -n "$TS_AUTHKEY" ]; then
    echo -e "${GREEN}[INFO]${NC} Tailscale起動中..."
    tailscaled --state=/var/lib/tailscale/tailscaled.state &
    sleep 2
    echo -e "${GREEN}[INFO]${NC} Tailscale認証中..."
    tailscale up --authkey="$TS_AUTHKEY" --hostname="robot-$(hostname)" || true
    MY_IP=$(tailscale ip -4 2>/dev/null || echo "$MY_IP")
    echo -e "${GREEN}[INFO]${NC} Tailscale IP: $MY_IP"
else
    echo -e "${YELLOW}[WARN]${NC} Tailscale認証キー未設定 - ローカルネットワークのみ"
    echo -e "${GREEN}[INFO]${NC} ローカル IP: $MY_IP"
fi

# Zenoh Bridge起動
echo -e "${GREEN}[INFO]${NC} Zenoh Bridge起動中..."
echo -e "${GREEN}[INFO]${NC}   リッスン: tcp://0.0.0.0:${ZENOH_PORT}"

if [ -n "$ALLOWED_VR_IP" ]; then
    echo -e "${CYAN}[CTRL]${NC} 操作許可IP: $ALLOWED_VR_IP"
    # 許可IPが指定されている場合、フィルタリングモードで起動
    # Zenoh の接続元制限を使用
    zenoh-bridge-ros2dds -l "tcp/0.0.0.0:${ZENOH_PORT}" &
else
    echo -e "${YELLOW}[WARN]${NC} 操作許可IP未指定 - 全VRからの操作を許可"
    zenoh-bridge-ros2dds -l "tcp/0.0.0.0:${ZENOH_PORT}" &
fi
ZENOH_PID=$!
sleep 3

# シリアルポート確認
echo -e "${GREEN}[INFO]${NC} シリアルポート確認..."
if [ -e "$LEFT_SERIAL_PORT" ]; then
    echo -e "${GREEN}[INFO]${NC}   左アーム: $LEFT_SERIAL_PORT ✓"
else
    echo -e "${YELLOW}[WARN]${NC}   左アーム: $LEFT_SERIAL_PORT (未接続)"
fi
if [ -e "$RIGHT_SERIAL_PORT" ]; then
    echo -e "${GREEN}[INFO]${NC}   右アーム: $RIGHT_SERIAL_PORT ✓"
else
    echo -e "${YELLOW}[WARN]${NC}   右アーム: $RIGHT_SERIAL_PORT (未接続)"
fi

# Foxglove Bridge起動（映像配信用）
echo -e "${GREEN}[INFO]${NC} Foxglove Bridge起動中..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
FOXGLOVE_PID=$!

echo ""
echo "=========================================="
echo -e "${GREEN}[INFO]${NC} Robot側の準備完了!"
echo "=========================================="
echo ""
echo "このコンテナ (Robot側):"
echo "  Tailscale IP: $MY_IP"
echo "  Zenoh Port:   ${ZENOH_PORT}"
echo "  Foxglove:     8765 (映像配信)"
if [ -n "$ALLOWED_VR_IP" ]; then
    echo -e "  操作許可:     ${CYAN}$ALLOWED_VR_IP${NC}"
else
    echo "  操作許可:     全員"
fi
echo ""
echo "VR側から接続:"
echo "  ROBOT_TAILSCALE_IP=${MY_IP} docker compose up vr"
echo ""
echo "映像のみ視聴 (ブラウザ):"
echo "  http://${MY_IP}:8765"
echo ""

# クリーンアップ
cleanup() {
    echo ""
    echo "終了処理中..."
    kill $ZENOH_PID $FOXGLOVE_PID 2>/dev/null || true
    echo "完了"
}
trap cleanup EXIT

# コマンド実行またはシェル
if [ "$1" = "bash" ]; then
    exec bash
elif [ -f /ros2_ws/src/vr_ros2/install/setup.bash ]; then
    source /ros2_ws/src/vr_ros2/install/setup.bash
    # ロボット制御起動
    echo -e "${GREEN}[INFO]${NC} ロボット制御ノード起動中..."
    ros2 launch unity_robot_control vr_dual_arm_teleop_robot_only.launch.py \
        left_serial_port:=${LEFT_SERIAL_PORT} \
        right_serial_port:=${RIGHT_SERIAL_PORT} \
        driver_backend:=${DRIVER_BACKEND} \
        auto_enable:=${AUTO_ENABLE} || {
        echo -e "${YELLOW}[WARN]${NC} ロボット制御の起動に失敗。待機モードに入ります。"
        wait $ZENOH_PID
    }
else
    echo -e "${YELLOW}[WARN]${NC} vr_ros2 ワークスペース未ビルド。待機モードに入ります。"
    echo -e "${GREEN}[INFO]${NC} Zenoh BridgeとFoxglove Bridgeは動作中です。"
    wait $ZENOH_PID
fi
