#!/bin/bash
#
# VLAbor Robot + FluentVision Entrypoint
#
# リモート版: Tailscale + Zenoh + FluentVision (カメラ、RealSense)
#
# 環境変数:
#   ALLOWED_VR_IP  - 操作を許可するVRのIP（指定なしは全許可）
#   ZENOH_PORT     - Zenohポート (default: 7447)
#   TS_AUTHKEY     - Tailscale認証キー
#   ENABLE_CAMERA  - カメラを有効化 (default: true)
#   CAMERA_DEVICE  - カメラデバイス (default: /dev/video0)

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "  VLAbor Robot + FluentVision Container"
echo "=========================================="
echo ""

# ROS2環境
source /opt/ros/${ROS_DISTRO}/setup.bash

# ワークスペースがビルドされていれば読み込み
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
    echo -e "${GREEN}[INFO]${NC} ワークスペース読み込み完了"
fi

# Tailscale起動
echo -e "${GREEN}[INFO]${NC} Tailscale起動中..."
tailscaled --state=/var/lib/tailscale/tailscaled.state &
sleep 2

if [ -n "$TS_AUTHKEY" ]; then
    echo -e "${GREEN}[INFO]${NC} Tailscale認証中..."
    tailscale up --authkey="$TS_AUTHKEY" --hostname="robot-$(hostname)"
else
    echo -e "${GREEN}[INFO]${NC} Tailscale手動ログイン..."
    tailscale up --hostname="robot-$(hostname)"
fi

MY_IP=$(tailscale ip -4 2>/dev/null || echo "unknown")
echo -e "${GREEN}[INFO]${NC} Tailscale IP: $MY_IP"

# カメラデバイス確認
echo -e "${GREEN}[INFO]${NC} カメラデバイス確認..."
if [ -e "$CAMERA_DEVICE" ]; then
    echo -e "${GREEN}[INFO]${NC}   カメラ: $CAMERA_DEVICE ✓"
else
    echo -e "${YELLOW}[WARN]${NC}   カメラ: $CAMERA_DEVICE (未接続)"
fi

# RealSense確認
if lsusb | grep -q "Intel.*RealSense"; then
    echo -e "${GREEN}[INFO]${NC}   RealSense: 検出 ✓"
else
    echo -e "${YELLOW}[WARN]${NC}   RealSense: 未検出"
fi

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

# Zenoh Bridge起動
echo -e "${GREEN}[INFO]${NC} Zenoh Bridge起動中..."
echo -e "${GREEN}[INFO]${NC}   リッスン: tcp://0.0.0.0:${ZENOH_PORT}"

if [ -n "$ALLOWED_VR_IP" ]; then
    echo -e "${CYAN}[CTRL]${NC} 操作許可IP: $ALLOWED_VR_IP"
    zenoh-bridge-ros2dds -l "tcp/0.0.0.0:${ZENOH_PORT}" &
else
    echo -e "${YELLOW}[WARN]${NC} 操作許可IP未指定 - 全VRからの操作を許可"
    zenoh-bridge-ros2dds -l "tcp/0.0.0.0:${ZENOH_PORT}" &
fi
ZENOH_PID=$!
sleep 3

# Foxglove Bridge起動（映像配信用）
echo -e "${GREEN}[INFO]${NC} Foxglove Bridge起動中..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
FOXGLOVE_PID=$!

# カメラ起動 (オプション)
CAMERA_PID=""
if [ "$ENABLE_CAMERA" = "true" ] && [ -e "$CAMERA_DEVICE" ]; then
    echo -e "${GREEN}[INFO]${NC} カメラノード起動中..."
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p video_device:=$CAMERA_DEVICE \
        -p framerate:=30.0 \
        -p image_width:=640 \
        -p image_height:=480 &
    CAMERA_PID=$!
fi

echo ""
echo "=========================================="
echo -e "${GREEN}[INFO]${NC} Robot + FluentVision 起動完了!"
echo "=========================================="
echo ""
echo "このコンテナ (Robot側):"
echo "  Tailscale IP: $MY_IP"
echo "  Zenoh Port:   ${ZENOH_PORT}"
echo "  Foxglove:     8765 (映像配信)"
if [ "$ENABLE_CAMERA" = "true" ]; then
    echo "  カメラ:       ${CAMERA_DEVICE}"
fi
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
    kill $ZENOH_PID $FOXGLOVE_PID $CAMERA_PID 2>/dev/null || true
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
        auto_enable:=${AUTO_ENABLE}
else
    echo -e "${YELLOW}[WARN]${NC} vr_ros2 未マウント。待機モード..."
    wait
fi
