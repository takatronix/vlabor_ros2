#!/bin/bash
#
# VLAbor + FluentVision Entrypoint
#

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "  VLAbor + FluentVision Container"
echo "=========================================="
echo ""

# ROS2環境
source /opt/ros/${ROS_DISTRO}/setup.bash

# ワークスペース読み込み
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
    echo -e "${GREEN}[INFO]${NC} ワークスペース読み込み完了"
fi

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

# Unity TCP Endpoint起動
echo -e "${GREEN}[INFO]${NC} Unity TCP Endpoint起動中..."
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
    -p ROS_IP:=0.0.0.0 \
    -p ROS_TCP_PORT:=${ROS_TCP_PORT} &
UNITY_PID=$!
sleep 2

# Foxglove Bridge起動
echo -e "${GREEN}[INFO]${NC} Foxglove Bridge起動中..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
FOXGLOVE_PID=$!

# カメラ起動 (オプション)
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
echo -e "${GREEN}[INFO]${NC} VLAbor + FluentVision 起動完了!"
echo "=========================================="
echo ""
echo "接続情報:"
echo "  Unity TCP:  0.0.0.0:${ROS_TCP_PORT}"
echo "  Foxglove:   0.0.0.0:8765"
echo "  カメラ:     ${CAMERA_DEVICE}"
echo ""
echo "Unity側の設定:"
echo "  ROS IP:     $(hostname -I | awk '{print $1}')"
echo "  ROS Port:   ${ROS_TCP_PORT}"
echo ""

# クリーンアップ
cleanup() {
    echo ""
    echo "終了処理中..."
    kill $UNITY_PID $FOXGLOVE_PID $CAMERA_PID 2>/dev/null || true
    echo "完了"
}
trap cleanup EXIT

# コマンド実行
if [ "$1" = "bash" ]; then
    exec bash
elif [ "$1" = "camera" ]; then
    # カメラのみ起動
    echo -e "${GREEN}[INFO]${NC} カメラモード"
    wait
else
    # ロボット制御起動
    if [ -f /ros2_ws/src/vr_ros2/install/setup.bash ]; then
        source /ros2_ws/src/vr_ros2/install/setup.bash
        echo -e "${GREEN}[INFO]${NC} ロボット制御ノード起動中..."
        ros2 launch unity_robot_control vr_dual_arm_teleop.launch.py \
            left_serial_port:=${LEFT_SERIAL_PORT} \
            right_serial_port:=${RIGHT_SERIAL_PORT} \
            driver_backend:=${DRIVER_BACKEND} \
            auto_enable:=${AUTO_ENABLE}
    else
        echo -e "${YELLOW}[WARN]${NC} vr_ros2 未マウント。待機モード..."
        wait
    fi
fi
