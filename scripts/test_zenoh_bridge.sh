#!/bin/bash
#
# Zenoh Bridge 簡易テスト
#

ZENOH_BRIDGE="$HOME/bin/zenoh-bridge-ros2dds"
ZENOH_PORT="${ZENOH_PORT:-7447}"

echo "=== Zenoh Bridge テスト ==="
echo ""

# Tailscale確認
echo "1. Tailscale確認..."
if tailscale status &>/dev/null; then
    MY_IP=$(tailscale ip -4)
    echo "   ✓ Tailscale接続済み: $MY_IP"
else
    echo "   ✗ Tailscale未接続"
    exit 1
fi

# ROS2環境
echo ""
echo "2. ROS2環境セットアップ..."
source /opt/ros/humble/setup.bash
echo "   ✓ ROS2 Humble"

# Zenoh Bridge起動
echo ""
echo "3. Zenoh Bridge起動..."
$ZENOH_BRIDGE -l "tcp/0.0.0.0:${ZENOH_PORT}" &
ZENOH_PID=$!
sleep 3

if kill -0 $ZENOH_PID 2>/dev/null; then
    echo "   ✓ Zenoh Bridge起動成功 (PID: $ZENOH_PID)"
    echo ""
    echo "=== 接続情報 ==="
    echo "Tailscale IP: $MY_IP"
    echo "Zenoh Port:   $ZENOH_PORT"
    echo ""
    echo "他のマシンから接続するには:"
    echo "  zenoh-bridge-ros2dds -e tcp://${MY_IP}:${ZENOH_PORT}"
    echo ""
    echo "ROS2トピックを確認:"
    echo "  ros2 topic list"
    echo ""
    echo "Ctrl+C で停止"

    trap "kill $ZENOH_PID 2>/dev/null; echo '停止しました'" EXIT
    wait $ZENOH_PID
else
    echo "   ✗ Zenoh Bridge起動失敗"
    exit 1
fi
