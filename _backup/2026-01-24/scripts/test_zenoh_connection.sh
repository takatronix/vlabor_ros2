#!/bin/bash
#
# Zenoh Bridge 接続テスト
# 同じマシンで Robot側 と VR側 の接続を検証
#

ZENOH_BRIDGE="$HOME/bin/zenoh-bridge-ros2dds"

echo "=========================================="
echo "  Zenoh Bridge 接続テスト"
echo "=========================================="
echo ""

# ROS2環境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 既存プロセス停止
pkill -f "zenoh-bridge-ros2dds" 2>/dev/null || true
sleep 1

echo "[1/4] Robot側 Zenoh Bridge 起動中 (Domain 0, Port 7447)..."
$ZENOH_BRIDGE -l tcp/127.0.0.1:7447 --domain 0 2>&1 | sed 's/^/[ROBOT] /' &
ROBOT_PID=$!
sleep 3

if ! kill -0 $ROBOT_PID 2>/dev/null; then
    echo "[ERROR] Robot側 Zenoh Bridge 起動失敗"
    exit 1
fi
echo "[OK] Robot側 起動完了 (PID: $ROBOT_PID)"
echo ""

echo "[2/4] VR側 Zenoh Bridge 起動中 (Domain 1, 接続先 127.0.0.1:7447)..."
# client モード（自分でlistenしない、接続のみ）
$ZENOH_BRIDGE client -e tcp/127.0.0.1:7447 --domain 1 2>&1 | sed 's/^/[VR] /' &
VR_PID=$!
sleep 3

if ! kill -0 $VR_PID 2>/dev/null; then
    echo "[ERROR] VR側 Zenoh Bridge 起動失敗"
    kill $ROBOT_PID 2>/dev/null
    exit 1
fi
echo "[OK] VR側 起動完了 (PID: $VR_PID)"
echo ""

echo "[3/4] Robot側 (Domain 0) でテストトピックをPublish..."
ROS_DOMAIN_ID=0 ros2 topic pub -1 /test_from_robot std_msgs/String "data: 'Hello from Robot side!'" &
PUB_PID=$!
sleep 2

echo ""
echo "[4/4] VR側 (Domain 1) でトピックを確認..."
echo ""
echo "=== VR側 (Domain 1) で見えるトピック ==="
ROS_DOMAIN_ID=1 ros2 topic list 2>/dev/null | head -20

echo ""
echo "=== Robot側から送信されたトピックを受信 ==="
ROS_DOMAIN_ID=1 timeout 5 ros2 topic echo /test_from_robot --once 2>/dev/null && echo "" && echo "[SUCCESS] トピックがZenoh経由で届いた！" || echo "[INFO] タイムアウト（トピック発見待ち）"

echo ""
echo "=========================================="
echo "  接続テスト完了"
echo "=========================================="
echo ""
echo "構成:"
echo "  Robot側 (Domain 0) → Zenoh Bridge (server, :7447)"
echo "                            ↑"
echo "  VR側 (Domain 1)   → Zenoh Bridge (client, →:7447)"
echo ""
echo "Ctrl+C で停止"

# クリーンアップ
cleanup() {
    echo ""
    echo "終了処理中..."
    kill $ROBOT_PID $VR_PID $PUB_PID 2>/dev/null || true
    echo "完了"
}
trap cleanup EXIT

# 待機
wait $ROBOT_PID $VR_PID 2>/dev/null
