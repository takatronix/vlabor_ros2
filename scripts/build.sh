#!/bin/bash
#
# VR ROS2 パッケージビルドスクリプト
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_ROS2_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$(dirname "$(dirname "$VR_ROS2_DIR")")"

# 色付き出力
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

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  VR ROS2 ビルド${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# ROS2環境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    print_error "ROS2 Humbleが見つかりません"
    exit 1
fi

cd "$ROS2_WS"

# ビルド対象パッケージ
PACKAGES=(
    "so101_description"
    "so101_controller"
    "so101_hardware_interface"
    "so101_ros2_bridge"
    "so101_bringup"
    "unity_robot_control"
)

print_status "ビルド対象パッケージ:"
for pkg in "${PACKAGES[@]}"; do
    echo "  - $pkg"
done
echo ""

# ビルド実行
print_status "ビルド開始..."
colcon build --symlink-install --packages-select ${PACKAGES[@]}

if [ $? -eq 0 ]; then
    echo ""
    print_status "ビルド成功！"
    print_status "環境をソース: source $ROS2_WS/install/setup.bash"
else
    echo ""
    print_error "ビルド失敗"
    exit 1
fi
