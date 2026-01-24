#!/bin/bash
#
# VR ROS2 システム状態確認スクリプト
#

# 色付き出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PID_FILE="/tmp/vr_ros2_pids.txt"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  VR ROS2 システム状態${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# ROS2環境確認
echo -e "${BLUE}[ROS2環境]${NC}"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "  ROS_DISTRO: ${GREEN}$ROS_DISTRO${NC}"
else
    echo -e "  ROS_DISTRO: ${RED}未設定${NC}"
fi

if [ -n "$ROS_DOMAIN_ID" ]; then
    echo -e "  ROS_DOMAIN_ID: ${GREEN}$ROS_DOMAIN_ID${NC}"
else
    echo -e "  ROS_DOMAIN_ID: ${YELLOW}0 (デフォルト)${NC}"
fi

echo ""

# 登録されたプロセス確認
echo -e "${BLUE}[登録プロセス]${NC}"
if [ -f "$PID_FILE" ]; then
    while read line; do
        pid=$(echo $line | cut -d: -f1)
        name=$(echo $line | cut -d: -f2)

        if kill -0 $pid 2>/dev/null; then
            echo -e "  ${GREEN}●${NC} $name (PID: $pid) - 実行中"
        else
            echo -e "  ${RED}○${NC} $name (PID: $pid) - 停止"
        fi
    done < "$PID_FILE"
else
    echo -e "  ${YELLOW}登録プロセスなし${NC}"
fi

echo ""

# ROS2ノード確認
echo -e "${BLUE}[ROS2ノード]${NC}"
source /opt/ros/humble/setup.bash 2>/dev/null

nodes=$(ros2 node list 2>/dev/null)
if [ -n "$nodes" ]; then
    echo "$nodes" | while read node; do
        echo -e "  ${GREEN}●${NC} $node"
    done
else
    echo -e "  ${YELLOW}アクティブなノードなし${NC}"
fi

echo ""

# トピック確認
echo -e "${BLUE}[主要トピック]${NC}"
topics=$(ros2 topic list 2>/dev/null | grep -E "(quest|left_arm|right_arm|target_pose|joint)" | head -20)
if [ -n "$topics" ]; then
    echo "$topics" | while read topic; do
        hz=$(timeout 2 ros2 topic hz "$topic" --window 5 2>/dev/null | grep "average rate" | head -1)
        if [ -n "$hz" ]; then
            echo -e "  ${GREEN}●${NC} $topic - $hz"
        else
            echo -e "  ${YELLOW}○${NC} $topic - データなし"
        fi
    done
else
    echo -e "  ${YELLOW}関連トピックなし${NC}"
fi

echo ""

# USBデバイス確認
echo -e "${BLUE}[USBデバイス (SO101)]${NC}"
for port in /dev/ttyACM*; do
    if [ -e "$port" ]; then
        echo -e "  ${GREEN}●${NC} $port"
    fi
done
for port in /dev/ttyUSB*; do
    if [ -e "$port" ]; then
        echo -e "  ${GREEN}●${NC} $port"
    fi
done

if [ ! -e /dev/ttyACM0 ] && [ ! -e /dev/ttyUSB0 ]; then
    echo -e "  ${RED}USBデバイスが見つかりません${NC}"
fi

echo ""
