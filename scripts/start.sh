#!/bin/bash
#
# VR ROS2 システム起動スクリプト
# Quest3 + SO101 VRテレオペレーション用
#
# アーキテクチャ:
#   Quest3 VR手 → Unity TCP → VR Control → IK Solver → SO101 Control Node → ロボット
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_ROS2_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$(dirname "$(dirname "$VR_ROS2_DIR")")"

# 色付き出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# PIDファイル
PID_FILE="/tmp/vr_ros2_pids.txt"

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  VR ROS2 Teleoperation System${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ROS2環境セットアップ
setup_ros2() {
    print_status "ROS2環境をセットアップ中..."

    # ROS2 Humbleをソース
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        print_error "ROS2 Humbleが見つかりません"
        exit 1
    fi

    # ワークスペースをソース
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash"
    else
        print_warning "ワークスペースがビルドされていません"
        print_status "ビルド実行: $SCRIPT_DIR/build.sh"
        exit 1
    fi

    # Python3.13 系の site-packages が混入すると ROS2(Python3.10) で型サポートが壊れるため除外
    if [ -n "$PYTHONPATH" ]; then
        PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v 'python3.13' | paste -sd ':' -)"
    fi
    export PYTHONPATH="/home/takatronix/ros2_ws/install/vr_haptic_msgs/local/lib/python3.10/dist-packages:/home/takatronix/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"

}

# ノード起動関数
start_node() {
    local name=$1
    local command=$2

    print_status "$name を起動中..."
    $command 2>&1 &
    local pid=$!
    echo "$pid:$name" >> "$PID_FILE"
    sleep 1

    if kill -0 $pid 2>/dev/null; then
        print_status "$name 起動完了 (PID: $pid)"
    else
        print_error "$name の起動に失敗しました"
    fi
}

# VRテレオペ起動（デフォルト）
start_vr_teleop() {
    print_header
    echo ""
    echo -e "${BLUE}モード: VRテレオペレーション (Quest3 → SO101)${NC}"
    echo ""

    # 既存プロセスの確認
    if [ -f "$PID_FILE" ] && [ -s "$PID_FILE" ]; then
        print_warning "既存のプロセスが実行中の可能性があります"
        print_warning "停止する場合は: $SCRIPT_DIR/stop.sh"
        read -p "続行しますか? (y/N): " confirm
        if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
            exit 0
        fi
    fi

    # PIDファイル初期化
    > "$PID_FILE"

    setup_ros2

    echo ""
    print_status "=== ノード起動開始 ==="
    echo ""

    # デフォルトパラメータ（環境変数で上書き可能）
    LEFT_SERIAL_PORT="${LEFT_SERIAL_PORT:-/dev/ttyACM0}"
    RIGHT_SERIAL_PORT="${RIGHT_SERIAL_PORT:-/dev/ttyACM1}"
    # ROS2用キャリブレーション（空ならlaunchファイルのデフォルトを使用）
    LEFT_CALIB_PATH="${LEFT_CALIB_PATH:-}"
    RIGHT_CALIB_PATH="${RIGHT_CALIB_PATH:-}"
    DRIVER_BACKEND="${DRIVER_BACKEND:-feetech}"
    AUTO_ENABLE="${AUTO_ENABLE:-false}"
    ROS_IP="${ROS_IP:-0.0.0.0}"
    ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
    AUTO_DETECT_IP="${AUTO_DETECT_IP:-true}"
    FOXGLOVE="${FOXGLOVE:-true}"

    # キャリブレーションパスが指定されていれば引数に追加
    CALIB_ARGS=""
    if [ -n "${LEFT_CALIB_PATH}" ]; then
        CALIB_ARGS="${CALIB_ARGS} left_calibration_path:=${LEFT_CALIB_PATH}"
    fi
    if [ -n "${RIGHT_CALIB_PATH}" ]; then
        CALIB_ARGS="${CALIB_ARGS} right_calibration_path:=${RIGHT_CALIB_PATH}"
    fi

    # 直結起動（Unity → IK → SO101）
    start_node "VR Teleop (direct launch)" \
        "ros2 launch unity_robot_control vr_dual_arm_teleop_direct.launch.py \
ros_ip:=${ROS_IP} \
ros_tcp_port:=${ROS_TCP_PORT} \
auto_detect_ip:=${AUTO_DETECT_IP} \
left_serial_port:=${LEFT_SERIAL_PORT} \
right_serial_port:=${RIGHT_SERIAL_PORT} \
driver_backend:=${DRIVER_BACKEND} \
auto_enable:=${AUTO_ENABLE} \
${CALIB_ARGS}"

    # WebUI起動（左アーム用）
    LEFT_WEBUI_PORT="${LEFT_WEBUI_PORT:-8080}"
    start_node "SO101 WebUI (left)" \
        "ros2 run unity_robot_control so101_webui_node --ros-args \
-r __node:=so101_webui_left \
-p web_port:=${LEFT_WEBUI_PORT} \
-p arm_namespaces:=[left_arm]"

    # WebUI起動（右アーム用）
    RIGHT_WEBUI_PORT="${RIGHT_WEBUI_PORT:-8081}"
    start_node "SO101 WebUI (right)" \
        "ros2 run unity_robot_control so101_webui_node --ros-args \
-r __node:=so101_webui_right \
-p web_port:=${RIGHT_WEBUI_PORT} \
-p arm_namespaces:=[right_arm]"

    # Foxglove Bridge（オプション）
    if [ "${FOXGLOVE}" = "true" ]; then
        if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
            start_node "Foxglove Bridge" \
                "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
        else
            print_warning "foxglove_bridge パッケージがインストールされていません"
        fi
    fi

    echo ""
    print_status "=== 起動完了 ==="
    echo ""
    print_status "起動したノード:"
    cat "$PID_FILE" | while read line; do
        pid=$(echo $line | cut -d: -f1)
        name=$(echo $line | cut -d: -f2)
        echo -e "  ${GREEN}●${NC} $name (PID: $pid)"
    done

    echo ""
    print_status "データフロー:"
    echo "  Quest3 → /quest/left_hand/pose,/quest/right_hand/pose"
    echo "       → /left_arm/target_pose,/right_arm/target_pose"
    echo "       → /left_arm/ik/joint_angles,/right_arm/ik/joint_angles"
    echo ""
    print_status "WebUI (左アーム): http://localhost:${LEFT_WEBUI_PORT}/"
    print_status "WebUI (右アーム): http://localhost:${RIGHT_WEBUI_PORT}/"
    print_status "トピック確認: ros2 topic list"
    print_status "状態確認: $SCRIPT_DIR/status.sh"
    print_status "停止: $SCRIPT_DIR/stop.sh"
}

# 使用方法
show_usage() {
    echo "使用方法: $0 [オプション]"
    echo ""
    echo "オプション:"
    echo "  (なし)          VRテレオペ起動（Quest3 → SO101）"
    echo "  --unity-only    Unity TCP + VR制御ノードのみ起動（デバッグ用）"
    echo "  --robot-only    SO101制御ノードのみ起動（デバッグ用）"
    echo "  -h, --help      ヘルプを表示"
    echo ""
    echo "環境変数で上書き可能:"
    echo "  LEFT_SERIAL_PORT, RIGHT_SERIAL_PORT, LEFT_CALIB_PATH, RIGHT_CALIB_PATH"
    echo "  DRIVER_BACKEND(mock|feetech), AUTO_ENABLE(true|false)"
    echo "  ROS_IP, ROS_TCP_PORT, AUTO_DETECT_IP"
    echo "  LEFT_WEBUI_PORT (default: 8080), RIGHT_WEBUI_PORT (default: 8081)"
    echo "  FOXGLOVE=true  Foxglove Bridge を起動"
}

# 引数処理
case "$1" in
    --unity-only)
        print_header
        > "$PID_FILE"
        setup_ros2
        echo ""
        echo -e "${BLUE}モード: Unity接続のみ（デバッグ）${NC}"
        echo ""
        start_node "Unity TCP Endpoint" "ros2 run ros_tcp_endpoint default_server_endpoint"
        start_node "VR Dual Arm Control" "ros2 run unity_robot_control vr_dual_arm_control_node"
        ;;
    --robot-only)
        print_header
        > "$PID_FILE"
        setup_ros2
        echo ""
        echo -e "${BLUE}モード: ロボットのみ（デバッグ）${NC}"
        echo ""
        LEFT_SERIAL_PORT="${LEFT_SERIAL_PORT:-/dev/ttyACM0}"
        RIGHT_SERIAL_PORT="${RIGHT_SERIAL_PORT:-/dev/ttyACM1}"
        # ROS2用キャリブレーション（so101_descriptionパッケージ内）
        SO101_DESC_SHARE="$(ros2 pkg prefix so101_description)/share/so101_description"
        LEFT_CALIB_PATH="${LEFT_CALIB_PATH:-${SO101_DESC_SHARE}/config/calibration/left_arm.json}"
        RIGHT_CALIB_PATH="${RIGHT_CALIB_PATH:-${SO101_DESC_SHARE}/config/calibration/right_arm.json}"
        DRIVER_BACKEND="${DRIVER_BACKEND:-feetech}"
        AUTO_ENABLE="${AUTO_ENABLE:-false}"

        start_node "SO101 Control (left_arm)" \
            "ros2 run unity_robot_control so101_control_node --ros-args \
 -r __ns:=/left_arm \
 -p serial_port:=${LEFT_SERIAL_PORT} \
 -p calibration_path:=${LEFT_CALIB_PATH} \
 -p driver_backend:=${DRIVER_BACKEND} \
 -p auto_enable:=${AUTO_ENABLE} \
 -p gripper_scale:=2.0 \
 -r joint_ctrl_single:=ik/joint_angles"

        start_node "SO101 Control (right_arm)" \
            "ros2 run unity_robot_control so101_control_node --ros-args \
 -r __ns:=/right_arm \
 -p serial_port:=${RIGHT_SERIAL_PORT} \
 -p calibration_path:=${RIGHT_CALIB_PATH} \
 -p driver_backend:=${DRIVER_BACKEND} \
 -p auto_enable:=${AUTO_ENABLE} \
 -p gripper_scale:=2.0 \
 -r joint_ctrl_single:=ik/joint_angles"
        ;;
    -h|--help)
        show_usage
        exit 0
        ;;
    *)
        start_vr_teleop
        ;;
esac

echo ""
print_status "Ctrl+C で終了"
wait
