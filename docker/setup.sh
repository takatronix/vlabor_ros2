#!/bin/bash
#
# VLAbor Docker セットアップスクリプト
#
# 使い方:
#   ./setup.sh              # 基本セットアップ
#   ./setup.sh --realsense  # RealSense対応を含む
#   ./setup.sh --build      # イメージをビルド
#   ./setup.sh --all        # 全てのセットアップ
#

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "=========================================="
echo "  VLAbor Docker セットアップ"
echo "=========================================="
echo ""

# Parse arguments
INSTALL_REALSENSE=false
BUILD_IMAGES=false

for arg in "$@"; do
    case $arg in
        --realsense)
            INSTALL_REALSENSE=true
            ;;
        --build)
            BUILD_IMAGES=true
            ;;
        --all)
            INSTALL_REALSENSE=true
            BUILD_IMAGES=true
            ;;
        --help|-h)
            echo "使い方: ./setup.sh [オプション]"
            echo ""
            echo "オプション:"
            echo "  --realsense  RealSenseカーネルモジュールをインストール"
            echo "  --build      Dockerイメージをビルド"
            echo "  --all        全てのセットアップを実行"
            echo "  --help       このヘルプを表示"
            exit 0
            ;;
    esac
done

# Check Docker
echo -e "${GREEN}[1/5]${NC} Docker確認..."
if ! command -v docker &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} Dockerがインストールされていません"
    echo ""
    echo "Dockerをインストールしてください:"
    echo "  curl -fsSL https://get.docker.com | sh"
    echo "  sudo usermod -aG docker \$USER"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo -e "${YELLOW}[WARN]${NC} Dockerデーモンが起動していないか、権限がありません"
    echo ""
    echo "以下を試してください:"
    echo "  sudo systemctl start docker"
    echo "  sudo usermod -aG docker \$USER && newgrp docker"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} Docker: $(docker --version | cut -d' ' -f3)"

# Check Docker Compose
echo -e "${GREEN}[2/5]${NC} Docker Compose確認..."
if ! docker compose version &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} Docker Composeがインストールされていません"
    exit 1
fi
echo -e "${GREEN}[OK]${NC} Docker Compose: $(docker compose version --short)"

# Check serial port permissions
echo -e "${GREEN}[3/5]${NC} シリアルポート権限確認..."
if groups | grep -q dialout; then
    echo -e "${GREEN}[OK]${NC} dialoutグループに所属しています"
else
    echo -e "${YELLOW}[WARN]${NC} dialoutグループに所属していません"
    echo ""
    read -p "dialoutグループに追加しますか？ [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo usermod -aG dialout $USER
        echo -e "${GREEN}[OK]${NC} dialoutグループに追加しました（再ログイン後に有効）"
    fi
fi

# Install RealSense drivers
echo -e "${GREEN}[4/5]${NC} RealSense確認..."
if $INSTALL_REALSENSE; then
    if command -v rs-enumerate-devices &> /dev/null; then
        echo -e "${GREEN}[OK]${NC} RealSense SDKは既にインストールされています"
    else
        echo -e "${CYAN}[INFO]${NC} RealSenseカーネルモジュールをインストールしています..."

        # Add Intel repository
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

        # Install
        sudo apt-get update
        sudo apt-get install -y librealsense2-dkms librealsense2-utils

        echo -e "${GREEN}[OK]${NC} RealSenseをインストールしました"
    fi
else
    if command -v rs-enumerate-devices &> /dev/null; then
        echo -e "${GREEN}[OK]${NC} RealSense SDK検出"
    else
        echo -e "${YELLOW}[SKIP]${NC} RealSenseは未インストール（--realsenseオプションで追加可能）"
    fi
fi

# Create .env file
echo -e "${GREEN}[5/5]${NC} 設定ファイル確認..."
if [ ! -f "$SCRIPT_DIR/.env" ]; then
    if [ -f "$SCRIPT_DIR/.env.example" ]; then
        cp "$SCRIPT_DIR/.env.example" "$SCRIPT_DIR/.env"
        echo -e "${GREEN}[OK]${NC} .envファイルを作成しました"
    fi
else
    echo -e "${GREEN}[OK]${NC} .envファイルは既に存在します"
fi

# Build images
if $BUILD_IMAGES; then
    echo ""
    echo -e "${CYAN}[INFO]${NC} Dockerイメージをビルドしています..."

    cd "$SCRIPT_DIR"

    echo -e "${CYAN}[BUILD]${NC} vlabor-local..."
    docker compose build local

    echo -e "${CYAN}[BUILD]${NC} vlabor-robot-fluent..."
    docker compose build robot-fluent

    echo -e "${GREEN}[OK]${NC} ビルド完了"
fi

# Summary
echo ""
echo "=========================================="
echo -e "${GREEN}[完了]${NC} セットアップが完了しました！"
echo "=========================================="
echo ""
echo "次のステップ:"
echo ""
echo "  1. ローカル版を起動:"
echo "     docker compose up local"
echo ""
echo "  2. リモート版（Robot側）を起動:"
echo "     export TS_AUTHKEY=tskey-auth-xxxxx"
echo "     docker compose up robot-fluent"
echo ""
echo "  3. Unity側の設定:"
echo "     ROS IP: $(hostname -I | awk '{print $1}')"
echo "     ROS Port: 42000"
echo ""

# Check devices
echo "検出されたデバイス:"
if ls /dev/ttyACM* 2>/dev/null; then
    echo -e "  シリアル: ${GREEN}$(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')${NC}"
else
    echo -e "  シリアル: ${YELLOW}未検出${NC}"
fi

if ls /dev/video* 2>/dev/null; then
    echo -e "  カメラ: ${GREEN}$(ls /dev/video* 2>/dev/null | head -3 | tr '\n' ' ')${NC}"
else
    echo -e "  カメラ: ${YELLOW}未検出${NC}"
fi

if command -v rs-enumerate-devices &> /dev/null && rs-enumerate-devices 2>/dev/null | grep -q "Device"; then
    echo -e "  RealSense: ${GREEN}検出${NC}"
else
    echo -e "  RealSense: ${YELLOW}未検出${NC}"
fi

echo ""
