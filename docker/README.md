# VLAbor Docker セットアップガイド

VRテレオペレーションフレームワーク「VLAbor」のDocker環境

## クイックスタート

```bash
# 1. セットアップスクリプトを実行
./setup.sh

# 2. ローカル版を起動（同一ネットワーク内、VPN不要）
docker compose up local
```

## システム構成

```
┌─── VR側（リモート）─────────────┐     ┌─── Robot側（現場）────────────┐
│  vlabor-vr                     │     │  vlabor-robot-fluent         │
│  - Unity TCP Endpoint (:42000) │     │  - Robot Control Nodes       │
│  - Zenoh Bridge (→Robot)       │◄───►│  - Zenoh Bridge (:7447)      │
│  - Tailscale                   │     │  - Foxglove Bridge (:8765)   │
└────────────────────────────────┘     │  - カメラ (USB/RealSense)    │
                                       │  - Tailscale                 │
                                       └───────────────────────────────┘
```

## Docker イメージ一覧

| イメージ | 用途 | サイズ | カメラ | Tailscale |
|---------|------|--------|--------|-----------|
| `vlabor-local` | ローカル環境（VPN不要） | ~5GB | USB | - |
| `vlabor-fluent` | ローカル+フル機能 | ~5GB | USB+RealSense | - |
| `vlabor-robot-fluent` | リモートRobot側（フル機能） | ~5GB | USB+RealSense | Yes |
| `vlabor-robot` | リモートRobot側（軽量） | ~1.3GB | - | Yes |
| `vlabor-vr` | リモートVR側 | ~1GB | - | Yes |

## 使用シナリオ

### シナリオ1: ローカル環境（同一ネットワーク）

VRデバイスとロボットが同じネットワーク内にある場合：

```bash
docker compose up local
```

Unity側の設定：
- ROS IP: `<このマシンのIP>`
- ROS Port: `42000`

### シナリオ2: リモート環境（インターネット経由）

#### Robot側マシンで実行：
```bash
# Tailscale認証キーを設定
export TS_AUTHKEY=tskey-auth-xxxxx

# Robot側を起動
docker compose up robot-fluent
```

#### VR側マシンで実行：
```bash
# Robot側のTailscale IPを指定
export ROBOT_TAILSCALE_IP=100.x.x.x
export TS_AUTHKEY=tskey-auth-xxxxx

docker compose up vr
```

## 環境変数

`.env`ファイルまたは環境変数で設定できます：

| 変数名 | デフォルト | 説明 |
|--------|-----------|------|
| `ROS_TCP_PORT` | 42000 | Unity TCP Endpointのポート |
| `LEFT_SERIAL_PORT` | /dev/ttyACM0 | 左アームのシリアルポート |
| `RIGHT_SERIAL_PORT` | /dev/ttyACM1 | 右アームのシリアルポート |
| `DRIVER_BACKEND` | feetech | ドライバーバックエンド |
| `AUTO_ENABLE` | false | 自動トルクON |
| `CAMERA_DEVICE` | /dev/video0 | カメラデバイス |
| `ENABLE_CAMERA` | true | カメラを有効化 |
| `TS_AUTHKEY` | - | Tailscale認証キー |
| `ROBOT_TAILSCALE_IP` | - | Robot側のTailscale IP |
| `ZENOH_PORT` | 7447 | Zenoh Bridgeのポート |
| `ALLOWED_VR_IP` | - | 操作を許可するVRのIP（空=全許可） |

## RealSenseを使用する場合

ホストマシンでカーネルモジュールをインストールする必要があります：

```bash
# セットアップスクリプトで自動インストール
./setup.sh --realsense

# または手動でインストール
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update && sudo apt install librealsense2-dkms
```

## ポート一覧

| ポート | 用途 |
|--------|------|
| 42000 | Unity TCP Endpoint |
| 8765 | Foxglove Bridge (映像配信) |
| 7447 | Zenoh Bridge (リモート通信) |

## 映像視聴（ブラウザ）

ロボットの映像をブラウザで確認できます：

```
http://<Robot IP>:8765
```

Foxglove Studioで接続する場合も同じURLを使用します。

## マルチユーザーアクセス

複数のVRユーザーが同時にアクセスする場合：

- **操作者**: 1人のみ（`ALLOWED_VR_IP`で指定）
- **視聴者**: 複数可能（Foxglove Bridge経由でカメラ映像を視聴）

```bash
# Robot側で操作を許可するIPを指定
ALLOWED_VR_IP=100.x.x.x docker compose up robot-fluent
```

## トラブルシューティング

### シリアルポートが見つからない

```bash
# デバイスを確認
ls -la /dev/ttyACM*

# 権限を追加（必要な場合）
sudo usermod -aG dialout $USER
# ログアウト後に再ログイン
```

### カメラが見つからない

```bash
# デバイスを確認
ls -la /dev/video*

# v4l2-utilsで確認
v4l2-ctl --list-devices
```

### RealSenseが検出されない

```bash
# ホストで確認
rs-enumerate-devices

# カーネルモジュールを確認
lsmod | grep uvcvideo
```

### Tailscaleに接続できない

```bash
# 手動でログイン
docker exec -it vlabor-robot-fluent tailscale up

# ステータス確認
docker exec -it vlabor-robot-fluent tailscale status
```

## ビルド方法

ソースからビルドする場合：

```bash
cd ~/ros2_ws/src/vr_ros2/docker
docker compose build local
docker compose build robot-fluent
```

## Zenoh Router（オプション）

両側がNAT内の場合、クラウドにZenoh Routerを立てることもできます：

```bash
# クラウドサーバーで
docker compose --profile router up zenoh-router
```

## ライセンス

MIT License

---

VLAbor - VLA + Labor/Lab
