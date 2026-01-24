# VLAbor Docker FluentVision統合

**日付**: 2026-01-22

## 概要

VLAbor Docker環境にFluentVisionパッケージ（カメラ、RealSense、AI）を統合。
ローカル版とリモート版（Tailscale + Zenoh）の両方をサポート。

## 変更ファイル

### 新規作成
- `docker/Dockerfile.fluent` - ローカル版 + FluentVision
- `docker/Dockerfile.robot-fluent` - リモートRobot + FluentVision + RealSense
- `docker/entrypoint-fluent.sh` - Fluent版エントリーポイント
- `docker/entrypoint-robot-fluent.sh` - Robot-Fluent版エントリーポイント
- `docker/setup.sh` - セットアップスクリプト
- `docker/.env.example` - 環境変数テンプレート
- `docker/.env` - 実際の環境変数（privateリポジトリ用）

### 修正
- `docker/Dockerfile.local` - FluentVision、usb_camパッケージ追加
- `docker/Dockerfile.robot` - COPYパス修正、CMD修正
- `docker/Dockerfile.vr` - COPYパス修正、CMD修正
- `docker/docker-compose.yml` - contextパス修正、新サービス追加
- `docker/entrypoint-local.sh` - カメラ起動、待機モード追加
- `docker/entrypoint-robot.sh` - Tailscaleオプション化、待機モード
- `docker/entrypoint-vr.sh` - Zenohクライアントモード、Tailscaleオプション化
- `docker/README.md` - 包括的なセットアップガイド

## 修正したエラー

### 1. Context path error
```
lstat /home/takatronix/ros2_ws/src/src: no such file
```
**原因**: docker-compose.ymlのcontextが`../..`で階層不足
**修正**: `context: ../../..` に変更

### 2. pcl_conversions not found
```
Could not find package configuration file provided by "pcl_conversions"
```
**原因**: fluent_libビルドに必要なPCLパッケージ不足
**修正**: `ros-humble-pcl-ros`, `ros-humble-pcl-conversions`追加

### 3. librealsense2-dkms build failure
```
Error! The dkms.conf file is missing.
```
**原因**: DockerコンテナでDKMS（カーネルモジュール）はビルド不可
**修正**: dkmsスキップ、utils/devのみインストール
**注意**: ホスト側でRealSenseカーネルモジュールが必要

### 4. COPY path not found
```
/docker/entrypoint-robot.sh: not found
```
**原因**: COPYのパスがcontext基準で不正
**修正**: `COPY src/vr_ros2/docker/entrypoint-*.sh /entrypoint.sh`

### 5. Container immediate exit
**原因**: `CMD ["bash"]`がentrypointの引数になり、`exec bash`実行
**修正**: `CMD []`に変更、待機は`wait`で実装

### 6. Tailscale auth failure without key
**原因**: TS_AUTHKEY未設定時にtailscale upが失敗
**修正**: `if [ -n "$TS_AUTHKEY" ]`でオプション化

### 7. Zenoh port 7447 conflict
**原因**: 同一マシンでRobotとVR両方がポート7447をリッスン
**修正**: VR側を`zenoh-bridge-ros2dds client -e "tcp/..."`に変更

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    ローカル版 (local/fluent)                  │
│  Unity ←→ TCP:42000 ←→ ROS2 ←→ Robot Arms                  │
│                           ↓                                   │
│                    Foxglove:8765 (Web UI)                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    リモート版                                 │
│                                                              │
│  ┌──────────────┐          Tailscale           ┌──────────┐ │
│  │   VR側       │  ←─── Zenoh:7447 ───→  │ Robot側  │ │
│  │ (クライアント) │                              │ (サーバー) │ │
│  │ Unity:42000  │                              │ Arms+Cam │ │
│  └──────────────┘                              └──────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 使用方法

### ローカル版（同一ネットワーク）
```bash
cd ~/ros2_ws/src/vr_ros2/docker
docker compose up local
```

### リモート版（Tailscale経由）
```bash
# Robot側
TS_AUTHKEY=tskey-xxx docker compose up robot-fluent

# VR側（別マシン）
ROBOT_TAILSCALE_IP=100.x.x.x docker compose up vr
```

## 次のAI向けメモ

1. RealSenseはホスト側でdkmsインストール必須
2. Zenoh v1.5.0使用、clientモードで接続
3. カメラは`ENABLE_CAMERA=true`で有効化
4. Foxglove Bridgeはポート8765で映像配信
5. Tailscale認証キーはOAuth Clientから取得
