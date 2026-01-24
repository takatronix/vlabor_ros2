# EtherCAT ROS2 通信調査レポート

## 概要

EtherCAT接続のDAIHENロボットアームをROS2から制御するための通信方法を調査しました。
EtherCATでの接続には**2つのパターン**があり、DAIHENロボットアームがどちらのモードで動作するかによって構成が異なります。

> **注**: 本ドキュメントで使用する専門用語（EtherCAT、PDO等）については「用語説明」セクションで解説しています。

---

## ROS2制御の実現性

### 結論

**EtherCAT対応のDAIHENロボットアームをROS2でコントロールすることは可能と考えられます。**

ただし、DAIHENコントローラーがEtherCATマスター/スレーブどちらで動作するかにより、接続方法と難易度が異なります。

### ケース別の実現性

| ケース | 実現性 | 難易度 | 接続方法 |
|--------|--------|--------|----------|
| **ケースA: DAIHENがスレーブ** | ✅ 可能 | 中程度 | ethercat_driver_ros2 + ros2_control |
| **ケースB: DAIHENがマスター** | ✅ 可能 | 高い | EtherNet/IP 等の上位プロトコル |

### ケースA（スレーブ）の場合の制御機能

| 機能 | 対応状況 | 備考 |
|------|---------|------|
| 位置制御 | ✅ | EtherCATで直接指令 |
| 速度制御 | ✅ | EtherCATで直接指令 |
| トルク制御 | ✅ | EtherCATで直接指令 |
| MoveIt2連携 | ✅ | ros2_control経由 |
| リアルタイム制御 | ✅ | 1ms周期も可能（RTカーネル・NIC・負荷条件に依存） |

### ケースB（マスター）の場合の制御機能

| 機能 | 対応状況 | 備考 |
|------|---------|------|
| プログラム実行/停止 | △ | プロトコル・機種依存（要確認） |
| 位置指令 | △ | プロトコル・機種依存（要確認） |
| I/O制御 | △ | プロトコル・機種依存（要確認） |
| MoveIt2連携 | △ | カスタム実装が必要 |
| リアルタイム制御 | △ | プロトコル依存（数十ms程度が一般的、要検証） |

### 確認が必要な事項

1. **最重要**: EtherCATマスター/スレーブどちらで動作するか？
2. マスターの場合、推奨する上位プロトコルは何か？
3. 外部からの制御コマンド仕様（位置指令の送り方等）

---

## 用語説明

### EtherCAT

| 項目 | 内容 |
|------|------|
| 正式名称 | Ethernet for Control Automation Technology |
| 開発元 | Beckhoff Automation GmbH（ドイツ） |
| 特徴 | 産業用リアルタイムEthernet通信プロトコル |
| 物理層 | 標準Ethernet（100Mbps）を使用、**専用ハードウェア不要** |
| 通信方式 | マスター/スレーブ型、**IPアドレスは使用しない** |

### IgH EtherCAT Master

| 項目 | 内容 |
|------|------|
| 正式名称 | IgH EtherCAT Master for Linux |
| 開発元 | IgH (Ingenieurgemeinschaft Haus GmbH) - ドイツ |
| 別名 | **EtherLab** |
| ライセンス | GPL v2（オープンソース） |
| 動作方式 | Linuxカーネルモジュール |
| 用途 | LinuxでEtherCATマスター機能を実現 |

### EtherCATマスターの選択肢

| 名称 | 開発元 | 方式 | 特徴 |
|------|--------|------|------|
| **IgH (EtherLab)** | IgH GmbH | カーネルモジュール | 高性能、産業用途で実績多数、ethercat_driver_ros2が採用 |
| **SOEM** | OpenEtherCATsociety | ユーザースペース | 軽量、開発・テスト向け |

本ドキュメントでは、ethercat_driver_ros2が採用している**IgH EtherCAT Master**を使用します。

### PDO (Process Data Object)

EtherCATでリアルタイムにやり取りするデータの単位です。

| 種類 | 方向 | 内容 | 例 |
|------|------|------|-----|
| **RxPDO** | マスター → スレーブ | 指令値 | 目標位置、目標速度、トルク指令 |
| **TxPDO** | スレーブ → マスター | フィードバック | 現在位置、現在速度、エラー状態 |

```
┌─────────────┐                    ┌─────────────┐
│   ROS2 PC   │  ── RxPDO ──▶     │   ロボット   │
│  (マスター)  │   （指令を送る）    │  (スレーブ)  │
│             │                    │             │
│             │  ◀── TxPDO ──     │             │
│             │  （状態を受け取る）  │             │
└─────────────┘                    └─────────────┘
```

PDOは高周期でやり取りされ、1kHz程度のサイクルが可能な構成もあります（参考値、機器・設定に依存）。
PDOの中身（どのデータをやり取りするか）はデバイスごとに異なるため、**PDOマッピング仕様**をメーカーから入手する必要があります。

---

## ケース判別

```
【ケースA】DAIHENロボットアームがスレーブ → ethercat_driver_ros2 が使える
【ケースB】DAIHENロボットアームがマスター → EtherNet/IP 等を使用
```

---

# ケースA: DAIHENがEtherCATスレーブの場合

## システム構成図

```
┌─────────────────────────────────────┐
│           ROS2 PC                   │
│  ┌─────────────────────────────┐    │
│  │ Ubuntu 22.04 LTS            │    │
│  │ ROS2 Humble                 │    │
│  │ ethercat_driver_ros2        │    │
│  │ IgH EtherCAT Master 1.5     │    │
│  └─────────────────────────────┘    │
│           │                         │
│  ┌────────┴────────┐                │
│  │ Intel NIC (専用) │ ← EtherCAT専用 │
│  └────────┬────────┘                │
└───────────│─────────────────────────┘
            │ EtherCAT (100Mbps)
            ▼
┌─────────────────────────────────────┐
│   DAIHEN コントローラー (スレーブ)     │
│   ┌─────────────────────────────┐   │
│   │       ロボットアーム         │   │
│   └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

---

## 必要なハードウェア

### 1. PC本体

| 項目 | 推奨スペック | 備考 |
|------|-------------|------|
| CPU | Intel Core i5以上 | - |
| メモリ | 8GB以上 | 16GB推奨 |
| ストレージ | SSD | - |

### 2. ネットワークカード (NIC)

**EtherCAT専用のNICを用意してください（オンボードLANとは別）**

IgH EtherCAT Master stable-1.5 で対応しているドライバ（実際のソースコードから確認）:

| ドライバ名 | 対応チップ | 備考 |
|-----------|-----------|------|
| `generic` | 汎用 | どのNICでも動作（性能は劣る） |
| `igb` | Intel I210, I350等 | **推奨** |
| `igc` | Intel I225, I226等 | 2.5GbE対応 |
| `e1000e` | Intel 古いチップ | - |
| `r8169` | Realtek | 動作するが非推奨 |

### 3. ケーブル

| 項目 | 仕様 |
|------|------|
| 規格 | CAT5e以上 |
| 種類 | ストレートケーブル |

---

## 必要なソフトウェア

### OS

| コンポーネント | バージョン | 備考 |
|----------------|-----------|------|
| Ubuntu | **22.04 LTS** | 必須（公式ドキュメント記載） |
| Secure Boot | **無効または署名が必要** | IgHがカーネルモジュールのため（MOK署名でも可） |

### リアルタイムカーネル（任意）

高精度制御が必要な場合のみ。Ubuntu 22.04で利用可能なパッケージの例:

```bash
# 利用可能なRTカーネルを検索
apt search linux-image.*realtime
```

**注意**: パッケージ名・バージョンは変更される可能性があります。インストール前に上記コマンドで最新の利用可能なパッケージを確認してください。

---

## インストール手順

以下は [ethercat_driver_ros2 公式INSTALL.md](https://github.com/ICube-Robotics/ethercat_driver_ros2/blob/main/INSTALL.md) に基づく手順です。

### 1. IgH EtherCAT Master のインストール

```bash
# 必須ツールのインストール
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools

# ソース取得
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.5
sudo rm -f /usr/bin/ethercat
sudo rm -f /etc/init.d/ethercat
./bootstrap

# ビルド・インストール
./configure --prefix=/usr/local/etherlab --disable-8139too --disable-eoe --enable-generic
make all modules
sudo make modules_install install
sudo depmod
```

**注意**: Linuxカーネルが更新されるたびに `make all modules` 以降を再実行する必要があります。

### 2. システム設定

```bash
# シンボリックリンク作成
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir -p /etc/sysconfig
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat

# udevルール作成
sudo tee /etc/udev/rules.d/99-EtherCAT.rules << 'EOF'
KERNEL=="EtherCAT[0-9]*", MODE="0664"
EOF

# ネットワーク設定（/etc/sysconfig/ethercat を編集）
# MASTER0_DEVICE="xx:xx:xx:xx:xx:xx"  ← NICのMACアドレス
# DEVICE_MODULES="generic"             ← または "igb" 等
```

### 3. EtherCATマスター起動・確認

```bash
# 起動
sudo /etc/init.d/ethercat start
# 出力例: Starting EtherCAT master 1.5.2  done

# 接続スレーブ確認
ethercat slaves
# 出力例: 0  0:0  PREOP  +  <device_name>
```

### 4. ROS2 Humble インストール

[公式ドキュメント](https://docs.ros.org/en/humble/Installation.html) に従ってインストール。

```bash
# 環境読み込み
source /opt/ros/humble/setup.bash

# colconインストール
sudo apt install python3-colcon-common-extensions
```

### 5. ethercat_driver_ros2 ビルド

```bash
# ワークスペース作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# パッケージ取得
git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2

# 依存解決・ビルド
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# 環境読み込み
source install/setup.bash
```

---

## 検証済みリポジトリ

以下のリポジトリは2026年1月15日時点で存在を確認済み:

| リポジトリ | URL | 状態 |
|-----------|-----|------|
| IgH EtherCAT Master | https://gitlab.com/etherlab.org/ethercat.git | ✅ 存在 |
| ethercat_driver_ros2 | https://github.com/ICube-Robotics/ethercat_driver_ros2.git | ✅ 存在 |
| SOEM | https://github.com/OpenEtherCATsociety/SOEM.git | ✅ 存在 |

---

## PDOマッピング設定

DAIHENから入手すべき情報:

| 項目 | 説明 |
|------|------|
| ESIファイル | EtherCAT Slave Information (XMLファイル) |
| Vendor ID | メーカー識別子 |
| Product ID | 製品識別子 |
| RxPDO | 指令値 (位置/速度/トルク) |
| TxPDO | フィードバック (実位置/状態) |

---

# ケースB: DAIHENがEtherCATマスターの場合

DAIHENコントローラーがEtherCATマスターとして動作する場合、ROS2 PCからEtherCATで直接接続することは原則できません。
この場合、**上位通信プロトコル**を使用してROS2 PCとDAIHENコントローラーを接続します。

## システム構成図

```
┌─────────────────────────────────────┐
│           ROS2 PC                   │
│  ┌─────────────────────────────┐    │
│  │ Ubuntu 22.04 LTS            │    │
│  │ ROS2 Humble                 │    │
│  │ 通信ライブラリ               │    │
│  │  - EtherNet/IP (役割は要確認)│    │
│  └─────────────────────────────┘    │
│           │                         │
│  ┌────────┴────────┐                │
│  │ 標準 Ethernet NIC│                │
│  └────────┬────────┘                │
└───────────│─────────────────────────┘
            │ TCP/IP (標準Ethernet)
            ▼
┌─────────────────────────────────────┐
│   DAIHEN コントローラー (マスター)    │
│   （OS・バージョンは要確認）          │
│   ┌─────────────────────────────┐   │
│   │ 上位通信サーバー             │   │
│   │  - EtherNet/IP (役割は要確認)│   │
│   └─────────────────────────────┘   │
│           │ EtherCAT (マスター)      │
│           ▼                         │
│   ┌─────────────────────────────┐   │
│   │ EtherCATスレーブデバイス     │   │
│   │ (サーボドライブ等)           │   │
│   └─────────────────────────────┘   │
│   ┌─────────────────────────────┐   │
│   │       ロボットアーム         │   │
│   └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

**注**: 上記は一般的な産業用ロボットの構成例です。

---

## DAIHENコントローラーの対応プロトコル

OTC DAIHENの公式情報によると、以下のプロトコルをサポートしています:

**情報ソース**: [OTC DAIHEN - Accessories and interfaces](https://otc-daihen.com/automation/robot-accessories/interfaces.html)

| プロトコル | サポート状況 |
|-----------|-------------|
| EtherNet/IP | ✅ 対応 |
| PROFINET | ✅ 対応 |
| PROFIBUS | ✅ 対応 |
| DeviceNet | ✅ 対応 |
| CC-Link | ✅ 対応 |


---

## 推奨プロトコル比較

| プロトコル | ROS2対応状況 | 特徴 | 推奨度 |
|-----------|-------------|------|--------|
| **EtherNet/IP** | ⚠️ ROS2実装は限定的 | リアルタイム制御可能、Rockwell系PLCとの親和性高い | ⭐⭐ |
| PROFINET | ⚠️ ROS2対応限定的 | Siemens系との親和性高い | ⭐ |

**注意事項**:
- EtherNet/IP、PROFINETはROS2実装が限定的なため、カスタム開発が必要になる可能性があります

---

## 推奨: EtherNet/IP接続

### 概要

EtherNet/IP (Ethernet Industrial Protocol) はODVAが策定した産業用Ethernetプロトコルです。
標準Ethernet上でリアルタイム制御データと通常のTCP/IPトラフィックを共存させることができます。

### メリット

- 標準Ethernetインフラを使用
- サイクリック通信（リアルタイム制御）対応
- 非サイクリック通信（設定・診断）対応
- Rockwell Automation製品との高い互換性

### ROS2 EtherNet/IPライブラリ

| ライブラリ | URL | 特徴 | 状態 |
|-----------|-----|------|------|
| odva_ethernetip | https://github.com/ros-drivers/odva_ethernetip | ODVA準拠 | ⚠️ **ROS1専用** |
| ibt_ros2_ethernetip | https://github.com/InnoboticsSRL/ibt_ros2_ethernetip | ROS2向け実装 | 要確認 |

### 注意事項

**ROS2向けEtherNet/IP実装は存在するものの、成熟度・互換性は要確認です。**

EtherNet/IPでDAIHENと接続する場合、以下のいずれかが必要です：
1. ROS1のodva_ethernetipをROS2に移植する
2. ros1_bridgeを使用してROS1ノード経由で接続する
3. EtherNet/IPライブラリを直接使用してカスタムROS2ノードを作成する

このため、EtherNet/IPはカスタム実装を前提とした検討が必要です。

---

## 確認チェックリスト

### 必須

- [ ] EtherCATマスター/スレーブどちらで動作するか
- [ ] コントローラー型番・ファームウェアバージョン

### ケースA (スレーブ) の場合

- [ ] ESIファイル (EtherCAT Slave Information)
- [ ] Vendor ID / Product ID
- [ ] PDOマッピング仕様
- [ ] CiA402プロファイル対応有無
- [ ] 対応サイクルタイム (最小値)

### ケースB (マスター) の場合

- [ ] 対応する上位通信プロトコル（EtherNet/IP 等）
- [ ] EtherNet/IPのアセンブリインスタンス番号
- [ ] 外部制御コマンド仕様（位置指令、速度指令、I/O制御等）
- [ ] 通信周期の制限

---

## 参考リンク

### EtherCAT関連

| 項目 | URL |
|------|-----|
| ethercat_driver_ros2 | https://github.com/ICube-Robotics/ethercat_driver_ros2 |
| ethercat_driver_ros2 ドキュメント | https://icube-robotics.github.io/ethercat_driver_ros2/ |
| IgH EtherCAT Master | https://gitlab.com/etherlab.org/ethercat |
| SOEM | https://github.com/OpenEtherCATsociety/SOEM |

### ROS2・DAIHEN関連

| 項目 | URL |
|------|-----|
| ROS2 Humble インストール | https://docs.ros.org/en/humble/Installation.html |
| DAIHEN ロボットサイト | https://www.daihen-robot.com/ |
| OTC DAIHEN インターフェース | https://otc-daihen.com/automation/robot-accessories/interfaces.html |

---

*調査日: 2026-01-15*
*情報源: 各リポジトリの公式ドキュメント、実際のapt検索結果*
