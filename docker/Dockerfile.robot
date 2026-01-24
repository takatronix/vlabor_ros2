#
# VLAbor - Robot Side Docker Image
#
# Robot側（現場）で使用: ロボット制御ノード + Zenoh Bridge
#
# Build:
#   docker build -f Dockerfile.robot -t vlabor-robot .
#
# Run:
#   docker run -it --net=host --privileged \
#     -v /dev:/dev \
#     -e TS_AUTHKEY=tskey-client-xxx \
#     vlabor-robot
#

FROM ros:humble-ros-base

LABEL maintainer="takatronix"
LABEL description="VLAbor - Robot側 (Robot Control + Zenoh Bridge)"

# 環境変数
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ZENOH_VERSION=1.5.0

# 基本パッケージインストール
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    unzip \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Python依存関係
RUN pip3 install \
    pyserial \
    numpy \
    ikpy

# Tailscale インストール
RUN curl -fsSL https://tailscale.com/install.sh | sh

# Zenoh Bridge インストール
RUN cd /tmp && \
    wget -q "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip" -O zenoh.zip && \
    unzip zenoh.zip && \
    mv zenoh-bridge-ros2dds /usr/local/bin/ && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm -rf /tmp/*

# ワークスペース作成
WORKDIR /ros2_ws

# ソースコードコピー（ビルド時）
# COPY . /ros2_ws/src/vr_ros2

# エントリーポイントスクリプト
COPY src/vr_ros2/docker/entrypoint-robot.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 環境変数（デフォルト値）
ENV ZENOH_PORT=7447
ENV LEFT_SERIAL_PORT=/dev/ttyACM0
ENV RIGHT_SERIAL_PORT=/dev/ttyACM1
ENV DRIVER_BACKEND=feetech
ENV AUTO_ENABLE=false
ENV TS_AUTHKEY=""

EXPOSE 7447 8765

ENTRYPOINT ["/entrypoint.sh"]
CMD []
