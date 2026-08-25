FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ==============================
# 删除可能存在的 ROS 2 源
# 说明：ros:humble 基础镜像自带的 ROS 源 GPG 密钥已过期(EXPKEYSIG F42ED6FBAB17C654)，
#       会导致 apt-get update 报签名错误，故先移除；下方会用清华 TUNA 源补回。
# ==============================
RUN rm -f /etc/apt/sources.list.d/ros*.list

# ==============================
# 设置语言环境
# ==============================
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ==============================
# 安装开发环境（全部来自 Ubuntu 官方源）
# ==============================
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    vim \
    nano \
    curl \
    wget \
    unzip \
    build-essential \
    cmake \
    gdb \
    python3-pip \
    python3-vcstool \
    python3-argcomplete \
    libopencv-dev \
    libeigen3-dev \
    usbutils \
    udev \
    net-tools \
    iputils-ping \
    htop \
    tree \
    && rm -rf /var/lib/apt/lists/*

# ==============================
# 初始化 rosdep
# ==============================
RUN rosdep init 2>/dev/null || true

# ==============================
# 加入清华 TUNA ROS 2 源 + 当前 ROS 签名密钥，固化 robot_localization
# 背景：2026-08-14 曾在容器运行时用清华源装好 robot_localization（ekf_node 依赖）
#       + diagnostic-updater 4.0.7（基础镜像 3.1.2 为纯 Python 版，缺
#       libdiagnostic_updater.so，ekf_node 起不来）。但当时只存在容器可写层、未进
#       镜像，容器一重建就丢（2026-08-17 排查确认）。故在此把密钥与源写死进镜像，
#       永久固化。
#       ros-archive-keyring.gpg 是从已验证容器导出的当前 ROS2 签名密钥，避免构建
#       时依赖 raw.githubusercontent 外网。
# ==============================
COPY ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-tuna.list \
    && apt-get update \
    && apt-get install -y \
        ros-humble-robot-localization \
        ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# ==============================
# 创建普通开发用户（兼容已存在的用户）
# ==============================
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

RUN if getent group ${USER_GID} > /dev/null 2>&1; then \
        echo "Group gid ${USER_GID} already exists."; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi && \
    if id -u ${USERNAME} > /dev/null 2>&1; then \
        echo "User ${USERNAME} already exists, updating UID/GID/home."; \
        usermod -u ${USER_UID} -g ${USER_GID} -d /home/${USERNAME} -m -s /bin/bash ${USERNAME}; \
    else \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ==============================
# 创建 ROS2 工作空间
# ==============================
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros2_ws

# ==============================
# 切换用户
# ==============================
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ==============================
# 自动加载环境
# ==============================
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
