FROM docker.io/ros:noetic-perception-focal

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && rm -f /etc/apt/sources.list.d/*.list \
    && printf '%s\n' \
        'deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse' \
        'deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse' \
        'deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse' \
        'deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse' \
        > /etc/apt/sources.list \
    && printf '%s\n' \
        'deb https://mirrors.ustc.edu.cn/ros/ubuntu/ focal main' \
        > /etc/apt/sources.list.d/ros-latest.list

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libboost-thread-dev \
    libboost-dev \
    libomp-dev \
    wget \
    ca-certificates \
    ros-noetic-rviz \
    ros-noetic-image-transport-plugins \
    ros-noetic-eigen-conversions \
    ros-noetic-tf \
    ros-noetic-pcl-ros

WORKDIR /tmp
RUN git clone https://github.com/strasdat/Sophus.git \
    && cd Sophus \
    && git checkout a621ff \
    && sed -i 's/unit_complex_.real() = 1./unit_complex_ = std::complex<double>(1., unit_complex_.imag())/' sophus/so2.cpp \
    && sed -i 's/unit_complex_.imag() = 0./unit_complex_ = std::complex<double>(unit_complex_.real(), 0.)/' sophus/so2.cpp \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j"$(nproc)" && make install \
    && ldconfig \
    && cd / && rm -rf /tmp/Sophus

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

RUN git clone https://github.com/xuankuzcr/rpg_vikit.git

COPY . /catkin_ws/src/fast_livo

WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j"$(nproc)"

COPY ros_entrypoint.sh /ros_entrypoint_custom.sh
RUN chmod +x /ros_entrypoint_custom.sh

WORKDIR /catkin_ws
ENTRYPOINT ["/ros_entrypoint_custom.sh"]
CMD ["bash"]
