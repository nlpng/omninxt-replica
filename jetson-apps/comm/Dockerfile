FROM ros:noetic-ros-base

ENV DAI=2.26.0
ENV DAI_WS=/root/dai_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV WS=/root/noetic_ws

WORKDIR ${DAI_WS}
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        libusb-1.0-0-dev \
        python3-catkin-tools \
        software-properties-common \
    # build DAI Core
    && wget https://github.com/luxonis/depthai-core/releases/download/v${DAI}/depthai-core-v${DAI}.tar.gz \
    && tar -xvf depthai-core-v${DAI}.tar.gz \
    && rm depthai-core-v${DAI}.tar.gz \
    && cd depthai-core-v${DAI} \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    # copy DAI Core files
    && cd ${DAI_WS}/depthai-core-v${DAI}/build/install/include \
    && mv ./depthai /usr/local/include/depthai \
    && mv ./depthai-shared /usr/local/include/depthai-shared \
    && mv ./depthai-bootloader-shared /usr/local/include/depthai-bootloader-shared \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-mavros \
        ros-noetic-mavros-extras\
        ros-noetic-depthai-bridge \
        ros-noetic-depthai-ros-msgs \
        ros-noetic-image-transport \
        ros-noetic-cv-bridge \
        ros-noetic-rviz \
        ros-noetic-foxglove-bridge \
    && cd /tmp \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x ./install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR ${WS}

COPY ./resources/oak_ffc_4p $WS/src/oak_ffc_4p

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash \
    && catkin build -DCMAKE_BUILD_TYPE=Release

RUN sed --in-place --expression \
    '$isource "/root/noetic_ws/devel/setup.bash"' /ros_entrypoint.sh
