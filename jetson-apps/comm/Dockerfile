FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV WS=/root/noetic_ws

COPY ./resources/depthai-ros/depthai-ros $WS/src/depthai-ros
COPY ./resources/depthai-ros/depthai_bridge $WS/src/depthai_bridge
COPY ./resources/depthai-ros/depthai_descriptions $WS/src/depthai_descriptions
COPY ./resources/depthai-ros/depthai_ros_driver $WS/src/depthai_ros_driver
COPY ./resources/depthai-ros/depthai_ros_msgs $WS/src/depthai_ros_msgs

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
        ros-noetic-mavros \
        ros-noetic-mavros-extras\
        ros-noetic-rviz \
        ros-noetic-foxglove-bridge \
    && cd /tmp \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x ./install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && cd $WS \
    && rosdep install --from-paths src --rosdistro noetic --ignore-src -y \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /root/noetic_ws

COPY ./resources/oak_ffc_4p $WS/src/oak_ffc_4p

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash \
    && catkin build -DCMAKE_BUILD_TYPE=Release

RUN sed --in-place --expression \
    '$isource "/root/noetic_ws/devel/setup.bash"' /ros_entrypoint.sh
