FROM osrf/ros:galactic-desktop

MAINTAINER duyongquan <quandy2020@126.com>

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends 

# OpenRobitcs workspace
ENV ROS2_WS /workspace

RUN mkdir -p $ROS2_WS
WORKDIR $ROS2_WS

# Install OpenRobitcs
# COPY ./install /tmp/install

# RUN bash /tmp/install/install_navigation2.sh
# RUN bash /tmp/install/install_open_robotics.sh
