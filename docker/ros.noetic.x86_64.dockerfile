FROM osrf/ros:noetic-desktop-full

MAINTAINER duyongquan <quandy2020@126.com>

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    qtcreator \
    libqt5charts5-dev
    libqt5svg5-dev \
    libompl-dev \

# OpenRobitcs workspace
ENV ROS_WS /workspace

RUN mkdir -p $ROS_WS
WORKDIR $ROS_WS
