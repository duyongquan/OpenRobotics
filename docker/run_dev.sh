#!/usr/bin/env bash
# https://docs.ros.org/en/galactic/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

# copy OpenRobotics docker
# if [ -d LTSLAM ]; then
#     rm -rf ./OpenRobotics
# fi
# mkdir OpenRobotics
# cp -r ${CONTEXT}/../* ./OpenRobotics

REPO=osrf/ros:galactic-desktop
TAG="${REPO}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"