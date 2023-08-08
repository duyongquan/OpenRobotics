#!/usr/bin/env bash

DOCKERFILE=$1

CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

# copy OpenRobotics docker
# if [ -d LTSLAM ]; then
#     rm -rf ./OpenRobotics
# fi
# mkdir OpenRobotics
# cp -r ${CONTEXT}/../* ./OpenRobotics

REPO=open_robotics
TAG="${REPO}"

# Fail on first error.
set -e
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"