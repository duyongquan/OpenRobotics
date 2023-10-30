#!/usr/bin/env bash

# Fail on first error.
set -e


mkdir -p /workspace/src
cd  /workspace/src

git clone -b galactic https://github.com/ros-planning/navigation2.git