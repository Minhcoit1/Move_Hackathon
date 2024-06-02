#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/vehicle-motion-playground/02_RosCommunication/devel/setup.bash
source /opt/vehicle-motion-playground/01_RosBridge/devel/setup.bash

XDG_RUNTIME_DIR="/tmp/xdg_runtime_dir"
mkdir -p ${XDG_RUNTIME_DIR} && chmod 777 ${XDG_RUNTIME_DIR}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}

exec "$@"
