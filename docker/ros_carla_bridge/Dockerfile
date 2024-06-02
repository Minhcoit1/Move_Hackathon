FROM osrf/ros:noetic-desktop-focal

ARG CARLA_VERSION=0.9.13
ARG UBUNTU_VERSION=20.04
ARG VMP_ROOT=/opt/vehicle-motion-playground

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    ros-noetic-perception-pcl \
    ros-noetic-rqt-gui-py \
    ros-noetic-derived-object-msgs \
    ros-noetic-cv-bridge \
    wget \
    python3-pip \
    xdg-user-dirs

RUN mkdir -p ${VMP_ROOT}
WORKDIR ${VMP_ROOT}

COPY ./02_RosCommunication ./02_RosCommunication
RUN source /opt/ros/noetic/setup.bash && \
    cd ./02_RosCommunication && \
    catkin_make

COPY ./01_RosBridge ./01_RosBridge
RUN source /opt/ros/noetic/setup.bash && \
    source ${VMP_ROOT}/02_RosCommunication/devel/setup.bash && \
    cd ./01_RosBridge && \
    catkin_make && \
    source ${VMP_ROOT}/01_RosBridge/devel/setup.bash

COPY ./requirements.txt .
RUN pip3 install -r requirements.txt

COPY ./carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg .
ENV PYTHONPATH ${VMP_ROOT}/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg:${PYTHONPATH}

COPY ./ros_entrypoint.sh /

# Ensure script is executable
RUN ["sudo", "chmod", "+x", "/ros_entrypoint.sh"]

ENTRYPOINT ["/ros_entrypoint.sh"]