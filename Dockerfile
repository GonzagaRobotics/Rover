# syntax=docker/dockerfile:1
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

# ROS wants UTF-8 locales
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS2 Jazzy
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe
RUN apt-get update
RUN apt-get install -y curl

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros-base
RUN apt-get install -y ros-dev-tools

RUN echo ". /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Rover
RUN apt-get install -y ros-jazzy-pcl-ros
RUN apt-get install -y ffmpeg libavformat-dev libswscale-dev libavcodec-dev

COPY --exclude=__pycache__/ src/ /Rover/src/
COPY --exclude=__pycache__/ launch/ /Rover/launch/