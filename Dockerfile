# syntax=docker/dockerfile:1
FROM ros-jazzy
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y ros-jazzy-pcl-ros ffmpeg libavformat-dev libswscale-dev libavcodec-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install onnxruntime opencv-python-headless -I --break-system-packages --no-cache-dir

WORKDIR /Rover
# COPY . ./

SHELL ["/bin/bash", "-c"]
CMD ["./scripts/run.sh", "object_detect"]
# RUN . /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to object_detect
USER 1000:1000