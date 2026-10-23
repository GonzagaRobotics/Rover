# syntax=docker/dockerfile:1

# Build stage
FROM ros:jazzy AS build
SHELL ["/bin/bash", "-c"]
WORKDIR /Rover

RUN apt-get update && \
    apt-get install -y ros-jazzy-pcl-ros libx264-dev libyuv-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

COPY src ./src
RUN . /opt/ros/jazzy/setup.bash && colcon build
COPY launch ./launch

# Runtime stage
FROM ros:jazzy-ros-core AS runtime
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip ros-jazzy-pcl-ros libx264-164 libyuv0 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install onnxruntime opencv-python-headless -I --break-system-packages --no-cache-dir

WORKDIR /Rover

COPY --from=build /Rover/install ./install
COPY --from=build /Rover/launch ./launch

SHELL ["/bin/bash", "-c"]
USER 1000:1000