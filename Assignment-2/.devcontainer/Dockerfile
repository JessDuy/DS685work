FROM ros:humble-ros-core

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-sensor-msgs ros-humble-vision-msgs \
    libgl1 libglib2.0-0 ffmpeg git curl && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install --no-cache-dir opencv-python numpy psycopg2-binary Pillow

SHELL ["/bin/bash", "-lc"]
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
WORKDIR /workspaces/DS685work/Assignment-2/ros_ws
