# ROS 2 Kilted development environment for communicating with a Booster Studio
# virtual-robot container on native Ubuntu. Build context is this T2WS directory.
FROM osrf/ros:kilted-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git tmux python3-pip python3-colcon-common-extensions build-essential \
        ros-kilted-cv-bridge ros-kilted-teleop-twist-keyboard \
        ros-kilted-rosbag2 ros-kilted-rosbag2-storage-default-plugins \
        libgl1 libglib2.0-0 python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/t2ws-venv \
    && /opt/t2ws-venv/bin/python -m pip install --upgrade \
        pip wheel "setuptools>=68,<80"

ENV PATH="/opt/t2ws-venv/bin:${PATH}"

# Install optional application dependencies in /opt/t2ws-venv when they are needed.
# RUN pip3 install --no-cache-dir boosteros onnxruntime onnx matplotlib

# Booster's ROS 2 interface (msgs/srvs). Only the interface package is built:
# booster_ros2_example needs the Booster C++ SDK, which is not installed here.
RUN git clone --branch master https://github.com/BoosterRobotics/booster_robotics_sdk_ros2.git /opt/booster_ws/src/booster_robotics_sdk_ros2 \
    && . /opt/ros/kilted/setup.sh \
    && cd /opt/booster_ws && colcon build --packages-select booster_interface --cmake-args -DCMAKE_BUILD_TYPE=Release

# The Booster SDK needs this profiles file (participant "booster_dds") or it refuses to start.
COPY docker/fastdds_profiles.xml /opt/booster/fastdds_profiles.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/fastdds_profiles.xml

COPY docker/ros_env.sh /etc/ros_env.sh
RUN echo '. /etc/ros_env.sh' >> /etc/bash.bashrc
COPY docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh /etc/ros_env.sh \
    && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
