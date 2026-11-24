# Development image for the 3D multi-object tracker.
# ROS 2 Jazzy on Ubuntu 24.04 (Noble), with the C++ and Python dependencies
# needed to build tracker_core and run nusc_tools.
FROM ros:jazzy-ros-base

# Avoid interactive prompts (e.g. tzdata) during apt installs.
ENV DEBIAN_FRONTEND=noninteractive

# ---------- System and ROS dependencies ----------
# - libeigen3-dev:  linear algebra for the Kalman filter (header-only)
# - libgtest-dev:   GoogleTest for C++ unit tests
# - ament-cmake-gtest: CMake glue to register GoogleTest tests with colcon
# - vision-msgs:    Detection3DArray for detections and tracks
# - visualization-msgs: MarkerArray for RViz2/Foxglove
# - python3-pip:    to install nuscenes-devkit (not packaged by Ubuntu)
# - python3-pytest: test runner used by colcon for ament_python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        libgtest-dev \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pytest \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-vision-msgs \
        ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---------- Python dependencies ----------
# Installed into the system interpreter on purpose: ROS 2 Python nodes run
# with /usr/bin/python3, so a separate venv would not be visible to them.
# --break-system-packages is acceptable here because the container is
# disposable (PEP 668 protects the host, not throwaway images).
# numpy<2 keeps compatibility with the numpy shipped by Ubuntu/ROS.
# motmetrics + pandas are needed by nuscenes.eval.tracking. devkit 1.2.0 requires
# motmetrics==1.4.0 on Python >= 3.10 (see nuscenes/eval/tracking/mot.py).
RUN pip3 install --no-cache-dir --break-system-packages \
        "numpy<2" \
        nuscenes-devkit==1.2.0 \
        pyquaternion \
        motmetrics==1.4.0 \
        "pandas<2.3"

# ---------- Non-root user ----------
# Files created in the mounted repo (build/, install/, results/) must belong
# to the host user, not root. Pass the host UID/GID at build time:
#   docker build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) ...
# The Ubuntu 24.04 base image ships a default 'ubuntu' user with UID 1000,
# which would clash, so it is removed first.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

# ---------- Workspace and dataset ----------
# The repo root is the colcon workspace (it contains src/) and is mounted at /ws.
# The nuScenes dataset is mounted read-only at /data/nuscenes (never copied in).
ENV NUSC_ROOT=/data/nuscenes
WORKDIR /ws

USER ${USERNAME}
CMD ["bash"]
