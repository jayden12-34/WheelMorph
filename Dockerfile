FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PATH=/opt/ros/humble/bin:$PATH

# Install essential apt packages (minimal to avoid pip/apt conflicts)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates build-essential \
    git wget locales sudo iputils-ping net-tools procps \
    python3-pip python3-setuptools python3-wheel \
    python3-argcomplete python3-apt python3-distutils \
    python3-empy python3-numpy \
    python3-tk \
    python3-serial \
    libpython3-dev pkg-config cmake unzip jq \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    libglib2.0-0 libsm6 libxrender1 libxext6 \
    libopencv-dev python3-opencv \
    xauth x11-apps dbus-x11 \
    udev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# Add ROS 2 apt repository and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 base and CycloneDDS RMW implementation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true \
 && rosdep update --rosdistro humble || true

# Install dev/build tools via pip (no conflicting ros Python packages)
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir \
    colcon-common-extensions \
    vcstool \
    rosinstall-generator \
    argcomplete \
    empy \
    setuptools \
    opencv-python-headless \
    dynamixel-sdk \
    pyserial \
    keyboard || true

# Create non-root user (default UID/GID 1000; override at build-time with build args)
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd -g ${USER_GID} ${USERNAME} || true \
 && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} || true \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME} \
 && chmod 0440 /etc/sudoers.d/90-${USERNAME} \
 && usermod -a -G dialout ${USERNAME} || true

# Workspace directory
RUN mkdir -p /workspace && chown -R ${USER_UID}:${USER_GID} /workspace

# Copy entrypoint (ensure entrypoint.sh exists next to Dockerfile)
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Source ROS in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

VOLUME ["/workspace","/tmp/.X11-unix","/tmp/.docker-xauth"]
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
