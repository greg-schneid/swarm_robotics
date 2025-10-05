FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/home/brain-streamer/brain_streamer_ros

# Install dependencies and tools for a full Ubuntu experience
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    vim \
    curl \
    wget \
    sudo \
    bash-completion \
    build-essential \
    bluez bluez-tools bluetooth rfkill dbus \
    python3-tk tk-dev libx11-6 libxext6 libxrender1 libxft2 libxinerama1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Install liblsl - required for muselsl
RUN wget -q https://github.com/sccn/liblsl/releases/download/v1.16.2/liblsl-1.16.2-noble_amd64.deb \
 && apt-get update \
 && apt-get install -y ./liblsl-1.16.2-noble_amd64.deb \
 && rm -f liblsl-1.16.2-noble_amd64.deb

# Make sure pylsl knows exactly where liblsl is
ENV PYLSL_LIB=/usr/lib/x86_64-linux-gnu/liblsl.so.1.16.2

# Install Python packages for Muse EEG streaming
RUN pip3 install --no-cache-dir --break-system-packages muselsl bleak numpy pylsl

# Initialize rosdep
RUN rosdep init || true

# Configure ROS environment for all users
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Default command
CMD ["bash"]