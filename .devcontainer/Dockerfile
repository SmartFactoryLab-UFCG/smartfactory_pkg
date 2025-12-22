FROM ubuntu:24.04 AS smartfactoring_jazzy

# Remove default user Ubuntu from UID 1000
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    curl \
    evince \
    gdb \
    git \
    gnupg2 \
    htop \
    iftop \
    iproute2 \
    iputils-ping \
    lsb-release \
    nano \
    net-tools \
    openssh-client \
    psmisc \
    python3-argcomplete \
    python3-pip \
    software-properties-common \
    sudo \
    traceroute \
    terminator \
    vim \
    wget \
    xterm \
    wireless-tools \
    usbutils \
    liburdfdom-tools \
    socat \
    && rm -rf /var/lib/apt/lists/*

# Environment variables for ROS 2
ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.11/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# install prerequisites, add universe
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      software-properties-common \
      ca-certificates \
      && \
    add-apt-repository universe

# Install ROS2
# download, unpack & install the ros2-apt-source helper package
RUN set -eux; \
    ROS_APT_SOURCE_VERSION="$(curl -s \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' \
      | awk -F\" '{print $4}')" && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# now you have the ROS2 APT repo & key installed, next:
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Install ROS 2 dependencies
RUN sudo apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-force-torque-sensor-broadcaster \
    ros-${ROS_DISTRO}-pose-broadcaster \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-moveit-task-constructor-core \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite \
    ros-${ROS_DISTRO}-test-msgs 

################
# Gazebo
################
# install packages
RUN apt-get update && apt-get install -q -y \
    gnupg \
  && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -y -q \
    gz-harmonic \
  && rm -rf /var/lib/apt/lists/*

# Get sources
RUN wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/refs/heads/master/collection-harmonic.yaml \
  && vcs import < collection-harmonic.yaml \
  # Get dependencies
  && sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && sudo apt-get update \
  && sudo apt -y install \
      $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

################
# Initialization
################
RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=smartfactoring_jazzy
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get install -y git-core \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libegl1 \
    libglvnd0 \
    libglx0 \
    libgl1 \
    libxext6 \
    libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

## Install CUDA toolkit
## Not necessary if nvidia-container-toolkit is installed in host machine
# RUN apt-get update && apt-get install -y \
#     cuda-toolkit-11-8 \
#     && rm -rf /var/lib/apt/lists/*

################
# ROS 2 and hardware prerequisites
################

# Orbbec prerequisites
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libudev-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openjdk-11-jdk \
    pkg-config \
    ros-${ROS_DISTRO}-cv-bridge
    
# Kinect prerequisites
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-tracetools-image-pipeline \
    ros-${ROS_DISTRO}-depth-image-proc

RUN apt-get install -y \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-tf-transformations

# ArUco prerequisites 
#RUN apt-get update && \
#    apt-get install -y \
#        python3-opencv && \
#    pip3 install --no-cache-dir transforms3d && \
#    rm -rf /var/lib/apt/lists/*

# MoveIt install
RUN apt-get install -y jupyter-notebook

RUN apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-resources \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rviz-visual-tools

RUN apt-get install -y \
    ros-${ROS_DISTRO}-ament-clang-format \
    ros-${ROS_DISTRO}-ros-testing \
    ros-${ROS_DISTRO}-picknik-twist-controller \
    ros-${ROS_DISTRO}-picknik-reset-fault-controller \
    ros-${ROS_DISTRO}-py-binding-tools \
    ros-${ROS_DISTRO}-ament-clang-tidy \
    ros-${ROS_DISTRO}-launch-pytest \
    ros-${ROS_DISTRO}-rmf-utils \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-ament-cmake-google-benchmark \
    ros-${ROS_DISTRO}-graph-msgs \
    ros-${ROS_DISTRO}-ros2-control

# Other
RUN apt-get install -y \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control-cmake \
    ros-${ROS_DISTRO}-compressed-depth-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim-demos \
    ros-${ROS_DISTRO}-theora-image-transport \
    ros-${ROS_DISTRO}-zstd-image-transport

# Code Style formating
RUN apt-get update && \
    apt-get install -y pylint && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get install -y ros-${ROS_DISTRO}-ament-lint

# Copy the libuvc directory from your host to the Docker image
COPY dependencies/libuvc /libuvc

# Build and install libuvc from the copied source
RUN cd /libuvc && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

COPY dependencies/libfreenect /libfreenect

RUN cd /libfreenect && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig
    
# Update the library path
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Graphics Environment for Gazebo
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-dri \
    libglapi-mesa \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libxi6 \
    libxtst6

# Standard init procedure
ENV DEBIAN_FRONTEND=

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

RUN usermod -aG video $USERNAME

################
# Colcon Build - Build inside dockerfile to assure integration
# Temporary solution. This build and test will be moved to CI.
################
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get -y autoremove && \
    apt-get -y clean

ENV DEBIAN_FRONTEND=

CMD ["bash"]

