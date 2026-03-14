FROM ubuntu:24.04 AS smartfactoring_jazzy

# Retry transient apt downloads during large ROS installs.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

# Some Ubuntu-derived bases include a default UID 1000 user, the official image does not.
RUN if id -u ubuntu >/dev/null 2>&1; then \
      touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu; \
    fi

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Common packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    evince \
    gdb \
    git \
    gnupg \
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
    udev \
    usbutils \
    vim \
    wget \
    wireless-tools \
    xterm \
    liburdfdom-tools \
    socat \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 environment
ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Install prerequisites and enable universe
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      software-properties-common \
      ca-certificates && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 apt source helper
RUN set -eux; \
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' \
      | awk -F\" '{print $4}')" && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Install ROS 2 base packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep only once
RUN rosdep init || echo "rosdep already initialized"

# Install ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
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
    ros-${ROS_DISTRO}-test-msgs \
    && rm -rf /var/lib/apt/lists/*

################
# User
################

ARG USERNAME=smartfactoring_jazzy
ARG USER_UID=1000
ARG USER_GID=1000

# Create non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Configure shell for user
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${USERNAME}/.bashrc \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

################
# NVIDIA / OpenGL
################

RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libx11-6 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

################
# ROS 2 and hardware prerequisites
################

# Orbbec prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libudev-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    default-jdk \
    pkg-config \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Kinect prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-tracetools-image-pipeline \
    ros-${ROS_DISTRO}-depth-image-proc \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# MoveIt and related tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    jupyter-notebook \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-resources \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    ros-${ROS_DISTRO}-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Other ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control-cmake \
    ros-${ROS_DISTRO}-compressed-depth-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim-demos \
    ros-${ROS_DISTRO}-theora-image-transport \
    ros-${ROS_DISTRO}-zstd-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Lint / formatting
RUN apt-get update && apt-get install -y --no-install-recommends \
    pylint \
    ros-${ROS_DISTRO}-ament-lint \
    && rm -rf /var/lib/apt/lists/*

################
# Native libraries
################

COPY dependencies/libuvc /libuvc
RUN cd /libuvc && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install

COPY dependencies/libfreenect /libfreenect
RUN cd /libfreenect && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

################
# Graphics environment for Gazebo
################

RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-dri \
    libglapi-mesa \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libxi6 \
    libxtst6 \
    && rm -rf /var/lib/apt/lists/*

################
# Final setup
################

ENV DEBIAN_FRONTEND=
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN usermod -aG video ${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces
CMD ["bash"]
