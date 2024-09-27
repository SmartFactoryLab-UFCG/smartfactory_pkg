#############################################
# Created from template athackst.humble.Dockerfile
#############################################

FROM ubuntu:22.04 as smartfactory-lab

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    python3-argcomplete \
    ros-humble-ros-base \
    ros-humble-desktop \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ecl* \
    ecl-exceptions* \
    ecl-threads* \
    ecl-geometry* \
    ecl-streams* \
    kobuki* \
    yocs* \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-gz \
    ros-humble-ros-testing \
    ros-${ROS_DISTRO}-sophus \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ament-cmake \
    ament-lint \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    gnupg2 \
    iproute2 \
    lsb-release \
    nano \
    net-tools \
    openssh-client \
    psmisc \
    python3-argcomplete \
    python3-pip \
    ros-dev-tools \
    ros-$ROS_DISTRO-ament-* \
    software-properties-common \
    sudo \
    terminator \
    vim \
    wget \
    xterm \
    wireless-tools \
    iftop \
    htop \
    traceroute \
    iputils-ping \
    evince \
    && rm -rf /var/lib/apt/lists/*


RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=smartfactory-lab
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
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 


# Install gazebo standard
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-gazebo* \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libegl1 \
  libglvnd0 \
  libglx0 \
  libgl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

################
# RobotXR ROS packages
################

# ROS2-for-unity prerequisites

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-fastrtps ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

RUN curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | bash && \
    apt-get update && \
    apt-get install -y python3-vcstool

RUN apt-get update && apt-get install -y \
    apt-transport-https && \
    apt-get update && apt-get install -y \
    dotnet-sdk-6.0

RUN apt install patchelf && \
    rm -rf /var/lib/apt/lists/* 

# Orbbec prerequisites

RUN apt-get update && apt-get install -y \
    build-essential \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libudev-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openjdk-11-jdk \
    pkg-config \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-image-transport-plugins \ 
    ros-${ROS_DISTRO}-point-cloud-transport \ 
    ros-${ROS_DISTRO}-point-cloud-interfaces \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-cv-bridge && \
    rm -rf /var/lib/apt/lists/*

# Kinecy prerequisites

RUN apt-get update && apt-get install -y \
    libturbojpeg0-dev \
    libglfw3-dev   \
    freenect \
    libfreenect-dev \
    libfreenect-demos && \
    rm -rf /var/lib/apt/lists/*
 
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

# Pylon dependencies
COPY dependencies/libpylon /libpylon

RUN cd /libpylon && \
    apt-get update && \
    apt-get install -y \
    ./pylon_7.5.0.15658-deb0_amd64.deb \
    ./pylon-supplementary-package-for-blaze-1.7.0.35c9275fa_amd64.deb && \
    rm -rf /var/lib/apt/lists/*

# Update the library path
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Aruco prerequisites

RUN apt-get update && \
    pip3 install opencv-python opencv-contrib-python transforms3d && \
    apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    pip install scikit-fuzzy &&\
    rm -rf /var/lib/apt/lists/*

# Standard init procedure

ENV DEBIAN_FRONTEND=

ENV TURTLEBOT_3D_SENSOR=astra_pro
ENV TURTLEBOT_3D_SENSOR2=none
ENV TURTLEBOT_BATTERY=none
ENV TURTLEBOT_STACKS=interbotix
ENV TURTLEBOT_BASE=kobuki
ENV TURTLEBOT_ARM=none
ENV TURTLEBOT_SERIAL_PORT=/dev/kobuki
ENV TURTLEBOT_LIDAR_SENSOR=rplidar_a3

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

RUN usermod -aG video $USERNAME

################
# Colcon Build RobotXR - Build inside dockerfile to assure integration
# Temporary solution. This build and test will be moved to CI.
################
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get -y autoremove && \
    apt-get -y clean

# Accept the WORKSPACE build argument
ARG WORKSPACE

# Set the working directory to the WORKSPACE argument value
WORKDIR $WORKSPACE

# Copy the ROS workspace into the container
COPY smartfactory_ws/ $WORKSPACE/

RUN rosdep init || echo "rosdep already initialized" && \
    sudo rosdep fix-permissions && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-get update && \
#     rm -rf build/ install/ log/ && \
#     . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /setup/entrypoint.sh

ENTRYPOINT ["/bin/bash", "/setup/entrypoint.sh"]

CMD ["bash"]
