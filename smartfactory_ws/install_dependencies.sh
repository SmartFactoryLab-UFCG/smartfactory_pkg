#!/bin/bash

# Atualiza o sistema
sudo apt-get update && sudo apt-get upgrade -y

# Adiciona os repositórios necessários do Ubuntu
echo "Adicionando repositórios do Ubuntu..."
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt-get update

# Instala pacotes básicos
echo "Instalando pacotes básicos..."
sudo apt-get install -y curl wget vim git locales bash-completion \
            build-essential cmake gdb gnupg2 iproute2 lsb-release \
            nano net-tools openssh-client psmisc python3-pip python3-dev \
            software-properties-common sudo terminator xterm \
            wireless-tools iftop htop traceroute

# Adicionar e configurar repositório do ROS 2
echo "Adicionando o repositório do ROS 2..."
sudo apt-add-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ros-archive-keyring.gpg
sudo mv ros-archive-keyring.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

ROS_DISTRO="humble"

echo "The ROS Distro is: ${ROS_DISTRO}"

# Instalar ROS 2
echo "Instalando ROS 2..."
sudo apt-get update
sudo apt-get install -y python3-argcomplete ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-desktop

# Instalar rosdep
sudo apt-get install -y python3-rosdep
sudo rosdep init
rosdep update

# Instalar colcon
sudo apt-get install -y python3-colcon-common-extensions

# Instalar pacotes ROS 2 adicionais necessários para build e rosbag
sudo apt-get install -y ros-${ROS_DISTRO}-rosbag2* ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-rosbag2-storage-default-plugins

# Configuração de variáveis de ambiente para ROS 2
echo "Configurando variáveis de ambiente para ROS 2..."
echo "export ROS_DISTRO=humble" >> ~/.bashrc
echo "export AMENT_PREFIX_PATH=/opt/ros/humble" >> ~/.bashrc
echo "export COLCON_PREFIX_PATH=/opt/ros/humble" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/opt/ros/humble/lib" >> ~/.bashrc
echo "export PATH=/opt/ros/humble/bin:$PATH" >> ~/.bashrc
echo "export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages" >> ~/.bashrc
echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
echo "export ROS_VERSION=2" >> ~/.bashrc

source ~/.bashrc

# Atualiza novamente após adicionar repositórios ROS 2
sudo apt-get update


# Instalar dependências ROS 2
sudo apt-get install -y --no-install-recommends python3-argcomplete ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-desktop ecl ecl-doc \
                        ros-${ROS_DISTRO}-kobuki* yocs* ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-ros-gz \
                        ros-${ROS_DISTRO}-ros-testing ros-${ROS_DISTRO}-sophus ros-${ROS_DISTRO}-gazebo-ros \
                        ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-plotjuggler ros-${ROS_DISTRO}-tf-transformations \
                        ament-cmake ament-lint ros-dev-tools ros-${ROS_DISTRO}-ament-*

                        
# Verifica se o pacote foi instalado com sucesso, caso contrário, tenta instalar novamente
for pkg in ros-${ROS_DISTRO}-sophus ros-${ROS_DISTRO}-test-msgs ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp; do
    if ! dpkg -l | grep -q $pkg; then
        echo "Tentando instalar $pkg novamente..."
        sudo apt-get install -y $pkg || echo "Falha ao instalar $pkg"
    fi
done

# Instalar outras dependências específicas de ROS e ferramentas adicionais
echo "Instalando outras dependências específicas de ROS e ferramentas adicionais..."
sudo apt-get install -y git bash-completion apt-transport-https patchelf
sudo apt-get install -y build-essential libeigen3-dev libgflags-dev libgoogle-glog-dev \
                        libudev-dev libusb-1.0-0-dev nlohmann-json3-dev openjdk-11-jdk pkg-config \
                        ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-camera-info-manager \
                        ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-publisher \
                        ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-point-cloud-transport \
                        ros-${ROS_DISTRO}-point-cloud-transport-plugins \
                        ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-eigen ros-${ROS_DISTRO}-tf2-sensor-msgs \
                        ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-apriltag-msgs
                        


# Instalar Gazebo Ignition e suas dependências
echo "Instalando Gazebo Ignition e suas dependências..."
wget https://packages.osrfoundation.org/gazebo.gpg -O pkgs-osrf-archive-keyring.gpg
sudo mv pkgs-osrf-archive-keyring.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y -q ignition-fortress \
    libignition-plugin-dev \
    libignition-cmake2-dev \
    libignition-gazebo6-dev

echo "Instalando dependências para OpenGL e X11..."
sudo apt-get install -y --no-install-recommends \
  libegl1 libglvnd0 libglx0 libgl1 libxext6 libx11-6
                        
echo "Construindo e instalando libuvc..."

# Get the directory where the script is located
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Navigate to the parent directory of the script's location
PARENT_DIR=$(dirname "$SCRIPT_DIR")

# Ajustar o caminho do diretório libuvc se for necessário
cd ${PARENT_DIR}/dependencies/libuvc
mkdir build -p
cd build
cmake ..
make
sudo make install

# Limpeza
echo "Limpando arquivos temporários..."
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

# Instala pacotes ROS 2 individuais
sudo apt-get update
echo "Instalando pacotes ROS 2 adicionais..."
sudo apt-get install -y ros-${ROS_DISTRO}-sophus || echo "Falha ao instalar ros-${ROS_DISTRO}-sophus"
sudo apt-get install -y ros-${ROS_DISTRO}-rtabmap-ros || echo "Falha ao instalar ros-${ROS_DISTRO}-rtabmap-ros"
sudo apt-get install -y ros-${ROS_DISTRO}-turtlebot4-simulator || echo "Falha ao instalar ros-${ROS_DISTRO}-turtlebot4-simulator"
sudo apt-get install -y ros-${ROS_DISTRO}-test-msgs || echo "Falha ao instalar ros-${ROS_DISTRO}-test-msgs"
sudo apt-get install -y ros-${ROS_DISTRO}-cyclonedds || echo "Falha ao instalar ros-${ROS_DISTRO}-cyclonedds"
sudo apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp || echo "Falha ao instalar ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"

sudo apt-get install -y ros-${ROS_DISTRO}-urg-node
sudo apt-get install -y ros-${ROS_DISTRO}-robot-localization
sudo apt-get install -y ros-${ROS_DISTRO}-kobuki-velocity-smoother
sudo apt-get install -y ros-${ROS_DISTRO}-ros-testing
sudo apt-get install -y ros-${ROS_DISTRO}-camera-info-manager
sudo apt-get install -y ros-${ROS_DISTRO}-image-publisher

sudo apt-get install -y ros-${ROS_DISTRO}-ros-testing

# Instalar ecl_build separadamente
echo "Instalando ecl_build separadamente..."
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-ecl-build

# Instalar ferramentas adicionais
sudo apt-get update
sudo apt-get install -y wireless-tools || echo "Falha ao instalar wireless-tools"
sudo apt-get install -y iftop || echo "Falha ao instalar iftop"
sudo apt-get install -y htop || echo "Falha ao instalar htop"
sudo apt-get install -y traceroute || echo "Falha ao instalar traceroute"

# Instalar dotnet-sdk-6.0
sudo apt-get update && sudo apt-get install -y dotnet-sdk-6.0

# Instalar pacotes ROS 2 adicionais caso não tenham sido instalados anteriormente
for pkg in ros-${ROS_DISTRO}-test-msgs ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp; do
    if ! dpkg -l | grep -q $pkg; then
        echo "Tentando instalar $pkg novamente..."
        sudo apt-get install -y $pkg || echo "Falha ao instalar $pkg"
    fi
done

# Limpeza final
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*