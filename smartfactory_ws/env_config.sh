#!/bin/bash

# Inicializa rosdep se necessário
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "Inicializando rosdep..."
  sudo rosdep init
fi
rosdep update

# Sourcing do setup do ROS 2
echo "Executando sourcing do setup do ROS 2..."
source /opt/ros/humble/setup.bash

# Verifica e realiza o echo das variáveis de ambiente do ROS 2 se ainda não estiverem presentes
if ! grep -q "export ROS_DISTRO=humble" ~/.bashrc; then
  echo "Adicionando configurações ao .bashrc..."
  echo "export ROS_DISTRO=humble" >> ~/.bashrc
  echo "export AMENT_PREFIX_PATH=/opt/ros/\$ROS_DISTRO" >> ~/.bashrc
  echo "export COLCON_PREFIX_PATH=/opt/ros/\$ROS_DISTRO" >> ~/.bashrc
  echo "export LD_LIBRARY_PATH=/opt/ros/\$ROS_DISTRO/lib" >> ~/.bashrc
  echo "export PATH=/opt/ros/\$ROS_DISTRO/bin:\$PATH" >> ~/.bashrc
  echo "export PYTHONPATH=/opt/ros/\$ROS_DISTRO/lib/python3.10/site-packages" >> ~/.bashrc
  echo "export ROS_PYTHON_VERSION=3" >> ~/.bashrc
  echo "export ROS_VERSION=2" >> ~/.bashrc
  echo "export TURTLEBOT_3D_SENSOR=astra_pro" >> ~/.bashrc
  echo "export TURTLEBOT_3D_SENSOR2=none" >> ~/.bashrc
  echo "export TURTLEBOT_BATTERY=none" >> ~/.bashrc
  echo "export TURTLEBOT_STACKS=interbotix" >> ~/.bashrc
  echo "export TURTLEBOT_BASE=kobuki" >> ~/.bashrc
  echo "export TURTLEBOT_ARM=none" >> ~/.bashrc
  echo "export TURTLEBOT_SERIAL_PORT=/dev/kobuki" >> ~/.bashrc
  echo "export TURTLEBOT_LIDAR_SENSOR=rplidar_a3" >> ~/.bashrc
else
  echo "Configurações do ROS 2 já estão presentes no .bashrc"
fi

# Definir o workspace do ROS 2
WORKSPACE="$HOME/robotxr/turtlebot2i_ws"

# Navegar para o workspace
cd $WORKSPACE

# Instalar dependências de pacotes no workspace (se necessário)
echo "Instalando dependências do workspace..."
rosdep install --from-paths src --ignore-src -r -y

# Construindo o workspace
echo "Construindo o workspace..."
colcon build --symlink-install --executor sequential

# Configuração de permissões e usuários (se necessário)
echo "Configurando permissões e usuários..."
USERNAME=robotxr
if id "$USERNAME" &>/dev/null; then
  echo "Usuário $USERNAME já existe."
else
  sudo useradd -m $USERNAME
  sudo usermod -aG video $USERNAME
fi

# Adicionando regras do udev
echo "Adicionando regras do udev..."

sudo chmod 666 /dev/ttyUSB*

# Kobuki
wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules
sudo cp 60-kobuki.rules /etc/udev/rules.d
rm "$WORKSPACE/60-kobuki.rules"

# RPLidar
if [ -d "$WORKSPACE/src/External_Packages/sllidar_ros2/scripts/" ]; then
  cd "$WORKSPACE/src/External_Packages/sllidar_ros2/scripts/"
  source create_udev_rules.sh 
else
  echo "Diretório $WORKSPACE/src/External_Packages/sllidar_ros2/scripts/ não encontrado."
fi

# Astra Pro
if [ -d "$WORKSPACE/src/External_Packages/ros2_astra_camera/astra_camera/scripts" ]; then
  cd "$WORKSPACE/src/External_Packages/ros2_astra_camera/astra_camera/scripts"
  sudo bash install.sh
else
  echo "Diretório "$WORKSPACE/src/External_Packages/ros2_astra_camera/astra_camera/scripts" não encontrado."
fi

# Recarregar as regras do udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# Limpar arquivos temporários e cache de pacotes
echo "Limpando arquivos temporários e cache de pacotes..."
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "Configuração do ambiente concluída com sucesso!"