#!/bin/bash

# Exit on any error
set -e

# Update package list and install prerequisites
echo "Updating package list and installing prerequisites..."
sudo apt-get update

# Add Docker's official GPG key
echo "Adding Docker's official GPG key..."
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the Docker repository
echo "Setting up Docker repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package list again
echo "Updating package list again..."
sudo apt-get update

# Install Docker Engine, CLI, and Containerd
echo "Installing Docker Engine, CLI, and Containerd..."
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to the Docker group
echo "Adding user to the Docker group..."
sudo usermod -aG docker $USER

echo "Docker installation and setup completed successfully!"

# Update package list and install dependencies
echo "Updating package list and installing dependencies..."
sudo apt-get update
sudo apt-get install -y wget gpg

# Download and add Microsoft GPG key
echo "Adding Microsoft GPG key..."
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/

# Add the Visual Studio Code repository
echo "Adding Visual Studio Code repository..."
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list

# Update package list again
echo "Updating package list again..."
sudo apt-get update

# Install Visual Studio Code
echo "Installing Visual Studio Code..."
sudo apt-get install -y code

# Clean up
echo "Cleaning up..."
rm -f packages.microsoft.gpg

echo "Visual Studio Code installation completed successfully!"

# Definir o workspace do ROS 2
WORKSPACE=~/dev/robotxr/turtlebot2i_ws

# Adicionando regras do udev
echo "Adicionando regras do udev..."

# Kobuki
wget https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules
sudo cp 60-kobuki.rules /etc/udev/rules.d
if [ -e /dev/ttyUSB0 ]; then
  sudo chmod 666 /dev/ttyUSB0
  ls -l /dev/kobuki
else
  echo "/dev/ttyUSB0 não encontrado. Verifique a conexão do dispositivo Kobuki."
fi

rm $WORKSPACE/60-kobuki.rules

# RPLidar
if [ -e /dev/ttyUSB1 ]; then
  sudo chmod 777 /dev/ttyUSB1
  if [ -d ~/sllidar_ros2/scripts/ ]; then
    cd $WORKSPACE/src/External_Packages/sllidar_ros2/scripts/
    source create_udev_rules.sh 
    ls -l /dev/ttyUSB1
  else
    echo "Diretório $WORKSPACE/src/External_Packages/sllidar_ros2/scripts/ não encontrado."
  fi
else
  echo "/dev/ttyUSB1 não encontrado. Verifique a conexão do dispositivo RPLidar."
fi

# Adicionando regra udev para renomear RPLidar
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules

# Astra Pro
if [ -d $WORKSPACE/src/External_Packages/ros2_astra_camera/astra_camera/scripts ]; then
  cd $WORKSPACE/src/External_Packages/ros2_astra_camera/astra_camera/scripts
  sudo bash install.sh
else
  echo "Diretório $WORKSPACE/src//External_Packages/ros2_astra_camera/astra_camera/scripts não encontrado."
fi

# Recarregar as regras do udev
sudo udevadm control --reload-rules
sudo udevadm trigger