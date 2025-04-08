#!/bin/bash

locale
read -p "Do you want to update your locale to en_SG.UTF-8 (y/n)?" input
if ["$input" = "y"];
    then
        sudo apt update
        sudo apt install locales
        sudo locale-gen en_SG en_SG.UTF-8
        sudo update-locale LC_ALL=en_SG.UTF-8 LANG=en_SG.UTF-8
        export LANG=en_SG.UTF-8
        locale

else
    echo "Proceeding without updating locale."

fi

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt upgrade

sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
# echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
