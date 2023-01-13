#!/bin/bash

distribution=${1:-noetic} 

apt-get install -y ros-$distribution-rosserial_python
apt-get install -y ros-$distribution-driver-base
apt-get install -y ros-$distribution-moveit
apt-get install -y ros-$distribution-moveit-commander
apt-get install -y ros-$distribution-moveit-setup-assistant-dbgsym
apt-get install -y ros-$distribution-dynamixel-workbench-msgs
apt-get install -y ros-$distribution-dynamixel-workbench-controllers
