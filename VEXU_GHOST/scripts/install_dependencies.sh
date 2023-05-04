#!/bin/bash
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions ros-foxy-xacro ros-foxy-gazebo-ros ros-foxy-gazebo-ros-pkgs
sudo apt-get install -y ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-rqt-tf-tree ros-foxy-joy-linux
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev
sudo apt-get install -y ros-foxy-rplidar-ros  ros-foxy-realsense2*
sudo apt-get install -y ros-foxy-gazebo-ros2-control ros-foxy-ros2-controllers libgazebo11-dev gazebo11

cd ~/VEXU_GHOST

echo "Building yaml-cpp from source in ~/ghost_deps"
cd ~/
mkdir ghost_deps && cd ghost_deps
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j
sudo make install

cd ~/VEXU_GHOST/ghost_deps
git clone -b ros2 git@github.com:Slamtec/rplidar_ros.git
cd rplidar_ros
colcon build --symlink-install
