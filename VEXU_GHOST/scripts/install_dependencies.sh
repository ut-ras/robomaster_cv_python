#!/bin/bash
# # https://launchpad.net/~openrobotics/+archive/ubuntu/gazebo11-non-amd64
sudo add-apt-repository -y ppa:openrobotics/gazebo11-non-amd64
sudo apt update

# # https://github.com/RobotLocomotion/drake-ci/pull/217/files
sudo apt-get install -o Dpkg::Use-Pty=0 -y --allow-downgrades libegl1 libegl-mesa0=22.0.1-1ubuntu2 libgbm1=22.0.1-1ubuntu2 \
    libgl1-mesa-dri=22.0.1-1ubuntu2 libglapi-mesa=22.0.1-1ubuntu2 libglx-mesa0=22.0.1-1ubuntu2
apt-mark hold libegl-mesa0 libgbm1 libgl1-mesa-dri libglapi-mesa libglx-mesa0
sudo apt-get install -y libgazebo-dev gazebo
sudo apt-get install -y libgoogle-glog-dev python3-colcon-common-extensions ros-${ROS_DISTRO}-xacro
sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-rqt-tf-tree ros-${ROS_DISTRO}-joy-linux
sudo apt-get install -y python3-pip libgtest-dev libgoogle-glog-dev
sudo apt-get install -y ros-${ROS_DISTRO}-rplidar-ros  ros-${ROS_DISTRO}-realsense2*
sudo apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers

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
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros
colcon build --symlink-install
