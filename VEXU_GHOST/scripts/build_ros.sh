#!/bin/bash

# Assumes repository is in base directory
cd ~/VEXU_GHOST
echo "---Building Ghost ROS Packages---"

# Build simulator packages depending on what is passed for EMBEDBUILD
if [ "$1" == "EMBEDBUILD" ];
then 
    colcon build --packages-up-to ghost_ros
else
    colcon build
fi

source install/setup.bash

# Process URDFs from Xacro and add to Share
GHOST_DESCRIPTION_SHARE_DIR="$PWD/install/ghost_description/share/ghost_description"
URDF_PATH="${GHOST_DESCRIPTION_SHARE_DIR}/urdf/ghost1.urdf"

if [ ! -d $GHOST_DESCRIPTION_SHARE_DIR ];
then
    touch $URDF_PATH
else
    echo
    echo "---Generating Ghost Description URDF---"
    xacro ghost_description/urdf/ghost1.xacro > $URDF_PATH
    echo "URDF written to" $URDF_PATH
fi

# Processes URDFs for simulation
if [ "$1" != "EMBEDBUILD" ];
then 
    GHOST_SIM_SHARE_DIR="$PWD/install/ghost_sim/share/ghost_sim"
    URDF_SIM_VOLTAGE_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_voltage.urdf"
    URDF_SIM_PID_PATH="${GHOST_SIM_SHARE_DIR}/urdf/ghost1_sim_pid.urdf"

    if [ ! -d $GHOST_SIM_SHARE_DIR ];
    then
        touch $URDF_SIM_VOLTAGE_PATH
        touch $URDF_SIM_PID_PATH
    else
        echo
        echo "---Generating Ghost Simulation URDFs---"
        xacro ghost_sim/urdf/ghost1_sim_voltage.xacro > $URDF_SIM_VOLTAGE_PATH
        xacro ghost_sim/urdf/ghost1_sim_pid.xacro > $URDF_SIM_PID_PATH
        echo "URDF written to" $URDF_SIM_VOLTAGE_PATH
        echo "URDF written to" $URDF_SIM_PID_PATH
    fi
fi

source ~/.bashrc
