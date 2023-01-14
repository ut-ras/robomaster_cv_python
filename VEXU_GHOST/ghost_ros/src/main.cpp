/*
 * Filename: main.cpp
 * Created Date: Monday October 24th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Friday October 28th 2022 3:45:21 pm
 * Modified By: Maxx Wilson
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "globals/globals.hpp"

#include "particle_filter/particle_filter_node.hpp"

using namespace std::literals::chrono_literals;

// Define Global Variables in shared memory
namespace globals{
    std::string repo_base_dir;
    bool run_ = true;
}


void SignalHandler(int) {
    if(rclcpp::ok()){
        rclcpp::shutdown();
    }

    if (!globals::run_) {
        printf("Force Exit.\n");
        exit(0);
    }

    printf("Exiting.\n");
    globals::run_ = false;
}

void particle_filter_main(){
    rclcpp::spin(std::make_shared<particle_filter::ParticleFilterNode>(globals::repo_base_dir + "ghost_ros/config/particle_filter.yaml"));
}

int main(int argc, char* argv[]){
    signal(SIGINT, SignalHandler);
    rclcpp::init(argc, argv);

    globals::repo_base_dir = getenv("HOME");
    globals::repo_base_dir += + "/VEXU_GHOST/";

    // Initialize modules
    std::thread particle_filter_thread(&particle_filter_main);    

    particle_filter_thread.join();

    return 0;
}