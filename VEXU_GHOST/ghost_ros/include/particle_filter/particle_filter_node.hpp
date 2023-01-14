#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "math/math_util.h"
#include "math/line2d.h"
#include "util/timer.h"
#include "yaml-cpp/yaml.h"

#include "globals/globals.hpp"
#include "particle_filter.hpp"

namespace particle_filter{

class ParticleFilterNode : public rclcpp::Node {
  public:

    ParticleFilterNode(std::string config_file);

    // Topic callback functions
    void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void OdometryCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    // Topic publish functions
    void PublishWorldTransform();
    void PublishVisualization();
    void PublishMapViz();
  
  private:
    void LoadConfiguration(std::string filename);
    void DrawParticles(geometry_msgs::msg::PoseArray &viz_msg);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_viz_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr world_tf_pub_;

    // Particle Filter
    ParticleFilter particle_filter_;
    sensor_msgs::msg::LaserScan last_laser_msg_;

    // Configuration
    YAML::Node config_yaml;
    ParticleFilterConfig config_params;

    bool first_map_load_;

};
} // namespace particle_filter