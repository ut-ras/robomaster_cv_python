//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/line2d.h"
#include "util/timer.h"
#include "ghost_ros/robot_config/v5_port_config.hpp"
#include "ghost_ros/ros_nodes/sentry_estimator_node.hpp"
#include "ghost_util/angle_util.hpp"

using Eigen::Vector2f;
using geometry::Line;
using geometry::Line2f;
using math_util::DegToRad;
using math_util::RadToDeg;
using particle_filter::ParticleFilter;
using particle_filter::ParticleFilterConfig;
using std::string;
using std::vector;
using std::placeholders::_1;

namespace ghost_ros
{

  SentryEstimatorNode::SentryEstimatorNode() : Node("sentry_estimator_node")
  {
    // Loads configuration from ROS Parameters
    LoadROSParams();

    // Use simulated time in ROS
    rclcpp::Parameter use_sim_time_param("use_sim_time", false);
    this->set_parameter(use_sim_time_param);

    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&SentryEstimatorNode::LaserCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&SentryEstimatorNode::OdomCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initial_pose",
        10,
        std::bind(&SentryEstimatorNode::InitialPoseCallback, this, _1));

    auto map_qos = rclcpp::QoS(10);
    map_qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Publishers
    cloud_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);
    map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_viz", map_qos);
    debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("estimation_debug", 10);
    estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("estimated_robot_pose", 10);

    // Init debug msg
    viz_msg_ = visualization_msgs::msg::MarkerArray{};

    particle_filter_ = ParticleFilter(config_params);
    first_map_load_ = true;
    laser_msg_received_ = false;

    const Vector2f init_loc(config_params.init_x, config_params.init_y);
    const float init_angle = config_params.init_r;

    odom_loc_(config_params.init_x, config_params.init_y);
    odom_angle_ = config_params.init_r;
    
    particle_filter_.Initialize(config_params.map, init_loc, config_params.init_r);
  }

  void SentryEstimatorNode::LoadROSParams()
  {
    // // Particle Filter
    config_params = ParticleFilterConfig();

    declare_parameter("particle_filter.map", "");
    config_params.map = get_parameter("particle_filter.map").as_string();

    declare_parameter("particle_filter.init_x", 0.0);
    declare_parameter("particle_filter.init_y", 0.0);
    declare_parameter("particle_filter.init_r", 0.0);
    config_params.init_x = get_parameter("particle_filter.init_x").as_double();
    config_params.init_y = get_parameter("particle_filter.init_y").as_double();
    config_params.init_r = get_parameter("particle_filter.init_r").as_double();

    declare_parameter("particle_filter.resample_frequency", 1);
    config_params.resample_frequency = get_parameter("particle_filter.resample_frequency").as_int();

    declare_parameter("particle_filter.init_x_sigma", 0.0);
    declare_parameter("particle_filter.init_y_sigma", 0.0);
    declare_parameter("particle_filter.init_r_sigma", 0.0);
    config_params.init_x_sigma = get_parameter("particle_filter.init_x_sigma").as_double();
    config_params.init_y_sigma = get_parameter("particle_filter.init_y_sigma").as_double();
    config_params.init_r_sigma = get_parameter("particle_filter.init_r_sigma").as_double();

    declare_parameter("particle_filter.k1", 0.0);
    declare_parameter("particle_filter.k2", 0.0);
    declare_parameter("particle_filter.k3", 0.0);
    declare_parameter("particle_filter.k4", 0.0);
    declare_parameter("particle_filter.k5", 0.0);
    declare_parameter("particle_filter.k6", 0.0);
    declare_parameter("particle_filter.k7", 0.0);
    declare_parameter("particle_filter.k8", 0.0);
    declare_parameter("particle_filter.k9", 0.0);
    config_params.k1 = get_parameter("particle_filter.k1").as_double();
    config_params.k2 = get_parameter("particle_filter.k2").as_double();
    config_params.k3 = get_parameter("particle_filter.k3").as_double();
    config_params.k4 = get_parameter("particle_filter.k4").as_double();
    config_params.k5 = get_parameter("particle_filter.k5").as_double();
    config_params.k6 = get_parameter("particle_filter.k6").as_double();
    config_params.k7 = get_parameter("particle_filter.k7").as_double();
    config_params.k8 = get_parameter("particle_filter.k8").as_double();
    config_params.k9 = get_parameter("particle_filter.k9").as_double();

    declare_parameter("particle_filter.laser_offset", 0.0);
    declare_parameter("particle_filter.laser_angle_offset", 0.0);
    declare_parameter("particle_filter.min_update_dist", 0.0);
    declare_parameter("particle_filter.min_update_angle", 0.0);
    config_params.laser_offset = get_parameter("particle_filter.laser_offset").as_double();
    config_params.laser_angle_offset = get_parameter("particle_filter.laser_angle_offset").as_double();
    config_params.min_update_dist = get_parameter("particle_filter.min_update_dist").as_double();
    config_params.min_update_angle = get_parameter("particle_filter.min_update_angle").as_double();

    declare_parameter("particle_filter.sigma_observation", 0.0);
    declare_parameter("particle_filter.gamma", 0.0);
    declare_parameter("particle_filter.dist_short", 0.0);
    declare_parameter("particle_filter.dist_long", 0.0);
    declare_parameter("particle_filter.range_min", 0.0);
    declare_parameter("particle_filter.range_max", 0.0);
    declare_parameter("particle_filter.resize_factor", 0.0);
    declare_parameter("particle_filter.num_particles", 50);
    config_params.sigma_observation = get_parameter("particle_filter.sigma_observation").as_double();
    config_params.gamma = get_parameter("particle_filter.gamma").as_double();
    config_params.dist_short = get_parameter("particle_filter.dist_short").as_double();
    config_params.dist_long = get_parameter("particle_filter.dist_long").as_double();
    config_params.range_min = get_parameter("particle_filter.range_min").as_double();
    config_params.range_max = get_parameter("particle_filter.range_max").as_double();
    config_params.resize_factor = get_parameter("particle_filter.resize_factor").as_double();
    config_params.num_particles = get_parameter("particle_filter.num_particles").as_int();

    declare_parameter("particle_filter.use_skip_range", false);
    declare_parameter("particle_filter.skip_index_min", 0);
    declare_parameter("particle_filter.skip_index_max", 0);
    config_params.use_skip_range = get_parameter("particle_filter.use_skip_range").as_bool();
    config_params.skip_index_min = get_parameter("particle_filter.skip_index_min").as_int();
    config_params.skip_index_max = get_parameter("particle_filter.skip_index_max").as_int();
  }

  void SentryEstimatorNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if(!laser_msg_received_){
      laser_msg_received_ = true;
    }
    try{
    last_laser_msg_ = msg;
    particle_filter_.ObserveLaser(
        msg->ranges,
        msg->range_min,
        msg->range_max,
        msg->angle_min+config_params.laser_angle_offset,
        msg->angle_max+config_params.laser_angle_offset);

    PublishEstimatedPosition();
    PublishVisualization();
    }
    catch(std::exception e){
      RCLCPP_ERROR(this->get_logger(), "Laser: ", e.what());
    }
  }

  void SentryEstimatorNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    try{

      odom_loc_ = Eigen::Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
      odom_angle_ = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      particle_filter_.Predict(odom_loc_, odom_angle_);

      PublishEstimatedPosition();
      PublishVisualization();
      }
    catch(std::exception e){
      RCLCPP_ERROR(this->get_logger(), "Odom: ", e.what());
    }
  }

  void SentryEstimatorNode::PublishEstimatedPosition()
  {
    try{
      Eigen::Vector2f location;
      float angle;

      particle_filter_.GetLocation(&location, &angle);

      geometry_msgs::msg::Pose p;

      p.position.x = location.x();
      p.position.y = location.y();

      p.orientation.z = sin(angle * 0.5);
      p.orientation.w = cos(angle * 0.5);

      estimated_pose_pub_->publish(p);
    }
    catch(std::exception e){
      RCLCPP_ERROR(this->get_logger(), "Publish Estimated Position: ", e.what());
    }
  }

  void SentryEstimatorNode::InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    try{
    // Set new initial pose
    const Vector2f init_loc(msg->pose.pose.position.x, msg->pose.pose.position.y);
    const float init_angle = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_INFO(
        this->get_logger(),
        "Initialize: %s (%f,%f) %f\u00b0\n",
        config_params.map.c_str(),
        init_loc.x(),
        init_loc.y(),
        RadToDeg(init_angle));

    particle_filter_.Initialize(config_params.map, init_loc, init_angle);

    // PublishWorldTransform();
    PublishEstimatedPosition();
    PublishVisualization();
    PublishMapViz();
    }
    catch(std::exception e){
      RCLCPP_ERROR(this->get_logger(), "Initial Pose: ", e.what());
    }
  }

  void SentryEstimatorNode::DrawParticles(geometry_msgs::msg::PoseArray &cloud_msg)
  {
    vector<particle_filter::Particle> particles;
    particle_filter_.GetParticles(&particles);
    for (const particle_filter::Particle &p : particles)
    {
      auto pose_msg = geometry_msgs::msg::Pose{};
      pose_msg.position.x = p.loc.x();
      pose_msg.position.y = p.loc.y();
      pose_msg.orientation.w = cos(p.angle * 0.5);
      pose_msg.orientation.z = sin(p.angle * 0.5);
      cloud_msg.poses.push_back(pose_msg);
    }
  }

  void SentryEstimatorNode::DrawPredictedScan(visualization_msgs::msg::MarkerArray &viz_msg) {
    if(!laser_msg_received_){
      return;
    }
    Vector2f robot_loc(0, 0);
    float robot_angle(0);
    particle_filter_.GetLocation(&robot_loc, &robot_angle);
    vector<Vector2f> predicted_scan;

    particle_filter_.GetPredictedPointCloud(
        robot_loc,
        robot_angle,
        last_laser_msg_->ranges.size(),
        last_laser_msg_->range_min,
        last_laser_msg_->range_max,
        last_laser_msg_->angle_min + config_params.laser_angle_offset,
        last_laser_msg_->angle_max + config_params.laser_angle_offset,
        &predicted_scan);

    auto predicted_scan_msg = visualization_msgs::msg::Marker{};
    predicted_scan_msg.header.stamp = this->get_clock()->now();
    predicted_scan_msg.header.frame_id = "world";
    predicted_scan_msg.id = 1;
    predicted_scan_msg.type = 8; // Points
    predicted_scan_msg.color.b = 1.0;
    predicted_scan_msg.color.a = 1.0;
    predicted_scan_msg.scale.x = 0.04;
    predicted_scan_msg.scale.y = 0.04;

    for(std::size_t i = 0; i < predicted_scan.size(); i++){
      int laser_index = i * config_params.resize_factor;
      if(!config_params.use_skip_range || laser_index < config_params.skip_index_min || laser_index > config_params.skip_index_max){
        // Transform particle to map
        auto point_msg = geometry_msgs::msg::Point{};
        point_msg.x = predicted_scan[i].x();
        point_msg.y = predicted_scan[i].y();
        predicted_scan_msg.points.push_back(point_msg);
      }
    }
    viz_msg.markers.push_back(predicted_scan_msg);


    // Publish observed scan in world frame
    auto true_scan_msg = visualization_msgs::msg::Marker{};
    true_scan_msg.header.stamp = this->get_clock()->now();
    true_scan_msg.header.frame_id = "world";
    true_scan_msg.id = 2;
    true_scan_msg.type = 8; // Points
    true_scan_msg.color.r = 1.0;
    true_scan_msg.color.a = 1.0;
    true_scan_msg.scale.x = 0.02;
    true_scan_msg.scale.y = 0.02;

    auto rot_bl_to_world = Eigen::Rotation2D<float>(robot_angle).toRotationMatrix();
    for(std::size_t i = 0; i < last_laser_msg_->ranges.size(); i++){
      int laser_index = ((int) (i / config_params.resize_factor)) * config_params.resize_factor;
      if(!config_params.use_skip_range || laser_index < config_params.skip_index_min || laser_index > config_params.skip_index_max){
        // Transform particle to map
        float range = last_laser_msg_->ranges[i];
        if(range >= config_params.range_min && range <= config_params.range_max){
          float angle = last_laser_msg_->angle_min + i * last_laser_msg_->angle_increment + config_params.laser_angle_offset + robot_angle;

          Eigen::Vector2f p = Eigen::Vector2f(range*cos(angle), range*sin(angle)) + robot_loc + rot_bl_to_world*Eigen::Vector2f(config_params.laser_offset, 0.0);

          auto point_msg = geometry_msgs::msg::Point{};
          point_msg.x = p.x();
          point_msg.y = p.y();
          true_scan_msg.points.push_back(point_msg);
        }
      }
    }
    viz_msg.markers.push_back(true_scan_msg);
  }

  void SentryEstimatorNode::PublishMapViz()
  {
    auto map_msg = visualization_msgs::msg::Marker{};
    auto map = particle_filter_.GetMap();

    // Iterate through all lines in map
    for (size_t i = 0; i < map.lines.size(); ++i)
    {
      const geometry::Line2f line = map.lines[i];

      auto start_point = geometry_msgs::msg::Point{};
      start_point.x = line.p0.x();
      start_point.y = line.p0.y();

      auto end_point = geometry_msgs::msg::Point{};
      end_point.x = line.p1.x();
      end_point.y = line.p1.y();

      map_msg.points.push_back(start_point);
      map_msg.points.push_back(end_point);
    }

    map_msg.header.stamp = this->get_clock()->now();
    map_msg.header.frame_id = "world";
    map_msg.id = 0;
    map_msg.type = 5;   // Line List
    map_msg.action = 0; // Add / Modify
    map_msg.scale.x = 0.01;
    map_msg.color.r = 0.0;
    map_msg.color.g = 0.0;
    map_msg.color.b = 0.0;
    map_msg.color.a = 1.0;

    map_viz_pub_->publish(map_msg);
  }

  void SentryEstimatorNode::PublishVisualization()
  {
    static double t_last = 0;

    // if (GetMonotonicTime() - t_last < 1/30.0)
    // {
    //   // Rate-limit visualization.
    //   return;
    // }
    t_last = GetMonotonicTime();

    // Publish Particle Cloud
    auto cloud_msg = geometry_msgs::msg::PoseArray{};
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = this->get_clock()->now();
    DrawParticles(cloud_msg);
    cloud_viz_pub_->publish(cloud_msg);

    // Publish Debug Markers
    viz_msg_ = visualization_msgs::msg::MarkerArray{};
    DrawPredictedScan(viz_msg_);

    if (first_map_load_)
    {
      PublishMapViz();
      first_map_load_ = false;
    }

    debug_viz_pub_->publish(viz_msg_);
  }

  void SentryEstimatorNode::DrawWheelAxisVectors(std::vector<geometry::Line2f> & lines){
    int j = 0;
    for(auto & line : lines){
      auto marker_msg = visualization_msgs::msg::Marker{};
      
      marker_msg.header.frame_id = "base_link";
      marker_msg.header.stamp = this->get_clock()->now();
      marker_msg.id = j++; marker_msg.action = 0; marker_msg.type = 0;
      marker_msg.scale.x = 0.01; marker_msg.scale.y = 0.01; marker_msg.scale.z = 0.01; marker_msg.color.a = 1;

      geometry_msgs::msg::Point p0{}; p0.x = line.p0.x(); p0.y = line.p0.y(); p0.z = 0.0; marker_msg.points.push_back(p0);
      geometry_msgs::msg::Point p1{}; p1.x = line.p1.x(); p1.y = line.p1.y(); p1.z = 0.0; marker_msg.points.push_back(p1);

      viz_msg_.markers.push_back(marker_msg);
    }
  }

  void SentryEstimatorNode::DrawICRPoints(std::vector<Eigen::Vector3f> & points)
    {
    auto marker_msg_points = visualization_msgs::msg::Marker{};
    marker_msg_points.header.frame_id = "base_link";
    marker_msg_points.header.stamp = this->get_clock()->now();
    marker_msg_points.id = 4; marker_msg_points.action = 0; marker_msg_points.type = 7;
    marker_msg_points.scale.x = 0.01; marker_msg_points.scale.y = 0.01; marker_msg_points.scale.z = 0.01;
    marker_msg_points.color.b = 1; marker_msg_points.color.a = 1;

    for(auto & point : points){
      geometry_msgs::msg::Point point_msg{};
      point_msg.x = point.x();
      point_msg.y = point.y();
      point_msg.z = point.z();
      marker_msg_points.points.push_back(point_msg);
    }
    viz_msg_.markers.push_back(marker_msg_points);

    // Publish H-Space Sphere
    auto marker_msg_sphere = visualization_msgs::msg::Marker{};
    marker_msg_sphere.header.frame_id = "base_link";
    marker_msg_sphere.header.stamp = this->get_clock()->now();
    marker_msg_sphere.id = 5; marker_msg_sphere.action = 0; marker_msg_sphere.type = 2;
    marker_msg_sphere.scale.x = 1; marker_msg_sphere.scale.y = 1; marker_msg_sphere.scale.z = 1;
    marker_msg_sphere.color.r = 0.75; marker_msg_sphere.color.g = 0.75; marker_msg_sphere.color.b = 0.75;
    marker_msg_sphere.color.a = 0.25;
    viz_msg_.markers.push_back(marker_msg_sphere);
  }
} // namespace ghost_ros

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_ros::SentryEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}