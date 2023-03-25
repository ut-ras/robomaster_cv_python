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
#include "ghost_ros/ros_nodes/ghost_estimator_node.hpp"

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

  GhostEstimatorNode::GhostEstimatorNode() : Node("ghost_estimator_node")
  {
    // Loads configuration from ROS Parameters
    LoadROSParams();

    // Use simulated time in ROS
    rclcpp::Parameter use_sim_time_param("use_sim_time", false);
    this->set_parameter(use_sim_time_param);

    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GhostEstimatorNode::LaserCallback, this, _1));

    encoder_sub_ = this->create_subscription<ghost_msgs::msg::V5SensorUpdate>(
        "/v5/sensor_update",
        10,
        std::bind(&GhostEstimatorNode::EncoderCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initial_pose",
        10,
        std::bind(&GhostEstimatorNode::InitialPoseCallback, this, _1));

    // Publishers
    cloud_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);
    map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("map_viz", 10);
    debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("estimation_debug", 10);
    world_tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Init debug msg
    viz_msg_ = visualization_msgs::msg::MarkerArray{};

    particle_filter_ = ParticleFilter(config_params);
    last_laser_msg_ = sensor_msgs::msg::LaserScan{};
    first_map_load_ = true;

    const Vector2f init_loc(config_params.init_x, config_params.init_y);
    const float init_angle = config_params.init_r;

    odom_loc_(config_params.init_x, config_params.init_y);
    odom_angle_ = config_params.init_r;
    
    particle_filter_.Initialize(config_params.map, init_loc, config_params.init_r);
  }

  void GhostEstimatorNode::LoadROSParams()
  {
    // Odometry
    declare_parameter("odometry.left_mod_x", 0.0);
    declare_parameter("odometry.left_mod_y", 0.0);
    declare_parameter("odometry.right_mod_x", 0.0);
    declare_parameter("odometry.right_mod_y", 0.0);
    declare_parameter("odometry.back_mod_x", 0.0);
    declare_parameter("odometry.back_mod_y", 0.0);

    left_wheel_link_ = Eigen::Vector2f(
      get_parameter("odometry.left_mod_x").as_double(), 
      get_parameter("odometry.left_mod_y").as_double()
    );

    right_wheel_link_ = Eigen::Vector2f(
      get_parameter("odometry.right_mod_x").as_double(),
      get_parameter("odometry.right_mod_y").as_double()
    );
    
    back_wheel_link_ = Eigen::Vector2f(
      get_parameter("odometry.back_mod_x").as_double(),
      get_parameter("odometry.back_mod_y").as_double()
    );

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
    config_params.k1 = get_parameter("particle_filter.k1").as_double();
    config_params.k2 = get_parameter("particle_filter.k2").as_double();
    config_params.k3 = get_parameter("particle_filter.k3").as_double();
    config_params.k4 = get_parameter("particle_filter.k4").as_double();
    config_params.k5 = get_parameter("particle_filter.k5").as_double();
    config_params.k6 = get_parameter("particle_filter.k6").as_double();

    declare_parameter("particle_filter.laser_offset", 0.0);
    declare_parameter("particle_filter.min_update_dist", 0.0);
    declare_parameter("particle_filter.min_update_angle", 0.0);
    config_params.laser_offset = get_parameter("particle_filter.laser_offset").as_double();
    config_params.min_update_dist = get_parameter("particle_filter.min_update_dist").as_double();
    config_params.min_update_angle = get_parameter("particle_filter.min_update_angle").as_double();

    declare_parameter("particle_filter.sigma_observation", 0.0);
    declare_parameter("particle_filter.gamma", 0.0);
    declare_parameter("particle_filter.dist_short", 0.0);
    declare_parameter("particle_filter.dist_long", 0.0);
    declare_parameter("particle_filter.range_min", 0.0);
    declare_parameter("particle_filter.range_max", 0.0);
    declare_parameter("particle_filter.resize_factor", 0.0);
    config_params.sigma_observation = get_parameter("particle_filter.sigma_observation").as_double();
    config_params.gamma = get_parameter("particle_filter.gamma").as_double();
    config_params.dist_short = get_parameter("particle_filter.dist_short").as_double();
    config_params.dist_long = get_parameter("particle_filter.dist_long").as_double();
    config_params.range_min = get_parameter("particle_filter.range_min").as_double();
    config_params.range_max = get_parameter("particle_filter.range_max").as_double();
    config_params.resize_factor = get_parameter("particle_filter.resize_factor").as_double();
  }

  void GhostEstimatorNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Process laser observation

    // Update robot pose estimate

    // Update unmapped obstacle scans

    PublishWorldTransform();
    PublishVisualization();
  }

  void GhostEstimatorNode::EncoderCallback(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg)
  {
    // Calculate ICR Estimate from encoder angles
    CalculateHSpaceICR(msg);

    // With ICR Estimate, accumulate encoder ticks off drivetrain to estimate robot motion
    CalculateOdometry(msg);

    particle_filter_.Predict(odom_loc_, odom_angle_);

    // Publish newest robot state
    PublishGhostRobotState(msg);

    // Publish Visualization
    PublishJointStateMsg(msg);
    PublishWorldTransform();
    PublishVisualization();
  }

  void GhostEstimatorNode::CalculateOdometry(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
    Eigen::Matrix2f diff_swerve_jacobian;
    Eigen::Matrix2f diff_swerve_jacobian_inverse;
    diff_swerve_jacobian << 12.0 / 18.0 / 2.0, -12.0 / 18.0 / 2.0,
                            12.0 / 45.0 / 2.0, 12.0 / 45.0 / 2.0;
                            
    diff_swerve_jacobian_inverse << 18.0 / 12.0, 45.0 / 12.0,
                                    -18.0 / 12.0, 45.0 / 12.0;

    Eigen::Vector2f(
      msg->encoders[ghost_v5_config::DRIVE_LEFT_FRONT_MOTOR].velocity_rpm,
      msg->encoders[ghost_v5_config::DRIVE_LEFT_BACK_MOTOR].velocity_rpm);
  }

  void GhostEstimatorNode::PublishGhostRobotState(const ghost_msgs::msg::V5SensorUpdate::SharedPtr sensor_update_msg){
    // Initialize robot state msg
    auto robot_state_msg = ghost_msgs::msg::GhostRobotState{};
    robot_state_msg.header.stamp = sensor_update_msg->header.stamp;
    robot_state_msg.header.frame_id = "base_link";

    ///// Drivetrain States /////
    // Robot Location
    Vector2f robot_loc(0, 0);
    float robot_angle(0);
    particle_filter_.GetLocation(&robot_loc, &robot_angle);

    robot_state_msg.x = robot_loc.x();
    robot_state_msg.y = robot_loc.y();
    robot_state_msg.theta = robot_angle;

    // Velocity estimate (from odometry)

    robot_state_msg.x_vel = 0.0;
    robot_state_msg.y_vel = 0.0;
    robot_state_msg.theta_vel = 0.0;

    // Currently unimplemented (Should come from IMU / EKF)
    robot_state_msg.x_accel = 0.0;
    robot_state_msg.y_accel = 0.0;
    robot_state_msg.theta_accel = 0.0;

    ///// Steering States /////
    robot_state_msg.left_steering_angle = sensor_update_msg->encoders[ghost_v5_config::STEERING_LEFT_ENCODER].angle_degrees;
    robot_state_msg.right_steering_angle = sensor_update_msg->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER].angle_degrees;
    robot_state_msg.back_steering_angle = sensor_update_msg->encoders[ghost_v5_config::STEERING_BACK_ENCODER].angle_degrees;

    robot_state_msg.left_steering_vel = sensor_update_msg->encoders[ghost_v5_config::STEERING_LEFT_ENCODER].velocity_rpm;
    robot_state_msg.right_steering_vel = sensor_update_msg->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER].velocity_rpm;
    robot_state_msg.back_steering_vel = sensor_update_msg->encoders[ghost_v5_config::STEERING_BACK_ENCODER].velocity_rpm;

    ///// Shooter States /////
    robot_state_msg.left_shooter_vel = sensor_update_msg->encoders[ghost_v5_config::SHOOTER_LEFT_MOTOR].velocity_rpm;
    robot_state_msg.right_shooter_vel = sensor_update_msg->encoders[ghost_v5_config::SHOOTER_RIGHT_MOTOR].velocity_rpm;

    ///// Turret States /////
    robot_state_msg.turret_angle = sensor_update_msg->encoders[ghost_v5_config::TURRET_MOTOR].angle_degrees;
    robot_state_msg.turret_vel = sensor_update_msg->encoders[ghost_v5_config::TURRET_MOTOR].velocity_rpm;
  }

  void GhostEstimatorNode::CalculateHSpaceICR(ghost_msgs::msg::V5SensorUpdate::SharedPtr encoder_msg){
    // Calculate Odometry
    auto left_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_LEFT_ENCODER];
    auto right_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER];
    auto back_encoder = encoder_msg->encoders[ghost_v5_config::STEERING_BACK_ENCODER];

    // Calculate ICR
    std::vector<Eigen::Vector3f> h_space_icr_points{0};

    // Calculate Wheel Axis Unit Direction Vectors
    Eigen::Vector2f left_encoder_dir(
        sin(left_encoder.angle_degrees * M_PI / 180.0),
        -cos(left_encoder.angle_degrees * M_PI / 180.0));
    Eigen::Vector2f right_encoder_dir(
        sin(right_encoder.angle_degrees * M_PI / 180.0),
        -cos(right_encoder.angle_degrees * M_PI / 180.0));
    Eigen::Vector2f back_encoder_dir(
        sin(back_encoder.angle_degrees * M_PI / 180.0),
        -cos(back_encoder.angle_degrees * M_PI / 180.0));

    // Calculate Wheel Axis Vectors
    geometry::Line2f left_encoder_vector(left_wheel_link_, left_encoder_dir + left_wheel_link_);
    geometry::Line2f right_encoder_vector(right_wheel_link_, right_encoder_dir + right_wheel_link_);
    geometry::Line2f back_encoder_vector(back_wheel_link_, back_encoder_dir + back_wheel_link_);

    // Iterate through each pair of lines and calculate ICR
    auto line_pairs = std::vector<std::pair<geometry::Line2f, geometry::Line2f>>{
        std::pair<geometry::Line2f, geometry::Line2f>(left_encoder_vector, right_encoder_vector),
        std::pair<geometry::Line2f, geometry::Line2f>(back_encoder_vector, left_encoder_vector),
        std::pair<geometry::Line2f, geometry::Line2f>(back_encoder_vector, right_encoder_vector)
        };

    for (auto &pair : line_pairs)
    {
      auto l1 = pair.first;
      auto l2 = pair.second;
      if (fabs(geometry::Cross(l1.Dir(), l2.Dir())) < 1e-5)
      {
        h_space_icr_points.push_back(Eigen::Vector3f(l1.Dir().x(), l1.Dir().y(), 0.0));
      }
      else
      {
        Eigen::Hyperplane<float, 2> hp1 = Eigen::Hyperplane<float, 2>::Through(l1.p0, l1.p1);
        Eigen::Hyperplane<float, 2> hp2 = Eigen::Hyperplane<float, 2>::Through(l2.p0, l2.p1);

        auto intersection = hp1.intersection(hp2);
        auto intersection_3d = Eigen::Vector3f(intersection.x(), intersection.y(), 1);
        h_space_icr_points.push_back(intersection_3d / intersection_3d.norm());
      }
    }

    // Calculate distance from first point and subsequent points and their antipoles
    // Select closer of the two (point / antipole) for calculating average
    if((h_space_icr_points[0] - h_space_icr_points[1]).norm() > (h_space_icr_points[0] + h_space_icr_points[1]).norm()){
      h_space_icr_points[1] *= -1;
    }

    if((h_space_icr_points[0] - h_space_icr_points[2]).norm() > (h_space_icr_points[0] + h_space_icr_points[2]).norm()){
      h_space_icr_points[2] *= -1;
    }

    // Average ICR points in H-Space as our estimated center of rotation
    h_space_icr_avg_ = (h_space_icr_points[0] + h_space_icr_points[1] + h_space_icr_points[2]) / 3;

    // Handle parallel case
    if(fabs(h_space_icr_avg_[2]) < 1e-9){
      icr_flat_estimation_ = Eigen::Vector3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0);
    }
    else{
      icr_flat_estimation_ = Eigen::Vector3f(h_space_icr_avg_[0]/h_space_icr_avg_[2], h_space_icr_avg_[1]/h_space_icr_avg_[2], 0);
    }

    // Initialize Visualization msg, and add debug visualization
    std::vector<geometry::Line2f> lines{left_encoder_vector, right_encoder_vector, back_encoder_vector};
    DrawWheelAxisVectors(lines);

    std::vector<Eigen::Vector3f> points{
      h_space_icr_points[0],
      h_space_icr_points[1],
      h_space_icr_points[2],
      h_space_icr_avg_,
      icr_flat_estimation_
    };
    DrawICRPoints(points);
  }

  void GhostEstimatorNode::InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
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

    PublishWorldTransform();
    PublishVisualization();
    PublishMapViz();
  }

  void GhostEstimatorNode::DrawParticles(geometry_msgs::msg::PoseArray &viz_msg)
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
      viz_msg.poses.push_back(pose_msg);
    }
  }

  void GhostEstimatorNode::PublishWorldTransform()
  {
    auto tf_msg = tf2_msgs::msg::TFMessage{};

    auto world_to_base_tf = geometry_msgs::msg::TransformStamped{};
    world_to_base_tf.header.stamp = this->get_clock()->now();
    world_to_base_tf.header.frame_id = "world";
    world_to_base_tf.child_frame_id = "base_link";

    Vector2f robot_loc(0, 0);
    float robot_angle(0);
    particle_filter_.GetLocation(&robot_loc, &robot_angle);

    world_to_base_tf.transform.translation.x = robot_loc.x();
    world_to_base_tf.transform.translation.y = robot_loc.y();
    world_to_base_tf.transform.translation.z = 0.0;
    world_to_base_tf.transform.rotation.x = 0.0;
    world_to_base_tf.transform.rotation.y = 0.0;
    world_to_base_tf.transform.rotation.z = sin(robot_angle * 0.5);
    world_to_base_tf.transform.rotation.w = cos(robot_angle * 0.5);

    tf_msg.transforms.push_back(world_to_base_tf);
    world_tf_pub_->publish(tf_msg);
  }

  void GhostEstimatorNode::PublishMapViz()
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

  void GhostEstimatorNode::PublishVisualization()
  {
    viz_msg_ = visualization_msgs::msg::MarkerArray{};
    static double t_last = 0;
    if (GetMonotonicTime() - t_last < 0.05)
    {
      // Rate-limit visualization.
      return;
    }
    t_last = GetMonotonicTime();

    // Publish Particle Cloud
    auto viz_msg = geometry_msgs::msg::PoseArray{};
    viz_msg.header.frame_id = "world";
    viz_msg.header.stamp = this->get_clock()->now();
    DrawParticles(viz_msg);
    cloud_viz_pub_->publish(viz_msg);

    if (first_map_load_)
    {
      PublishMapViz();
      first_map_load_ = false;
    }

    debug_viz_pub_->publish(viz_msg_);
  }

  void GhostEstimatorNode::PublishJointStateMsg(const ghost_msgs::msg::V5SensorUpdate::SharedPtr msg){
    // Calculate Odometry
    auto left_encoder = msg->encoders[ghost_v5_config::STEERING_LEFT_ENCODER];
    auto right_encoder = msg->encoders[ghost_v5_config::STEERING_RIGHT_ENCODER];
    auto back_encoder = msg->encoders[ghost_v5_config::STEERING_BACK_ENCODER];

    // Publish joint states
    auto joint_state_msg = sensor_msgs::msg::JointState{};
    joint_state_msg.header.stamp = get_clock()->now();

    joint_state_msg.name.push_back("steering_left");  joint_state_msg.name.push_back("driveshaft_left");
    joint_state_msg.name.push_back("steering_right"); joint_state_msg.name.push_back("driveshaft_right"); 
    joint_state_msg.name.push_back("steering_back");  joint_state_msg.name.push_back("driveshaft_back");

    joint_state_msg.position.push_back(left_encoder.angle_degrees * M_PI / 180.0); joint_state_msg.position.push_back(0.0);
    joint_state_msg.position.push_back(right_encoder.angle_degrees * M_PI / 180.0); joint_state_msg.position.push_back(0.0);
    joint_state_msg.position.push_back(back_encoder.angle_degrees * M_PI / 180.0); joint_state_msg.position.push_back(0.0);

    joint_state_pub_->publish(joint_state_msg);
  }

  void GhostEstimatorNode::DrawWheelAxisVectors(std::vector<geometry::Line2f> & lines){
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

  void GhostEstimatorNode::DrawICRPoints(std::vector<Eigen::Vector3f> & points)
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
  rclcpp::spin(std::make_shared<ghost_ros::GhostEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}