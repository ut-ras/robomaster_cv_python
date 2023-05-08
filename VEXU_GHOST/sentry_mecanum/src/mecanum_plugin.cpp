// recieves command on cmd_vel, and then should calculate wheel moves, move wheels,
// calculate some error, and then publish the move to mecanum_vel which is connected to a planar move (and publish to odom)
// (error code not written)
// current issue is that plugin load is called and logging messages in there works, but the subscription callback doesn't work

#include "mecanum_plugin.hpp"

namespace mecanum_plugin
{
  class MecanumPluginPrivate {
    public:

      // Gazebo Ptrs
      gazebo::physics::ModelPtr model_;
      gazebo::physics::JointPtr left_back_joint;
      gazebo::physics::JointPtr left_front_joint;
      gazebo::physics::JointPtr right_back_joint;
      gazebo::physics::JointPtr right_front_joint;
      gazebo::physics::LinkPtr base_link;

      /// Node for ROS communication.
      gazebo_ros::Node::SharedPtr ros_node_;

      /// Subscribers
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_topic_sub_;

      /// Publishers
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr planar_control_topic_;

      double x_stddev;
      double y_stddev;
      double r_stddev;

      double wheel_radius;
      double left_right_seperation;
      double front_back_seperation;

      std::normal_distribution<double> x_noise_dist;
      std::normal_distribution<double> y_noise_dist;
      std::normal_distribution<double> r_noise_dist;

      // Mersenne twister PRNG, initialized with seed from previous random device instance
      std::mt19937 gen; 
  };

  MecanumPlugin::MecanumPlugin(): impl_(std::make_unique<MecanumPluginPrivate>()){}

  MecanumPlugin::~MecanumPlugin(){}

  void MecanumPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {

    // Get ROS Node and Gazebo Model Ptr
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    impl_->model_= model;
    
    impl_->left_back_joint = model->GetJoint(sdf->GetElement("LeftBack")->Get<std::string>());
    impl_->left_front_joint = model->GetJoint(sdf->GetElement("LeftFront")->Get<std::string>());
    impl_->right_front_joint = model->GetJoint(sdf->GetElement("RightFront")->Get<std::string>());
    impl_->right_back_joint = model->GetJoint(sdf->GetElement("RightBack")->Get<std::string>());
    impl_->base_link = model->GetLink(sdf->GetElement("BaseLink")->Get<std::string>());

    impl_->wheel_radius = sdf->Get<double>("WheelRadius", 0.0).first;
    impl_->left_right_seperation = sdf->Get<double>("LeftRightSeperation", 0.0).first;
    impl_->front_back_seperation = sdf->Get<double>("FrontBackSeperation", 0.0).first;

    impl_->odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    impl_->command_topic_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 
      10, 
      std::bind(&MecanumPlugin::command_callback, this, std::placeholders::_1));

    impl_->planar_control_topic_  = impl_->ros_node_->create_publisher<geometry_msgs::msg::Twist>("mecanum_vel", 10);

  };

  void MecanumPlugin::command_callback(const geometry_msgs::msg::Twist::SharedPtr cmd)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Command Velocity Received");

    ignition::math::Pose3d world_pose = impl_->model_->WorldPose();

    float theta = acos(world_pose.Pos().X()/sqrt(pow(world_pose.Pos().X(),2)+pow(world_pose.Pos().Y(),2))); //robot rotation
    float vel_forward = cmd->linear.x*cos(theta)-cmd->linear.y*sin(theta);
    float vel_sideways = cmd->linear.x*sin(theta)+cmd->linear.y*cos(theta);
    float angular_vel = cmd->angular.z;

    float inverse_radius = 1/impl_->wheel_radius;
    float seperation = impl_->front_back_seperation + impl_->left_right_seperation;

    float left_front_av = inverse_radius*(vel_forward+vel_sideways-seperation*angular_vel); //1
    float right_front_av = inverse_radius*(vel_forward-vel_sideways+seperation*angular_vel); //2
    float left_back_av = inverse_radius*(vel_forward-vel_sideways-seperation*angular_vel); //3
    float right_back_av = inverse_radius*(vel_forward+vel_sideways+seperation*angular_vel); //4

    // add error

    impl_->left_back_joint->SetVelocity(0, left_back_av);
    impl_->left_front_joint->SetVelocity(0, left_front_av);
    impl_->right_back_joint->SetVelocity(0, right_back_av);
    impl_->right_front_joint->SetVelocity(0, right_front_av);

    geometry_msgs::msg::Twist planar_move;
    planar_move.linear.x = cmd->linear.x;
    planar_move.linear.y = cmd->linear.y;
    planar_move.angular.z = cmd->angular.z;
    impl_->planar_control_topic_->publish(planar_move);
    // ignition::math::Vector3d angular_velocity(0,cmd->angular.z,0);
    // impl_->base_link->SetAngularVel(angular_velocity);

    auto odom_msg = nav_msgs::msg::Odometry{};
    odom_msg.header.stamp = impl_->ros_node_->get_clock()->now();
    odom_msg.pose.pose.position.x = world_pose.Pos().X() + impl_->x_noise_dist(impl_->gen);
    odom_msg.pose.pose.position.y = world_pose.Pos().Y() + impl_->y_noise_dist(impl_->gen);
    double angle = 2.0 * atan2(world_pose.Rot().Z(), world_pose.Rot().W()) + impl_->r_noise_dist(impl_->gen);
    odom_msg.pose.pose.orientation.z = sin(angle * 0.5);
    odom_msg.pose.pose.orientation.w = cos(angle * 0.5);
    impl_->odom_pub_->publish(odom_msg);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MecanumPlugin)
}