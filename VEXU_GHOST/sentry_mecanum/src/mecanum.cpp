#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace sentry_mecanum
{

  class Mecanum : public rclcpp::Node {
    public:
    
      void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        this->left_front_joint = _model->GetJoint(_sdf->GetElement("LeftRear")->Get<std::string>());
        gazebo::physics::JointPtr left_back_joint = _model->GetJoint(_sdf->GetElement("LeftBack")->Get<std::string>());
        gazebo::physics::JointPtr right_front_joint = _model->GetJoint(_sdf->GetElement("RightFront")->Get<std::string>());
        gazebo::physics::JointPtr right_back_joint = _model->GetJoint(_sdf->GetElement("RightBack")->Get<std::string>());

        double wheel_radius = _sdf->Get<double>("wheelRadius", 0.0).first;
        double left_right_seperation = _sdf->Get<double>("LeftRightSeperation", 0.0).first;
        double front_back_seperation = _sdf->Get<double>("FrontBackSeperation", 0.0).first;

        std::string cmd_topic = _sdf->Get<std::string>("commandTopic", "cmd_vel").first;
        std::string odom_topic = _sdf->Get<std::string>("odometryTopic", "odom").first;

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

        command_topic_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          cmd_topic, 
          10, 
          std::bind(&Mecanum::command_callback, this, std::placeholders::_1));
      }


      Mecanum() : Node("mecanum") {}

      private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_topic_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        void command_callback(const geometry_msgs::msg::Twist::SharedPtr cmd)
        {
          double geometry_factor = (this->left_right_seperation + this->front_back_seperation)/2;

          float x = cmd->linear.x;
          float y = cmd->linear.y;
          float rotation = cmd->angular.z;

          
        }
  };
} 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sentry_mecanum::Mecanum>());
    rclcpp::shutdown();
    return 0;
}