#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace sentry_mecanum {

    class Mecanum : public rclcpp::Node {
        public:

            gazebo::physics::JointPtr left_front_joint;
            gazebo::physics::JointPtr left_back_joint;
            gazebo::physics::JointPtr right_front_joint;
            gazebo::physics::JointPtr right_back_joint;

            double wheel_radius;
            double left_right_seperation;
            double front_back_seperation;

            Mecanum();

            void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_topic_sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

            void command_callback(const geometry_msgs::msg::Twist::SharedPtr cmd);
    };
}