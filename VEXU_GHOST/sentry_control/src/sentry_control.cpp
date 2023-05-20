#include <memory>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Bool;

class SentryControl : public rclcpp::Node {
  public:
    SentryControl() : Node("sentry_control") {

      robot_pose_sub_ = this->create_subscription<Pose>(
        "estimated_robot_pose", 
        10, 
        std::bind(&SentryControl::update_robot_pose_callback, this, _1));

      goal_pose_sub_ = this->create_subscription<Pose>(
        "goal_pose", 
        10, 
        std::bind(&SentryControl::update_goal_pose_callback, this, _1));

      rviz_goal_pose_sub_ = this->create_subscription<PoseStamped>(
        "goal_pose_stamped", 
        10, 
        std::bind(&SentryControl::update_goal_pose_stamped_callback, this, _1));

      automatic_control_sub_ = this->create_subscription<Bool>(
        "automatic_control", 
        10, 
        std::bind(&SentryControl::update_in_control, this, _1));

      command_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    }

    private:
      bool in_control = false;

      Pose robot_pose;
      Pose goal_pose;

      rclcpp::Subscription<Pose>::SharedPtr robot_pose_sub_;
      rclcpp::Subscription<Pose>::SharedPtr goal_pose_sub_;
      rclcpp::Subscription<PoseStamped>::SharedPtr rviz_goal_pose_sub_;
      rclcpp::Subscription<Bool>::SharedPtr automatic_control_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;

      void update_in_control(const Bool::SharedPtr control)
      {
        in_control = control->data;
      }

      void update_command_velocity()
      {
        if (in_control == true) {
          Vector3 desired_velocity;
          desired_velocity.x = goal_pose.position.x - robot_pose.position.x;
          desired_velocity.y = goal_pose.position.y - robot_pose.position.y;

          float magnitude = sqrt(
            (desired_velocity.x * desired_velocity.x) +
            (desired_velocity.y * desired_velocity.y)
          );
          // square root magnitude and divide by 2
          desired_velocity.x = desired_velocity.x / sqrt(magnitude*4);
          desired_velocity.y = desired_velocity.y / sqrt(magnitude*4);

          // rotate to robot frame
          float rotation = -quaternion_to_rotation(robot_pose.orientation);
          float temp_x = desired_velocity.x;
          float temp_y = desired_velocity.y;
          desired_velocity.x = temp_x*cos(rotation) - temp_y*sin(rotation);
          desired_velocity.y = temp_x*sin(rotation) + temp_y*cos(rotation);

          Twist desired_twist;
          desired_twist.linear = desired_velocity;

          command_velocity_pub_->publish(desired_twist);
        }
      }

      void update_goal_pose_callback(const Pose::SharedPtr goal_pose_msg)
      {
        goal_pose = copy_pose(goal_pose_msg);
        goal_pose_callback();
      }

      void update_goal_pose_stamped_callback(const PoseStamped::SharedPtr rviz_goal_pose_msg)
      {
        goal_pose = rviz_goal_pose_msg->pose;
        goal_pose_callback();
      }

      void goal_pose_callback()
      {
        in_control = true;
        update_command_velocity();
      }

      void update_robot_pose_callback(const Pose::SharedPtr robot_pose_msg)
      {
        robot_pose = copy_pose(robot_pose_msg);
        update_command_velocity();
      }

      Pose copy_pose(Pose::SharedPtr pose) {
        Pose p;

        p.position.x = pose->position.x;
        p.position.y = pose->position.y;

        p.orientation.z = pose->orientation.z;
        p.orientation.w = pose->orientation.w;
        
        return p;
      }

      Quaternion rotation_to_quaternion(double r) {
        Quaternion q;
        q.x = 0;
        q.y = 0;
        q.z = sin(r * 0.5);
        q.w = cos(r * 0.5);
        return q;
      }

      double quaternion_to_rotation(Quaternion q) {
        tf2::Quaternion tf2q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf2q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
        // return 2.0 * atan2(q.z, q.w);
      }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentryControl>());
    rclcpp::shutdown();
    return 0;
}