#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;

class SentryControl : public rclcpp::Node {
  public:
    SentryControl() : Node("sentry_control") {

      robot_pose_sub_ = this->create_subscription<Pose>(
        "estimation/robot_pose", 
        10, 
        std::bind(&SentryControl::update_robot_pose_callback, this, _1));

      goal_pose_sub_ = this->create_subscription<Pose>(
        "goal_pose", 
        10, 
        std::bind(&SentryControl::update_goal_pose_callback, this, _1));

      command_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    }

    private:
      bool in_control = false;

      Pose robot_pose;
      Pose goal_pose;
      float speed = 1;

      rclcpp::Subscription<Pose>::SharedPtr robot_pose_sub_;
      rclcpp::Subscription<Pose>::SharedPtr goal_pose_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;

      void update_command_velocity()
      {
        if (in_control == true) {
           Vector3 desired_velocity;
          desired_velocity.x = goal_pose.position.x - robot_pose.position.x;
          desired_velocity.y = goal_pose.position.y - robot_pose.position.y;
          desired_velocity.z = goal_pose.position.z - robot_pose.position.z;

          float magnitude = sqrt(
            (desired_velocity.x * desired_velocity.x) +
            (desired_velocity.y * desired_velocity.y) +
            (desired_velocity.z * desired_velocity.z)
          );
          // normalize velocity
          desired_velocity.x = speed * desired_velocity.x / magnitude;
          desired_velocity.y = speed * desired_velocity.y / magnitude;
          desired_velocity.z = speed * desired_velocity.z / magnitude;

          Twist desired_twist;
          desired_twist.linear = desired_velocity;

          command_velocity_pub_->publish(desired_twist);
        }
      }

      void update_goal_pose_callback(const Pose::SharedPtr goal_pose_msg)
      {
        goal_pose = copy_pose(goal_pose_msg);
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
        return 2.0 * atan2(q.z, q.w);
      }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentryControl>());
    rclcpp::shutdown();
    return 0;
}