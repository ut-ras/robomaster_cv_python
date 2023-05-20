#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ghost_msgs/msg/ghost_robot_state.hpp"

class SentryControl : public rclcpp::Node {
    public:

        SentryControl();

    private:
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_sub_;
        rclcpp::Subscription<PoseStamped>::SharedPtr rviz_goal_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr automatic_control_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;

        geometry_msgs::msg::Pose robot_pose;
        geometry_msgs::msg::Pose goal_pose;

        void update_command_velocity();
        void update_goal_pose_callback(const geometry_msgs::msg::Pose::SharedPtr goal_pose_msg);
        void update_goal_pose_stamped_callback(const PoseStamped::SharedPtr rviz_goal_pose_msg);
        void update_robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr robot_pose_msg);
        void update_in_control(const std_msgs::msg::Bool::SharedPtr control);
};