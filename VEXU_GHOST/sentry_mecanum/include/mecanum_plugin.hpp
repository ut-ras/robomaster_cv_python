#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace mecanum_plugin {
    class MecanumPluginPrivate;

    class MecanumPlugin : public gazebo::ModelPlugin {
        public:
            MecanumPlugin();

            ~MecanumPlugin();
            
            void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

        private:
            std::unique_ptr<MecanumPluginPrivate> impl_;
            void command_callback(const geometry_msgs::msg::Twist::SharedPtr cmd);
    };
}