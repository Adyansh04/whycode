#ifndef WHYCON_ROS_COMPONENT_HPP
#define WHYCON_ROS_COMPONENT_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "whycode/ros/whycon_ros_interface.hpp"

namespace whycon
{

class WhyconComponent : public rclcpp::Node
{
public:
    explicit WhyconComponent(const rclcpp::NodeOptions& options);

private:
    std::shared_ptr<WhyconRosInterface> whycon_ros_;
};

}  // namespace whycon

#endif  // WHYCON_ROS_COMPONENT_HPP
