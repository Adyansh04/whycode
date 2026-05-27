#include "whycode/ros/whycon_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace whycon
{

WhyconComponent::WhyconComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("whycon", options)
{
    whycon_ros_ = std::make_shared<WhyconRosInterface>(this);
}

}  // namespace whycon

RCLCPP_COMPONENTS_REGISTER_NODE(whycon::WhyconComponent)
