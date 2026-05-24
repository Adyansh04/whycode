#include <rclcpp/rclcpp.hpp>

#include "whycode/ros/whycon_ros_interface.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("whycon");

    whycon::WhyconRosInterface whycon_ros(node.get());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
