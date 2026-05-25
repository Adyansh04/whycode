#include <rclcpp/rclcpp.hpp>

#include "whycode/ros/whycon_component.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<whycon::WhyconComponent>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
