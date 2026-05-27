#include <rclcpp/rclcpp.hpp>

#include "whycode/triangulate/two_marker_whycode_triangulation.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwoMarkerWhyCodeTriangulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
