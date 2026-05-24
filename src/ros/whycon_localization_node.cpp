#include <ros/ros.h>

#include "whycode/ros/whycon_ros_interface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "whycon");
    ros::NodeHandle n("~");

    whycon::WhyconRosInterface whycon_ros(n);

    ros::spin();
}
