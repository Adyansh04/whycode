#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "whycode/ros/whycon_ros_interface.hpp"

namespace whycon {

class WhyconNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit() {
        ros::NodeHandle nh(getPrivateNodeHandle());
        whycon_ros_ = std::make_shared<WhyconRosInterface>(nh);
    }

  private:
    std::shared_ptr<WhyconRosInterface> whycon_ros_;
};

}  // namespace whycon

PLUGINLIB_EXPORT_CLASS(whycon::WhyconNodelet, nodelet::Nodelet)