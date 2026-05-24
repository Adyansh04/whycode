#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>
#include <thread>

namespace whycon {

class BagImagePublisherNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit() {
        ros::NodeHandle&                nh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        // Parameters
        std::string bag_path, image_topic;
        nh.param<std::string>("bag_path", bag_path, "/libraries/whycon_stuff/rs_cam.bag");
        nh.param<std::string>("image_topic", image_topic, "/camera/color/image_raw");

        pub_ = it.advertise("image_raw", 1);

        // Start publishing in a separate thread so as not to block nodelet manager
        pub_thread_ = std::thread(&BagImagePublisherNodelet::publishFromBag, this, bag_path, image_topic);
    }

    ~BagImagePublisherNodelet() {
        if (pub_thread_.joinable())
            pub_thread_.join();
    }

  private:
    image_transport::Publisher pub_;
    std::thread                pub_thread_;

    void publishFromBag(const std::string& bag_path, const std::string& image_topic) {
        rosbag::Bag bag;
        try {
            bag.open(bag_path, rosbag::bagmode::Read);
        } catch (rosbag::BagException& e) {
            NODELET_ERROR_STREAM("Failed to open bag file: " << bag_path << " (" << e.what() << ")");
            return;
        }

        std::vector<std::string> topics{ image_topic };
        rosbag::View             view(bag, rosbag::TopicQuery(topics));

        ros::Time prev_stamp;
        bool      first = true;

        for (const rosbag::MessageInstance& m : view) {
            if (!ros::ok())
                break;
            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if (img) {
                if (!first) {
                    ros::Duration dt = img->header.stamp - prev_stamp;
                    if (dt.toSec() > 0 && dt.toSec() < 5.0) {  // avoid huge jumps
                        dt.sleep();
                    }
                } else {
                    first = false;
                }
                pub_.publish(img);
                prev_stamp = img->header.stamp;
            }
        }
        bag.close();
        NODELET_WARN("Finished publishing images from bag.");
    }
};

}  // namespace whycon

PLUGINLIB_EXPORT_CLASS(whycon::BagImagePublisherNodelet, nodelet::Nodelet)