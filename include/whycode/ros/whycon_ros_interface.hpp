#ifndef WHYCON_ROS_INTERFACE_HPP
#define WHYCON_ROS_INTERFACE_HPP

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <whycode_vision/msg/why_code_pose.hpp>
#include <whycode_vision/msg/why_code_pose_array.hpp>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "whycode/core/whycon_localization.hpp"
#include "whycode/image/debug_image_manager.hpp"
#include "whycode/image/image_handler.hpp"
#include "whycode/tracking/id_stabilizer.hpp"
#include "whycode/tracking/marker_tracker.hpp"
#include "whycode/tracking/pose_stabilizer.hpp"

namespace whycon {

class WhyconRosInterface {
  public:
    explicit WhyconRosInterface(rclcpp::Node* node);

  private:
    // Helper methods for initialization and publishing
    void loadParameters();
    void setupROSTopics();
    void initializeWhyConModules();
    void publishResults(const std_msgs::msg::Header& header);
    void publishSingleTF(const whycon::LocalizationSystem::MarkerPose& pose, const int marker_index);
    void createMarkerVisualization(const whycon::LocalizationSystem::MarkerPose& pose, int marker_index,
                                   const std_msgs::msg::Header& header, visualization_msgs::msg::Marker& marker);

    // Processing
    void onRosImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
    void processTimerCallback();
    void processLatestFrame();
    void detectionControlCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response>      res);

    rclcpp::Node* node_ = nullptr;

    // Core WhyCon components
    whycon::DetectorParameters                  parameters_;
    std::unique_ptr<whycon::LocalizationSystem> system_          = nullptr;
    std::unique_ptr<whycon::MarkerTracker>      tracker_         = nullptr;
    std::unique_ptr<whycon::IDStabilizer>       id_stabilizer_   = nullptr;
    std::unique_ptr<whycon::PoseStabilizer>     pose_stabilizer_ = nullptr;
    std::unique_ptr<ImageHandler>               image_handler_   = nullptr;
    std::unique_ptr<DebugImageManager>          debug_manager_   = nullptr;

    // ROS-specific members
    image_transport::Subscriber                                     image_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr             detection_control_service_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          image_pub_, debug_images_pub_;
    rclcpp::Publisher<whycode_vision::msg::WhyCodePoseArray>::SharedPtr whycode_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_markers_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                   process_timer_;

    // TF parameters
    std::string tf_frame_prefix_{};
    std::string parent_frame_id_{};

    // Reusable buffers
    cv::Mat                                               output_image_buffer_;
    whycode_vision::msg::WhyCodePoseArray   whycode_pose_array;
    whycon::LocalizationSystem::MarkerPose                pose_buffer_;
    whycode_vision::msg::WhyCodePose        whycode_pose_msg_buffer_;
    visualization_msgs::msg::Marker                      marker_buffer_;

    std::ostringstream                           string_stream_buffer_;
    std::vector<whycon::MarkerDetector::Marker*> valid_detections_buffer_;
    std::vector<int>                             removed_ids_buffer_;
    visualization_msgs::msg::MarkerArray         marker_array_buffer_;

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    int     cam_height_ = 0, cam_width_ = 0;

    int  targets_           = 0;
    int  min_track_age_     = 0;
    bool last_marker_count_ = 0;

    bool camera_params_loaded_ = false;

    // State variables
    bool is_tracking_       = false;
    bool should_reset_      = true;
    bool detection_enabled_ = true;

    // Publishing control flags
    bool publish_poses_                 = false;
    bool publish_images_                = false;
    bool publish_debug_images_          = false;
    bool publish_tf_                    = false;
    bool publish_visualization_markers_ = false;

    // var
    bool new_frame_available_ = false;

    // Latest frame state
    std_msgs::msg::Header latest_header_;
    rclcpp::Time          latest_ros_timestamp_;

    // Proecessing rate
    double process_rate_hz_ = 30.0;
};
}  // namespace whycon

#endif  // WHYCON_ROS_INTERFACE_HPP
