#ifndef WHYCON_ROS_INTERFACE_HPP
#define WHYCON_ROS_INTERFACE_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <whycode_vision/msg/why_code_pose.hpp>
#include <whycode_vision/msg/why_code_pose_array.hpp>

#include "whycode/core/whycon_localization.hpp"
#include "whycode/image/debug_image_manager.hpp"
#include "whycode/image/image_handler.hpp"
#include "whycode/ros/recycled_image_strategy.hpp"
#include "whycode/tracking/ambiguity_resolver.hpp"
#include "whycode/tracking/id_stabilizer.hpp"
#include "whycode/tracking/marker_tracker.hpp"
#include "whycode/tracking/pose_stabilizer.hpp"

namespace whycon
{

class WhyconRosInterface
{
public:
    explicit WhyconRosInterface(rclcpp::Node* node);

private:
    // Helper methods for initialization and publishing
    void loadParameters();
    void setupROSTopics();
    void initializeWhyConModules();
    void publishResults(const std_msgs::msg::Header& header, DebugImageManager* debug);
    void publishSingleTF(
        const whycon::LocalizationSystem::MarkerPose& pose, const std_msgs::msg::Header& header);
    void createMarkerVisualization(
        const whycon::LocalizationSystem::MarkerPose& pose, int marker_index,
        const std_msgs::msg::Header& header, visualization_msgs::msg::Marker& marker,
        visualization_msgs::msg::Marker& label);

    // Processing
    void onRosImageReceived(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
    void processLatestFrame();
    void detectionControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response>      res);

    rclcpp::Node* node_ = nullptr;

    // Core WhyCon components
    whycon::DetectorParameters                  parameters_;
    std::unique_ptr<whycon::LocalizationSystem> system_             = nullptr;
    std::unique_ptr<whycon::MarkerTracker>      tracker_            = nullptr;
    std::unique_ptr<whycon::IDStabilizer>       id_stabilizer_      = nullptr;
    std::unique_ptr<whycon::PoseStabilizer>     pose_stabilizer_    = nullptr;
    std::unique_ptr<whycon::AmbiguityResolver>  ambiguity_resolver_ = nullptr;
    std::unique_ptr<ImageHandler>               image_handler_      = nullptr;
    std::unique_ptr<DebugImageManager>          debug_manager_      = nullptr;

    // ROS-specific members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            image_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                  detection_control_service_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               image_pub_;
    rclcpp::Publisher<whycode_vision::msg::WhyCodePoseArray>::SharedPtr whycode_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  visualization_markers_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                      tf_broadcaster_;

    // One topic per debug panel, and its reused outgoing message
    std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, DebugImageManager::PANEL_COUNT>
                                                                        debug_pubs_;
    std::array<sensor_msgs::msg::Image, DebugImageManager::PANEL_COUNT> debug_msgs_;

    // TF parameters
    std::string tf_frame_prefix_{};
    std::string parent_frame_id_{};

    static constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>;  // For image labels

    // Reusable buffers
    sensor_msgs::msg::Image               output_image_msg_;  // The annotated frame, drawn in place
    whycode_vision::msg::WhyCodePoseArray whycode_pose_array;
    whycon::LocalizationSystem::MarkerPose pose_buffer_;
    whycode_vision::msg::WhyCodePose       whycode_pose_msg_buffer_;
    visualization_msgs::msg::Marker        marker_buffer_;
    visualization_msgs::msg::Marker        marker_label_buffer_;
    visualization_msgs::msg::Marker        delete_all_marker_;  // Clears last frame's RViz markers

    std::array<char, 128>                        label_buffer_{};
    std::vector<whycon::MarkerDetector::Marker*> valid_detections_buffer_;
    std::vector<int>                             removed_ids_buffer_;
    visualization_msgs::msg::MarkerArray         marker_array_buffer_;

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    int     cam_height_ = 0, cam_width_ = 0;

    int targets_       = 0;
    int min_track_age_ = 0;

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

    // Latest frame state
    std_msgs::msg::Header latest_header_;
    rclcpp::Time          latest_ros_timestamp_;
};
}  // namespace whycon

#endif  // WHYCON_ROS_INTERFACE_HPP
