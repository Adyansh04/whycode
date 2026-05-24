#ifndef WHYCON_ROS_INTERFACE_HPP
#define WHYCON_ROS_INTERFACE_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <whycon_whycode_localization/WhyCodePose.h>
#include <whycon_whycode_localization/WhyCodePoseArray.h>

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
    explicit WhyconRosInterface(ros::NodeHandle& n);

    bool detectionControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  private:
    // Helper methods for initialization and publishing
    void loadParameters();
    void setupROSTopics();
    void initializeWhyConModules();
    void publishResults(const std_msgs::Header& header);
    void publishSingleTF(const whycon::LocalizationSystem::MarkerPose& pose, const int marker_index);
    void publishVisualizationMarkers(const std_msgs::Header& header);
    void createMarkerVisualization(const whycon::LocalizationSystem::MarkerPose& pose, int marker_index,
                                   const std_msgs::Header& header, visualization_msgs::Marker& marker);

    // Processing
    void onRosImageReceived(const sensor_msgs::ImageConstPtr& image_msg);
    void onIceoryxImageReceived();
    void processTimerCallback(const ros::TimerEvent& event);
    void processLatestFrame();

    // Core WhyCon components
    whycon::DetectorParameters                  parameters_;
    std::unique_ptr<whycon::LocalizationSystem> system_          = nullptr;
    std::unique_ptr<whycon::MarkerTracker>      tracker_         = nullptr;
    std::unique_ptr<whycon::IDStabilizer>       id_stabilizer_   = nullptr;
    std::unique_ptr<whycon::PoseStabilizer>     pose_stabilizer_ = nullptr;
    std::unique_ptr<ImageHandler>               image_handler_   = nullptr;
    std::unique_ptr<DebugImageManager>          debug_manager_   = nullptr;

    // ROS-specific members
    image_transport::ImageTransport           it_;
    image_transport::Subscriber               image_sub_;
    ros::ServiceServer                        detection_control_service_;
    ros::Publisher                            image_pub_, poses_pub_, debug_images_pub_;
    ros::Publisher                            whycode_pose_pub_;
    ros::Publisher                            visualization_markers_pub_;
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    ros::Timer                                process_timer_;

    // TF parameters
    std::string tf_frame_prefix_{};
    std::string parent_frame_id_{};

    // Reusable buffers
    cv::Mat                                       output_image_buffer_;
    whycon_whycode_localization::WhyCodePoseArray whycode_pose_array;
    whycon::LocalizationSystem::MarkerPose        pose_buffer_;
    whycon_whycode_localization::WhyCodePose      whycode_pose_msg_buffer_;
    visualization_msgs::Marker                    marker_buffer_;

    std::ostringstream                           string_stream_buffer_;
    std::vector<whycon::MarkerDetector::Marker*> valid_detections_buffer_;
    std::vector<int>                             removed_ids_buffer_;
    visualization_msgs::MarkerArray              marker_array_buffer_;

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

    // Input Source management
    enum class InputSource { ROS, ICEORYX };
    InputSource input_source_ = InputSource::ROS;

    // var
    bool new_frame_available_ = false;

    // Latest frame state
    std_msgs::Header latest_header_;
    ros::Time        latest_ros_timestamp_;

    // Proecessing rate
    double process_rate_hz_ = 30.0;
};
}  // namespace whycon

#endif  // WHYCON_ROS_INTERFACE_HPP
