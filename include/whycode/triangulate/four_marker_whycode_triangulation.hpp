#ifndef FOUR_MARKER_WHYCODE_TRIANGULATION_NODE_HPP
#define FOUR_MARKER_WHYCODE_TRIANGULATION_NODE_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <whycode_vision/msg/why_code_pose_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>

class FourMarkerWhyCodeTriangulationNode : public rclcpp::Node {
  public:
    struct MarkerData {
        geometry_msgs::msg::Pose              pose;
        int                                   whycode_id  = -1;
        int                                   tracking_id = -1;
        bool                                  id_valid    = false;
        bool                                  found       = false;
        std::chrono::steady_clock::time_point last_seen;

        // Reset Marker data
        void reset() {
            found    = false;
            id_valid = false;
        }

        // Convert ROS pose to Eigen Vector3d (position)
        Eigen::Vector3d getPosition() const {
            return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        }
    };

    struct TriangulationConfig {
        // Four marker IDs for rectangular arrangement
        int marker_id_top_left     = 3;
        int marker_id_bottom_left  = 1;
        int marker_id_top_right    = 4;
        int marker_id_bottom_right = 2;

        // Two known distances
        double known_distance_vertical   = 0.3;  // top-bottom distance
        double known_distance_horizontal = 0.5;  // left-right distance

        double      distance_tolerance             = 1e-6;
        double      marker_timeout                 = 1.0;  // seconds
        std::string camera_frame                   = "camera_link";
        std::string target_frame                   = "marker_center_final";
        bool        publish_tf                     = true;
        bool        publish_pose                   = true;
        bool        publish_intermediate_tf        = true;  // publish left/right center TFs
        bool        publish_camera_odom            = true;
        std::string odom_frame                     = "odom";
        std::string base_link_frame                = "base_link";
        bool        enable_ground_truth_comparison = true;

        bool        publish_camera_odom_tf   = true;
        bool        align_first_measurement  = true;
        std::string camera_odom_child_frame  = "camera_link";
        std::string camera_odom_parent_frame = "whycode_marker_center_final";
    };

    struct CameraPlaneAngles {
        double roll;   // Rotation around X-axis
        double pitch;  // Rotation around Y-axis
        double yaw;    // Rotation around Z-axis
    };

    struct PoseEstimationResult {
        Eigen::Vector3d                       position;
        Eigen::Matrix3d                       rotation;
        CameraPlaneAngles                     camera_plane_angles;
        bool                                  is_valid;
        std::string                           error_message;
        std::chrono::steady_clock::time_point timestamp;
    };

    struct CameraOdometry {
        double    x        = 0.0;
        double    y        = 0.0;
        double    theta    = 0.0;
        bool      is_valid = false;
        rclcpp::Time timestamp;
    };

    // Constructor
    explicit FourMarkerWhyCodeTriangulationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Destructor
    ~FourMarkerWhyCodeTriangulationNode() = default;

  private:
    bool            have_alignment_ = false;
    Eigen::Matrix3d R_align_        = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_align_        = Eigen::Vector3d::Zero();

    // ROS communication
    rclcpp::Subscription<whycode_vision::msg::WhyCodePoseArray>::SharedPtr poses_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr                        pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Configuration and data - 4 markers
    TriangulationConfig config_;
    MarkerData          marker_top_left_;
    MarkerData          marker_bottom_left_;
    MarkerData          marker_top_right_;
    MarkerData          marker_bottom_right_;

    // Ground truth comparison
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscriber_;
    nav_msgs::msg::Odometry                                  latest_ground_truth_;
    bool                                                     ground_truth_received_ = false;

    // Camera odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_odom_publisher_;
    CameraOdometry                                      latest_camera_odom_;

    // Timing
    rclcpp::TimerBase::SharedPtr          processing_timer_;
    std::chrono::steady_clock::time_point last_successful_estimation_;

    // Core functionality
    void loadParameters();
    void setupRosCommunication();
    void posesCallback(const whycode_vision::msg::WhyCodePoseArray::ConstSharedPtr& msg);
    void processingTimerCallback();

    void           groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    CameraOdometry calculateCameraOdometry(const PoseEstimationResult& result);
    void           publishCameraOdometry(const CameraOdometry& odom);
    void           compareWithGroundTruth(const CameraOdometry& calculated_odom);
    double         normalizeAngle(double angle) const;
    void           maybeInitAlignment(const Eigen::Matrix3d& R_mc, const Eigen::Vector3d& t_mc);

    // Hierarchical pose estimation methods
    bool                 performHierarchicalTriangulation();
    PoseEstimationResult estimateLeftCenterPose();
    PoseEstimationResult estimateRightCenterPose();
    PoseEstimationResult estimateFinalCenterPose(const PoseEstimationResult& left_result,
                                                 const PoseEstimationResult& right_result);

    // Core pose estimation (unchanged)
    PoseEstimationResult estimatePose(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const;

    // Utility functions
    bool areAllMarkersValid() const;
    bool validateInputs(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const;

    CameraPlaneAngles calculateCameraPlaneAngles(const Eigen::Vector3d& plane_normal) const;

    // Publishing functions
    void                       publishTransform(const PoseEstimationResult& result, const std::string& frame_name);
    void                       publishPose(const PoseEstimationResult& result);
    geometry_msgs::msg::PoseStamped eigenToRosPose(const Eigen::Vector3d& position,
                                                   const Eigen::Matrix3d& rotation) const;

    bool   gt_aligned_ = false;
    double gt_x0_ = 0.0, gt_y0_ = 0.0, gt_theta0_ = 0.0;

    // Constants
    static constexpr double MIN_DISTANCE       = 1e-8;
    static constexpr double MIN_KNOWN_DISTANCE = 1e-6;
    static constexpr double PROCESSING_RATE    = 30.0;  // Hz
};

#endif  // FOUR_MARKER_WHYCODE_TRIANGULATION_NODE_HPP
