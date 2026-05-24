#ifndef TWO_MARKER_APRILTAG_TRIANGULATION_NODE_HPP
#define TWO_MARKER_APRILTAG_TRIANGULATION_NODE_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>

#include "apriltag_ros/AprilTagDetectionArray.h"

class TwoMarkerApriltagTriangulationNode {
  public:
    struct MarkerData {
        geometry_msgs::Pose                   pose;
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
        // Two marker IDs for baseline
        int marker_id_1 = 0;
        int marker_id_2 = 1;

        // Single known distance
        double known_distance = 0.5;  // distance between markers

        double      distance_tolerance             = 0.5;
        double      marker_timeout                 = 5.0;  // seconds
        std::string camera_frame                   = "camera_link";
        std::string target_frame                   = "april_marker_center";
        bool        publish_tf                     = true;
        bool        publish_pose                   = true;
        bool        publish_camera_odom            = true;
        std::string odom_frame                     = "odom";
        std::string base_link_frame                = "base_link";
        bool        enable_ground_truth_comparison = false;

        bool        publish_camera_odom_tf   = true;
        bool        align_first_measurement  = false;
        std::string camera_odom_child_frame  = "camera_link";
        std::string camera_odom_parent_frame = "apriltag_marker_center";
        bool        align_ground_truth       = false;
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
        ros::Time timestamp;
    };

    // Constructor
    explicit TwoMarkerApriltagTriangulationNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    // Destructor
    ~TwoMarkerApriltagTriangulationNode() = default;

    // Main processing loop
    void spin();

  private:
    bool            have_alignment_ = false;
    Eigen::Matrix3d R_align_        = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_align_        = Eigen::Vector3d::Zero();

    // ROS handles
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;

    // ROS communication
    ros::Subscriber                                poses_subscriber_;
    ros::Publisher                                 pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Configuration and data - 2 markers
    TriangulationConfig config_;
    MarkerData          marker1_;
    MarkerData          marker2_;

    // Ground truth comparison
    ros::Subscriber    ground_truth_subscriber_;
    nav_msgs::Odometry latest_ground_truth_;
    bool               ground_truth_received_ = false;

    // Camera odometry
    ros::Publisher camera_odom_publisher_;
    CameraOdometry latest_camera_odom_;

    // Timing
    ros::Timer                            processing_timer_;
    std::chrono::steady_clock::time_point last_successful_estimation_;

    // Core functionality
    void loadParameters();
    void setupRosCommunication();
    void posesCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void processingTimerCallback(const ros::TimerEvent& event);

    void           groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
    CameraOdometry calculateCameraOdometry(const PoseEstimationResult& result);
    void           publishCameraOdometry(const CameraOdometry& odom);
    void           compareWithGroundTruth(const CameraOdometry& calculated_odom);
    double         normalizeAngle(double angle) const;
    void           maybeInitAlignment(const Eigen::Matrix3d& R_mc, const Eigen::Vector3d& t_mc);

    // Core pose estimation
    PoseEstimationResult estimatePose(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const;

    // Utility functions
    bool areTwoMarkersValid() const;
    bool validateInputs(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const;

    CameraPlaneAngles calculateCameraPlaneAngles(const Eigen::Vector3d& plane_normal) const;

    // Publishing functions
    void                       publishTransform(const PoseEstimationResult& result);
    void                       publishPose(const PoseEstimationResult& result);
    geometry_msgs::PoseStamped eigenToRosPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation) const;

    bool   gt_aligned_ = false;
    double gt_x0_ = 0.0, gt_y0_ = 0.0, gt_theta0_ = 0.0;

    // Constants
    static constexpr double MIN_DISTANCE       = 1e-8;
    static constexpr double MIN_KNOWN_DISTANCE = 1e-6;
    static constexpr double PROCESSING_RATE    = 30.0;  // Hz
};

#endif  // TWO_MARKER_APRILTAG_TRIANGULATION_NODE_HPP