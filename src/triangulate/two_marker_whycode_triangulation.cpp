#include "whycode/triangulate/two_marker_whycode_triangulation.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "whycon_whycode_localization/WhyCodePoseArray.h"

TwoMarkerWhyCodeTriangulationNode::TwoMarkerWhyCodeTriangulationNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
    loadParameters();
    setupRosCommunication();

    last_successful_estimation_ = std::chrono::steady_clock::now();

    ROS_INFO("WhyCode 2-Marker Triangulation Node initialized");
    ROS_INFO("Tracking markers: ID1=%d, ID2=%d", config_.marker_id_1, config_.marker_id_2);
    ROS_INFO("Known distance: %.3f m", config_.known_distance);
}

void TwoMarkerWhyCodeTriangulationNode::loadParameters() {
    private_nh_.param("marker_id_1", config_.marker_id_1, config_.marker_id_1);
    private_nh_.param("marker_id_2", config_.marker_id_2, config_.marker_id_2);
    private_nh_.param("known_distance", config_.known_distance, config_.known_distance);

    private_nh_.param("distance_tolerance", config_.distance_tolerance, config_.distance_tolerance);
    private_nh_.param("marker_timeout", config_.marker_timeout, config_.marker_timeout);
    private_nh_.param("camera_frame", config_.camera_frame, config_.camera_frame);
    private_nh_.param("target_frame", config_.target_frame, config_.target_frame);
    private_nh_.param("publish_tf", config_.publish_tf, config_.publish_tf);
    private_nh_.param("publish_pose", config_.publish_pose, config_.publish_pose);
    private_nh_.param("publish_camera_odom", config_.publish_camera_odom, config_.publish_camera_odom);
    private_nh_.param("odom_frame", config_.odom_frame, config_.odom_frame);
    private_nh_.param("base_link_frame", config_.base_link_frame, config_.base_link_frame);
    private_nh_.param("enable_ground_truth_comparison", config_.enable_ground_truth_comparison,
                      config_.enable_ground_truth_comparison);
    private_nh_.param("publish_camera_odom_tf", config_.publish_camera_odom_tf, config_.publish_camera_odom_tf);
    private_nh_.param("align_first_measurement", config_.align_first_measurement, config_.align_first_measurement);
    private_nh_.param("camera_odom_child_frame", config_.camera_odom_child_frame, config_.camera_odom_child_frame);
    private_nh_.param("camera_odom_parent_frame", config_.camera_odom_parent_frame, config_.camera_odom_parent_frame);
    private_nh_.param("align_ground_truth", config_.align_ground_truth, config_.align_ground_truth);

    // Validation
    if (config_.marker_id_1 == config_.marker_id_2) {
        ROS_ERROR("Marker IDs must be different! marker_id_1=%d, marker_id_2=%d", config_.marker_id_1,
                  config_.marker_id_2);
        ros::shutdown();
        return;
    }

    if (config_.known_distance <= MIN_KNOWN_DISTANCE) {
        ROS_ERROR("Known distance must be positive and > %.2e", MIN_KNOWN_DISTANCE);
        ros::shutdown();
    }
}

void TwoMarkerWhyCodeTriangulationNode::setupRosCommunication() {
    // Subscriber
    poses_subscriber_ = nh_.subscribe("whycon/poses", 1, &TwoMarkerWhyCodeTriangulationNode::posesCallback, this);

    // Publishers
    if (config_.publish_pose) {
        pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("two_marker_triangulation_whycode/pose", 1);
    }

    if (config_.publish_tf) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    }

    // Ground truth subscriber
    if (config_.enable_ground_truth_comparison) {
        ground_truth_subscriber_ =
                nh_.subscribe("/odom_ground_truth", 1, &TwoMarkerWhyCodeTriangulationNode::groundTruthCallback, this);
    }

    // Camera odometry publisher
    if (config_.publish_camera_odom) {
        camera_odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("two_marker_triangulation_whycode/camera_odom", 1);
    }

    // Processing timer
    processing_timer_ = nh_.createTimer(ros::Duration(1.0 / PROCESSING_RATE),
                                        &TwoMarkerWhyCodeTriangulationNode::processingTimerCallback, this);
}

void TwoMarkerWhyCodeTriangulationNode::posesCallback(
        const whycon_whycode_localization::WhyCodePoseArray::ConstPtr& msg) {
    // Reset found flags
    marker1_.reset();
    marker2_.reset();

    const auto current_time = std::chrono::steady_clock::now();

    // Process incoming markers
    for (const auto& marker : msg->poses) {
        if (marker.whycode_id == config_.marker_id_1) {
            marker1_.whycode_id  = marker.whycode_id;
            marker1_.tracking_id = marker.tracking_id;
            marker1_.id_valid    = marker.id_valid;
            marker1_.pose        = marker.pose;
            marker1_.found       = true;
            marker1_.last_seen   = current_time;
        } else if (marker.whycode_id == config_.marker_id_2) {
            marker2_.whycode_id  = marker.whycode_id;
            marker2_.tracking_id = marker.tracking_id;
            marker2_.id_valid    = marker.id_valid;
            marker2_.pose        = marker.pose;
            marker2_.found       = true;
            marker2_.last_seen   = current_time;
        }
    }
}

void TwoMarkerWhyCodeTriangulationNode::processingTimerCallback(const ros::TimerEvent& event) {
    if (!areTwoMarkersValid()) {
        return;
    }

    // Get marker positions
    const Eigen::Vector3d P1 = marker1_.getPosition();
    const Eigen::Vector3d P2 = marker2_.getPosition();

    // Estimate pose
    auto result = estimatePose(P1, P2, config_.known_distance);

    if (result.is_valid) {
        // Update timestamp
        last_successful_estimation_ = std::chrono::steady_clock::now();

        // Publish results
        if (config_.publish_tf) {
            publishTransform(result);
        }

        if (config_.publish_pose) {
            publishPose(result);
        }

        // Calculate and publish camera odometry
        if (config_.publish_camera_odom) {
            CameraOdometry camera_odom = calculateCameraOdometry(result);
            latest_camera_odom_        = camera_odom;
            publishCameraOdometry(camera_odom);

            if (config_.publish_camera_odom_tf) {
                geometry_msgs::TransformStamped ts;
                ts.header.stamp            = camera_odom.timestamp;
                ts.header.frame_id         = config_.camera_odom_parent_frame;
                ts.child_frame_id          = config_.camera_odom_child_frame;
                ts.transform.translation.x = camera_odom.x;
                ts.transform.translation.y = camera_odom.y;
                ts.transform.translation.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, camera_odom.theta);
                ts.transform.rotation = tf2::toMsg(q);
                tf_broadcaster_->sendTransform(ts);
            }

            // Compare with ground truth if available
            if (config_.enable_ground_truth_comparison && ground_truth_received_) {
                compareWithGroundTruth(camera_odom);
            }
        }

        // Debug output
        // ROS_INFO_THROTTLE(1.0, "2-Marker pose estimated successfully. Angle: %.2f degrees",
        //                   result.camera_plane_angles * 180.0 / M_PI);
    } else {
        ROS_WARN_THROTTLE(1.0, "2-Marker pose estimation failed: %s", result.error_message.c_str());
    }
}

TwoMarkerWhyCodeTriangulationNode::PoseEstimationResult
TwoMarkerWhyCodeTriangulationNode::estimatePose(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                                                double known_distance) const {
    PoseEstimationResult result;
    result.is_valid  = false;
    result.timestamp = std::chrono::steady_clock::now();

    // Input validation
    if (!validateInputs(P1, P2, known_distance)) {
        result.error_message = "Invalid inputs for pose estimation.";
        return result;
    }

    // Calculate vector between markers
    const Eigen::Vector3d marker_vector     = P2 - P1;
    const double          observed_distance = marker_vector.norm();

    // Check for degenerate case
    if (observed_distance < MIN_DISTANCE) {
        result.error_message = "Markers too close or coincident";
        return result;
    }

    // Calculate rough midpoint
    const Eigen::Vector3d midpoint_rough = 0.5 * (P1 + P2);

    // Construct coordinate frame
    // X-axis: direction from marker 1 to marker 2
    const Eigen::Vector3d x_axis = marker_vector.normalized();

    // Z-axis: normal to plane (Cross product of position vectors)
    Eigen::Vector3d z_axis             = P1.cross(P2);
    const double    cross_product_norm = z_axis.norm();

    // Check for collinear case
    if (cross_product_norm < MIN_DISTANCE) {
        result.error_message = "Markers and camera are collinear";
        return result;
    }

    z_axis.normalize();

    // Y-axis: right-handed system
    const Eigen::Vector3d y_axis = z_axis.cross(x_axis);

    // Distance correction (project to actual known distance)
    const Eigen::Vector3d corrected_P1     = midpoint_rough - 0.5 * known_distance * x_axis;
    const Eigen::Vector3d corrected_P2     = midpoint_rough + 0.5 * known_distance * x_axis;
    const Eigen::Vector3d corrected_center = 0.5 * (corrected_P1 + corrected_P2);

    // Build rotation matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;

    // Calculate camera-plane angle
    const CameraPlaneAngles camera_plane_angle = calculateCameraPlaneAngles(z_axis);

    // Populate result
    result.position            = corrected_center;
    result.rotation            = rotation_matrix;
    result.camera_plane_angles = camera_plane_angle;
    result.is_valid            = true;
    result.error_message       = "Pose estimation successful";

    return result;
}

bool TwoMarkerWhyCodeTriangulationNode::areTwoMarkersValid() const {
    const auto current_time     = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::duration<double>(config_.marker_timeout);

    // Check if both markers are found and have valid IDs
    if (!marker1_.found || !marker2_.found || !marker1_.id_valid || !marker2_.id_valid) {
        return false;
    }

    // Check for timeout on either marker
    if (current_time - marker1_.last_seen > timeout_duration || current_time - marker2_.last_seen > timeout_duration) {
        return false;
    }

    return true;
}

void TwoMarkerWhyCodeTriangulationNode::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_ground_truth_   = *msg;
    ground_truth_received_ = true;
}

TwoMarkerWhyCodeTriangulationNode::CameraOdometry
TwoMarkerWhyCodeTriangulationNode::calculateCameraOdometry(const PoseEstimationResult& result) {
    CameraOdometry odom;
    odom.timestamp = ros::Time::now();

    // T_camera_marker = [R_cm, t_cm]
    const Eigen::Matrix3d R_cm = result.rotation;
    const Eigen::Vector3d t_cm = result.position;

    // Invert to get T_marker_camera = [R_mc, t_mc]
    const Eigen::Matrix3d R_mc = R_cm.transpose();
    const Eigen::Vector3d t_mc = -R_mc * t_cm;

    // Optional alignment so first odom = (0,0,0)
    Eigen::Matrix3d R_use = R_mc;
    Eigen::Vector3d t_use = t_mc;
    if (config_.align_first_measurement) {
        maybeInitAlignment(R_mc, t_mc);
        R_use = R_align_ * R_mc;
        t_use = R_align_ * t_mc + t_align_;
    }

    // 2D odom in the marker plane: x,y from t_use; yaw from R_use about z-axis
    odom.x = t_use.x();
    odom.y = t_use.y();

    // Robust yaw extraction (rotation about z): atan2(R(1,0), R(0,0))
    const double yaw = std::atan2(R_use(1, 0), R_use(0, 0));
    odom.theta       = normalizeAngle(yaw);

    odom.is_valid = true;
    return odom;
}

void TwoMarkerWhyCodeTriangulationNode::maybeInitAlignment(const Eigen::Matrix3d& R_mc, const Eigen::Vector3d& t_mc) {
    if (have_alignment_)
        return;

    // We want (R_align * R_mc, R_align * t_mc + t_align) = (I, 0)
    // Choose R_align = R_mc^T, t_align = -R_align * t_mc
    R_align_        = R_mc.transpose();
    t_align_        = -R_align_ * t_mc;
    have_alignment_ = true;

    ROS_INFO("Initialized 2-marker camera odom alignment: start at x=0,y=0,theta=0");
}

void TwoMarkerWhyCodeTriangulationNode::publishCameraOdometry(const CameraOdometry& odom) {
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp    = odom.timestamp;
    odom_msg.header.frame_id = config_.camera_odom_parent_frame;
    odom_msg.child_frame_id  = config_.camera_odom_child_frame;

    // Position
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.position.z = 0.0;  // 2D odometry

    // Orientation (only yaw)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom.theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Covariance (can be tuned based on your accuracy requirements)
    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0]  = 0.01;  // x variance
    odom_msg.pose.covariance[7]  = 0.01;  // y variance
    odom_msg.pose.covariance[35] = 0.05;  // yaw variance

    camera_odom_publisher_.publish(odom_msg);
}

void TwoMarkerWhyCodeTriangulationNode::compareWithGroundTruth(const CameraOdometry& calculated_odom) {
    if (!ground_truth_received_) {
        return;
    }

    // Extract ground truth 2D pose
    double gt_x     = latest_ground_truth_.pose.pose.position.x;
    double gt_y     = latest_ground_truth_.pose.pose.position.y;
    double gt_theta = tf2::getYaw(latest_ground_truth_.pose.pose.orientation);

    // Optional ground truth alignment
    if (config_.align_ground_truth) {
        if (!gt_aligned_) {
            gt_x0_      = gt_x;
            gt_y0_      = gt_y;
            gt_theta0_  = gt_theta;
            gt_aligned_ = true;
            ROS_INFO("Initialized 2-marker ground truth alignment to start at 0,0,0");
        }
        gt_x -= gt_x0_;
        gt_y -= gt_y0_;
        gt_theta = normalizeAngle(gt_theta - gt_theta0_);
    }

    // Calculate errors
    double error_x     = calculated_odom.x - gt_x;
    double error_y     = calculated_odom.y - gt_y;
    double error_theta = normalizeAngle(calculated_odom.theta - gt_theta);

    // Calculate distance error
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

    // Log comparison (throttled to avoid spam)
    ROS_INFO_THROTTLE(0.5, "2-Marker Odometry Comparison:");
    ROS_INFO_THROTTLE(0.5, "  Calculated: x=%.3f, y=%.3f, theta=%.3f°", calculated_odom.x, calculated_odom.y,
                      calculated_odom.theta * 180.0 / M_PI);
    ROS_INFO_THROTTLE(0.5, "  Ground Truth: x=%.3f, y=%.3f, theta=%.3f°", gt_x, gt_y, gt_theta * 180.0 / M_PI);
    ROS_INFO_THROTTLE(0.5, "  Errors: dx=%.3f, dy=%.3f, dtheta=%.3f°, distance=%.3f", error_x, error_y,
                      error_theta * 180.0 / M_PI, distance_error);

    // Log warnings for large errors (red), success for small errors (green)
    if (distance_error > 0.1) {  // 10cm threshold
        ROS_ERROR_STREAM("\033[1;31m[2-Marker] Large position error detected: " << distance_error << " m\033[0m");
    } else {
        ROS_INFO_STREAM("\033[1;32m[2-Marker] Position error within threshold: " << distance_error << " m\033[0m");
    }

    if (std::abs(error_theta) > 0.175) {  // ~10 degree threshold
        ROS_ERROR_STREAM("\033[1;31m[2-Marker] Large orientation error detected: "
                         << std::abs(error_theta) * 180.0 / M_PI << " degrees\033[0m");
    } else {
        ROS_INFO_STREAM("\033[1;32m[2-Marker] Orientation error within threshold: "
                        << std::abs(error_theta) * 180.0 / M_PI << " degrees\033[0m");
    }
}

double TwoMarkerWhyCodeTriangulationNode::normalizeAngle(double angle) const {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

bool TwoMarkerWhyCodeTriangulationNode::validateInputs(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                                                       double known_distance) const {
    // Check for NaN or inf
    if (!P1.allFinite() || !P2.allFinite()) {
        ROS_ERROR("[2-Marker] Invalid input: P1 or P2 contains NaN or inf.");
        return false;
    }

    // Check known distance
    if (known_distance <= MIN_KNOWN_DISTANCE) {
        ROS_ERROR("[2-Marker] Invalid input: known_distance must be positive and > %.2e", MIN_KNOWN_DISTANCE);
        return false;
    }

    // Check if markers are too close
    const double observed_distance = (P2 - P1).norm();
    if (observed_distance < MIN_DISTANCE) {
        ROS_ERROR("[2-Marker] Invalid input: Markers are too close to each other: %.3f m", observed_distance);
        return false;
    }
    return true;
}

TwoMarkerWhyCodeTriangulationNode::CameraPlaneAngles
TwoMarkerWhyCodeTriangulationNode::calculateCameraPlaneAngles(const Eigen::Vector3d& plane_normal) const {
    CameraPlaneAngles angles;

    // Normalize the plane normal vector
    Eigen::Vector3d n = plane_normal.normalized();

    // Camera optical axis - [0,0,1] in camera frame (pointing forward)
    const Eigen::Vector3d camera_z_axis(0.0, 0.0, 1.0);
    const Eigen::Vector3d camera_x_axis(1.0, 0.0, 0.0);
    const Eigen::Vector3d camera_y_axis(0.0, 1.0, 0.0);

    // Calculate roll (rotation around X-axis)
    // Project normal onto YZ plane and measure angle from Z-axis
    Eigen::Vector2d yz_projection(n.y(), n.z());
    if (yz_projection.norm() > 1e-6) {
        angles.roll = std::atan2(yz_projection.x(), yz_projection.y());  // atan2(ny, nz)
    } else {
        angles.roll = 0.0;
    }

    // Calculate pitch (rotation around Y-axis)
    // Project normal onto XZ plane and measure angle from Z-axis
    Eigen::Vector2d xz_projection(n.x(), n.z());
    if (xz_projection.norm() > 1e-6) {
        angles.pitch = -std::atan2(xz_projection.x(), xz_projection.y());  // -atan2(nx, nz)
    } else {
        angles.pitch = 0.0;
    }

    // Calculate yaw (rotation around Z-axis)
    // Project normal onto XY plane and measure angle from X-axis
    Eigen::Vector2d xy_projection(n.x(), n.y());
    if (xy_projection.norm() > 1e-6) {
        angles.yaw = std::atan2(xy_projection.y(), xy_projection.x());  // atan2(ny, nx)
    } else {
        angles.yaw = 0.0;
    }

    return angles;
}

void TwoMarkerWhyCodeTriangulationNode::publishTransform(const PoseEstimationResult& result) {
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp    = ros::Time::now();
    transform_stamped.header.frame_id = config_.camera_frame;  // camera_link
    transform_stamped.child_frame_id  = config_.target_frame;  // marker_center

    // Position
    transform_stamped.transform.translation.x = result.position.x();
    transform_stamped.transform.translation.y = result.position.y();
    transform_stamped.transform.translation.z = result.position.z();

    // Rotation (convert from rotation matrix to quaternion)
    const Eigen::Quaterniond quat(result.rotation);
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();

    tf_broadcaster_->sendTransform(transform_stamped);
}

void TwoMarkerWhyCodeTriangulationNode::publishPose(const PoseEstimationResult& result) {
    geometry_msgs::PoseStamped pose_msg = eigenToRosPose(result.position, result.rotation);
    pose_publisher_.publish(pose_msg);
}

geometry_msgs::PoseStamped TwoMarkerWhyCodeTriangulationNode::eigenToRosPose(const Eigen::Vector3d& position,
                                                                             const Eigen::Matrix3d& rotation) const {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp    = ros::Time::now();
    pose.header.frame_id = config_.camera_frame;
    pose.header.seq++;

    // Position
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();

    // Orientation (convert rotation matrix to quaternion)
    const Eigen::Quaterniond quat(rotation);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    return pose;
}

void TwoMarkerWhyCodeTriangulationNode::spin() {
    ros::spin();
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "two_marker_triangulation_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try {
        TwoMarkerWhyCodeTriangulationNode node(nh, private_nh);
        node.spin();
    } catch (const std::exception& e) {
        ROS_ERROR("[2-Marker] Exception in triangulation node: %s", e.what());
        return 1;
    }

    return 0;
}
