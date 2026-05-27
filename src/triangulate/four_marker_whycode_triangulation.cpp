#include "whycode/triangulate/four_marker_whycode_triangulation.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>

#include "whycode_vision/msg/why_code_pose_array.hpp"

FourMarkerWhyCodeTriangulationNode::FourMarkerWhyCodeTriangulationNode(
    const rclcpp::NodeOptions& options)
  : rclcpp::Node("four_marker_whycode_triangulation_node", options)
{
    loadParameters();
    setupRosCommunication();

    last_successful_estimation_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "WhyCode 4-Marker Triangulation Node initialized");
    RCLCPP_INFO(
        get_logger(),
        "Tracking markers: TL=%d, BL=%d, TR=%d, BR=%d",
        config_.marker_id_top_left,
        config_.marker_id_bottom_left,
        config_.marker_id_top_right,
        config_.marker_id_bottom_right);
    RCLCPP_INFO(
        get_logger(),
        "Known distances: Vertical=%.3f m, Horizontal=%.3f m",
        config_.known_distance_vertical,
        config_.known_distance_horizontal);
}

void FourMarkerWhyCodeTriangulationNode::loadParameters()
{
    declare_parameter<int>("marker_id_top_left", config_.marker_id_top_left);
    declare_parameter<int>("marker_id_bottom_left", config_.marker_id_bottom_left);
    declare_parameter<int>("marker_id_top_right", config_.marker_id_top_right);
    declare_parameter<int>("marker_id_bottom_right", config_.marker_id_bottom_right);
    declare_parameter<double>("known_distance_vertical", config_.known_distance_vertical);
    declare_parameter<double>("known_distance_horizontal", config_.known_distance_horizontal);
    declare_parameter<double>("distance_tolerance", config_.distance_tolerance);
    declare_parameter<double>("marker_timeout", config_.marker_timeout);
    declare_parameter<std::string>("camera_frame", config_.camera_frame);
    declare_parameter<std::string>("target_frame", config_.target_frame);
    declare_parameter<bool>("publish_tf", config_.publish_tf);
    declare_parameter<bool>("publish_pose", config_.publish_pose);
    declare_parameter<bool>("publish_intermediate_tf", config_.publish_intermediate_tf);
    declare_parameter<bool>("publish_camera_odom", config_.publish_camera_odom);
    declare_parameter<std::string>("odom_frame", config_.odom_frame);
    declare_parameter<std::string>("base_link_frame", config_.base_link_frame);
    declare_parameter<bool>(
        "enable_ground_truth_comparison",
        config_.enable_ground_truth_comparison);
    declare_parameter<bool>("publish_camera_odom_tf", config_.publish_camera_odom_tf);
    declare_parameter<bool>("align_first_measurement", config_.align_first_measurement);
    declare_parameter<std::string>("camera_odom_child_frame", config_.camera_odom_child_frame);
    declare_parameter<std::string>("camera_odom_parent_frame", config_.camera_odom_parent_frame);

    get_parameter("marker_id_top_left", config_.marker_id_top_left);
    get_parameter("marker_id_bottom_left", config_.marker_id_bottom_left);
    get_parameter("marker_id_top_right", config_.marker_id_top_right);
    get_parameter("marker_id_bottom_right", config_.marker_id_bottom_right);
    get_parameter("known_distance_vertical", config_.known_distance_vertical);
    get_parameter("known_distance_horizontal", config_.known_distance_horizontal);
    get_parameter("distance_tolerance", config_.distance_tolerance);
    get_parameter("marker_timeout", config_.marker_timeout);
    get_parameter("camera_frame", config_.camera_frame);
    get_parameter("target_frame", config_.target_frame);
    get_parameter("publish_tf", config_.publish_tf);
    get_parameter("publish_pose", config_.publish_pose);
    get_parameter("publish_intermediate_tf", config_.publish_intermediate_tf);
    get_parameter("publish_camera_odom", config_.publish_camera_odom);
    get_parameter("odom_frame", config_.odom_frame);
    get_parameter("base_link_frame", config_.base_link_frame);
    get_parameter("enable_ground_truth_comparison", config_.enable_ground_truth_comparison);
    get_parameter("publish_camera_odom_tf", config_.publish_camera_odom_tf);
    get_parameter("align_first_measurement", config_.align_first_measurement);
    get_parameter("camera_odom_child_frame", config_.camera_odom_child_frame);
    get_parameter("camera_odom_parent_frame", config_.camera_odom_parent_frame);

    // Validation
    std::vector<int> marker_ids = { config_.marker_id_top_left,
                                    config_.marker_id_bottom_left,
                                    config_.marker_id_top_right,
                                    config_.marker_id_bottom_right };
    std::sort(marker_ids.begin(), marker_ids.end());
    for (size_t i = 1; i < marker_ids.size(); ++i)
    {
        if (marker_ids[i] == marker_ids[i - 1])
        {
            RCLCPP_ERROR(
                get_logger(),
                "Duplicate marker IDs detected! All 4 markers must have unique IDs.");
            rclcpp::shutdown();
            return;
        }
    }

    if (config_.known_distance_vertical <= MIN_KNOWN_DISTANCE ||
        config_.known_distance_horizontal <= MIN_KNOWN_DISTANCE)
    {
        RCLCPP_ERROR(get_logger(), "Known distances must be positive and > %.2e", MIN_KNOWN_DISTANCE);
        rclcpp::shutdown();
    }
}

void FourMarkerWhyCodeTriangulationNode::setupRosCommunication()
{
    // Subscriber
    poses_subscriber_ = create_subscription<whycode_vision::msg::WhyCodePoseArray>(
        "whycon/poses",
        rclcpp::QoS(10),
        std::bind(&FourMarkerWhyCodeTriangulationNode::posesCallback, this, std::placeholders::_1));

    // Publishers
    if (config_.publish_pose)
    {
        pose_publisher_ =
            create_publisher<geometry_msgs::msg::PoseStamped>("triangulation/pose", rclcpp::QoS(10));
    }

    if (config_.publish_tf || config_.publish_intermediate_tf)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Ground truth subscriber
    if (config_.enable_ground_truth_comparison)
    {
        ground_truth_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ground_truth",
            rclcpp::QoS(10),
            std::bind(
                &FourMarkerWhyCodeTriangulationNode::groundTruthCallback,
                this,
                std::placeholders::_1));
    }

    // Camera odometry publisher
    if (config_.publish_camera_odom)
    {
        camera_odom_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("triangulation/camera_odom", rclcpp::QoS(10));
    }

    // Processing timer
    const auto period = std::chrono::duration<double>(1.0 / PROCESSING_RATE);
    processing_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FourMarkerWhyCodeTriangulationNode::processingTimerCallback, this));
}

void FourMarkerWhyCodeTriangulationNode::posesCallback(
    const whycode_vision::msg::WhyCodePoseArray::ConstSharedPtr& msg)
{
    // Reset all found flags
    marker_top_left_.reset();
    marker_bottom_left_.reset();
    marker_top_right_.reset();
    marker_bottom_right_.reset();

    const auto current_time = std::chrono::steady_clock::now();

    // Process incoming markers
    for (const auto& marker : msg->poses)
    {
        if (marker.whycode_id == config_.marker_id_top_left)
        {
            marker_top_left_.whycode_id  = marker.whycode_id;
            marker_top_left_.tracking_id = marker.tracking_id;
            marker_top_left_.id_valid    = marker.id_valid;
            marker_top_left_.pose        = marker.pose;
            marker_top_left_.found       = true;
            marker_top_left_.last_seen   = current_time;
        }
        else if (marker.whycode_id == config_.marker_id_bottom_left)
        {
            marker_bottom_left_.whycode_id  = marker.whycode_id;
            marker_bottom_left_.tracking_id = marker.tracking_id;
            marker_bottom_left_.id_valid    = marker.id_valid;
            marker_bottom_left_.pose        = marker.pose;
            marker_bottom_left_.found       = true;
            marker_bottom_left_.last_seen   = current_time;
        }
        else if (marker.whycode_id == config_.marker_id_top_right)
        {
            marker_top_right_.whycode_id  = marker.whycode_id;
            marker_top_right_.tracking_id = marker.tracking_id;
            marker_top_right_.id_valid    = marker.id_valid;
            marker_top_right_.pose        = marker.pose;
            marker_top_right_.found       = true;
            marker_top_right_.last_seen   = current_time;
        }
        else if (marker.whycode_id == config_.marker_id_bottom_right)
        {
            marker_bottom_right_.whycode_id  = marker.whycode_id;
            marker_bottom_right_.tracking_id = marker.tracking_id;
            marker_bottom_right_.id_valid    = marker.id_valid;
            marker_bottom_right_.pose        = marker.pose;
            marker_bottom_right_.found       = true;
            marker_bottom_right_.last_seen   = current_time;
        }
    }
}

void FourMarkerWhyCodeTriangulationNode::processingTimerCallback()
{
    if (!areAllMarkersValid())
    {
        return;
    }

    // Perform hierarchical triangulation
    if (performHierarchicalTriangulation())
    {
        last_successful_estimation_ = std::chrono::steady_clock::now();
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Hierarchical pose estimation successful");
    }
    else
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hierarchical pose estimation failed");
    }
}

bool FourMarkerWhyCodeTriangulationNode::performHierarchicalTriangulation()
{
    // Step 1: Estimate left center pose (top_left + bottom_left)
    auto left_result = estimateLeftCenterPose();
    if (!left_result.is_valid)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Left center estimation failed: %s",
            left_result.error_message.c_str());
        return false;
    }

    // Step 2: Estimate right center pose (top_right + bottom_right)
    auto right_result = estimateRightCenterPose();
    if (!right_result.is_valid)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Right center estimation failed: %s",
            right_result.error_message.c_str());
        return false;
    }

    // Step 3: Estimate final center pose (left_center + right_center)
    auto final_result = estimateFinalCenterPose(left_result, right_result);
    if (!final_result.is_valid)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Final center estimation failed: %s",
            final_result.error_message.c_str());
        return false;
    }

    // Publish intermediate TFs
    if (config_.publish_intermediate_tf && tf_broadcaster_)
    {
        publishTransform(left_result, "marker_center_left");
        publishTransform(right_result, "marker_center_right");
    }

    // Publish final results
    if (config_.publish_tf && tf_broadcaster_)
    {
        publishTransform(final_result, config_.target_frame);
    }

    if (config_.publish_pose)
    {
        publishPose(final_result);
    }

    // Calculate and publish camera odometry using final result
    if (config_.publish_camera_odom)
    {
        CameraOdometry camera_odom = calculateCameraOdometry(final_result);
        latest_camera_odom_        = camera_odom;
        publishCameraOdometry(camera_odom);

        if (config_.publish_camera_odom_tf)
        {
            geometry_msgs::msg::TransformStamped ts;
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
        if (config_.enable_ground_truth_comparison && ground_truth_received_)
        {
            compareWithGroundTruth(camera_odom);
        }
    }

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    //                      "Final pose estimated successfully. Angle: %.2f degrees",
    //                      final_result.camera_plane_angle * 180.0 / M_PI);

    return true;
}

FourMarkerWhyCodeTriangulationNode::PoseEstimationResult
FourMarkerWhyCodeTriangulationNode::estimateLeftCenterPose()
{
    const Eigen::Vector3d P_top_left    = marker_top_left_.getPosition();
    const Eigen::Vector3d P_bottom_left = marker_bottom_left_.getPosition();

    return estimatePose(P_top_left, P_bottom_left, config_.known_distance_vertical);
}

FourMarkerWhyCodeTriangulationNode::PoseEstimationResult
FourMarkerWhyCodeTriangulationNode::estimateRightCenterPose()
{
    const Eigen::Vector3d P_top_right    = marker_top_right_.getPosition();
    const Eigen::Vector3d P_bottom_right = marker_bottom_right_.getPosition();

    return estimatePose(P_top_right, P_bottom_right, config_.known_distance_vertical);
}

FourMarkerWhyCodeTriangulationNode::PoseEstimationResult
FourMarkerWhyCodeTriangulationNode::estimateFinalCenterPose(
    const PoseEstimationResult& left_result, const PoseEstimationResult& right_result)
{
    // Use the center positions from left and right results
    const Eigen::Vector3d P_left_center  = left_result.position;
    const Eigen::Vector3d P_right_center = right_result.position;

    return estimatePose(P_left_center, P_right_center, config_.known_distance_horizontal);
}

FourMarkerWhyCodeTriangulationNode::PoseEstimationResult
FourMarkerWhyCodeTriangulationNode::estimatePose(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const
{
    PoseEstimationResult result;
    result.is_valid  = false;
    result.timestamp = std::chrono::steady_clock::now();

    // Input validation
    if (!validateInputs(P1, P2, known_distance))
    {
        result.error_message = "Invalid inputs for pose estimation.";
        return result;
    }

    // Calculate vector between markers
    const Eigen::Vector3d marker_vector     = P2 - P1;
    const double          observed_distance = marker_vector.norm();

    // Check for degenerate case
    if (observed_distance < MIN_DISTANCE)
    {
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
    if (cross_product_norm < MIN_DISTANCE)
    {
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

    // Calculate camera-plane angles
    const CameraPlaneAngles camera_plane_angles = calculateCameraPlaneAngles(z_axis);

    // Populate result
    result.position            = corrected_center;
    result.rotation            = rotation_matrix;
    result.camera_plane_angles = camera_plane_angles;
    result.is_valid            = true;
    result.error_message       = "Pose estimation successful";

    return result;
}

bool FourMarkerWhyCodeTriangulationNode::areAllMarkersValid() const
{
    const auto current_time     = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::duration<double>(config_.marker_timeout);

    // Check if all 4 markers are found and have valid IDs
    if (!marker_top_left_.found || !marker_bottom_left_.found || !marker_top_right_.found ||
        !marker_bottom_right_.found || !marker_top_left_.id_valid ||
        !marker_bottom_left_.id_valid || !marker_top_right_.id_valid ||
        !marker_bottom_right_.id_valid)
    {
        return false;
    }

    // Check for timeout on any marker
    if (current_time - marker_top_left_.last_seen > timeout_duration ||
        current_time - marker_bottom_left_.last_seen > timeout_duration ||
        current_time - marker_top_right_.last_seen > timeout_duration ||
        current_time - marker_bottom_right_.last_seen > timeout_duration)
    {
        return false;
    }

    return true;
}

void FourMarkerWhyCodeTriangulationNode::groundTruthCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    latest_ground_truth_   = *msg;
    ground_truth_received_ = true;
}

FourMarkerWhyCodeTriangulationNode::CameraOdometry
FourMarkerWhyCodeTriangulationNode::calculateCameraOdometry(const PoseEstimationResult& result)
{
    CameraOdometry odom;
    odom.timestamp = now();

    // T_camera_marker = [R_cm, t_cm]
    const Eigen::Matrix3d R_cm = result.rotation;
    const Eigen::Vector3d t_cm = result.position;

    // Invert to get T_marker_camera = [R_mc, t_mc]
    const Eigen::Matrix3d R_mc = R_cm.transpose();
    const Eigen::Vector3d t_mc = -R_mc * t_cm;

    // Optional alignment so first odom = (0,0,0)
    Eigen::Matrix3d R_use = R_mc;
    Eigen::Vector3d t_use = t_mc;
    if (config_.align_first_measurement)
    {
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

void FourMarkerWhyCodeTriangulationNode::maybeInitAlignment(
    const Eigen::Matrix3d& R_mc, const Eigen::Vector3d& t_mc)
{
    if (have_alignment_)
        return;

    // We want (R_align * R_mc, R_align * t_mc + t_align) = (I, 0)
    // Choose R_align = R_mc^T, t_align = -R_align * t_mc
    R_align_        = R_mc.transpose();
    t_align_        = -R_align_ * t_mc;
    have_alignment_ = true;

    RCLCPP_INFO(get_logger(), "Initialized camera odom alignment: start at x=0,y=0,theta=0");
}

void FourMarkerWhyCodeTriangulationNode::publishCameraOdometry(const CameraOdometry& odom)
{
    nav_msgs::msg::Odometry odom_msg;

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

    camera_odom_publisher_->publish(odom_msg);
}

void FourMarkerWhyCodeTriangulationNode::compareWithGroundTruth(const CameraOdometry& calculated_odom)
{
    if (!ground_truth_received_)
    {
        return;
    }

    // Extract ground truth 2D pose
    double gt_x     = latest_ground_truth_.pose.pose.position.x;
    double gt_y     = latest_ground_truth_.pose.pose.position.y;
    double gt_theta = tf2::getYaw(latest_ground_truth_.pose.pose.orientation);

    if (!gt_aligned_)
    {
        gt_x0_      = gt_x;
        gt_y0_      = gt_y;
        gt_theta0_  = gt_theta;
        gt_aligned_ = true;
        RCLCPP_INFO(get_logger(), "Initialized ground truth alignment to start at 0,0,0");
    }
    gt_x -= gt_x0_;
    gt_y -= gt_y0_;
    gt_theta = normalizeAngle(gt_theta - gt_theta0_);

    // Calculate errors
    double error_x     = calculated_odom.x - gt_x;
    double error_y     = calculated_odom.y - gt_y;
    double error_theta = normalizeAngle(calculated_odom.theta - gt_theta);

    // Calculate distance error
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

    // Log comparison (throttled to avoid spam)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Odometry Comparison:");
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "  Calculated: x=%.3f, y=%.3f, theta=%.3f°",
        calculated_odom.x,
        calculated_odom.y,
        calculated_odom.theta * 180.0 / M_PI);
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "  Ground Truth: x=%.3f, y=%.3f, theta=%.3f°",
        gt_x,
        gt_y,
        gt_theta * 180.0 / M_PI);
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "  Errors: dx=%.3f, dy=%.3f, dtheta=%.3f°, distance=%.3f",
        error_x,
        error_y,
        error_theta * 180.0 / M_PI,
        distance_error);

    // Log warnings for large errors (red), success for small errors (green)
    if (distance_error > 0.1)
    {  // 10cm threshold
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "\033[1;31mLarge position error detected: " << distance_error << " m\033[0m");
    }
    else
    {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\033[1;32mPosition error within threshold: " << distance_error << " m\033[0m");
    }

    if (std::abs(error_theta) > 0.175)
    {  // ~10 degree threshold
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "\033[1;31mLarge orientation error detected: " << std::abs(error_theta) * 180.0 / M_PI
                                                           << " degrees\033[0m");
    }
    else
    {
        RCLCPP_INFO_STREAM(
            get_logger(),
            "\033[1;32mOrientation error within threshold: " << std::abs(error_theta) * 180.0 / M_PI
                                                             << " degrees\033[0m");
    }
}

double FourMarkerWhyCodeTriangulationNode::normalizeAngle(double angle) const
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

bool FourMarkerWhyCodeTriangulationNode::validateInputs(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, double known_distance) const
{
    // Check for NaN or inf
    if (!P1.allFinite() || !P2.allFinite())
    {
        RCLCPP_ERROR(get_logger(), "Invalid input: P1 or P2 contains NaN or inf.");
        return false;
    }

    // Check known distance
    if (known_distance <= MIN_KNOWN_DISTANCE)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid input: known_distance must be positive and > %.2e",
            MIN_KNOWN_DISTANCE);
        return false;
    }

    // Check if markers are too close
    const double observed_distance = (P2 - P1).norm();
    if (observed_distance < MIN_DISTANCE)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid input: Markers are too close to each other: %.3f m",
            observed_distance);
        return false;
    }
    return true;
}

FourMarkerWhyCodeTriangulationNode::CameraPlaneAngles
FourMarkerWhyCodeTriangulationNode::calculateCameraPlaneAngles(
    const Eigen::Vector3d& plane_normal) const
{
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
    if (yz_projection.norm() > 1e-6)
    {
        angles.roll = std::atan2(yz_projection.x(), yz_projection.y());  // atan2(ny, nz)
    }
    else
    {
        angles.roll = 0.0;
    }

    // Calculate pitch (rotation around Y-axis)
    // Project normal onto XZ plane and measure angle from Z-axis
    Eigen::Vector2d xz_projection(n.x(), n.z());
    if (xz_projection.norm() > 1e-6)
    {
        angles.pitch = -std::atan2(xz_projection.x(), xz_projection.y());  // -atan2(nx, nz)
    }
    else
    {
        angles.pitch = 0.0;
    }

    // Calculate yaw (rotation around Z-axis)
    // Project normal onto XY plane and measure angle from X-axis
    Eigen::Vector2d xy_projection(n.x(), n.y());
    if (xy_projection.norm() > 1e-6)
    {
        angles.yaw = std::atan2(xy_projection.y(), xy_projection.x());  // atan2(ny, nx)
    }
    else
    {
        angles.yaw = 0.0;
    }

    return angles;
}

void FourMarkerWhyCodeTriangulationNode::publishTransform(
    const PoseEstimationResult& result, const std::string& frame_name)
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp    = now();
    transform_stamped.header.frame_id = config_.camera_frame;  // camera_link
    transform_stamped.child_frame_id  = frame_name;

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

void FourMarkerWhyCodeTriangulationNode::publishPose(const PoseEstimationResult& result)
{
    geometry_msgs::msg::PoseStamped pose_msg = eigenToRosPose(result.position, result.rotation);
    pose_publisher_->publish(pose_msg);
}

geometry_msgs::msg::PoseStamped FourMarkerWhyCodeTriangulationNode::eigenToRosPose(
    const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation) const
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp    = now();
    pose.header.frame_id = config_.camera_frame;

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
