#include "whycode/ros/whycon_ros_interface.hpp"

#include <rmw/qos_profiles.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

#include "whycode/utils/whycon_config.h"

whycon::WhyconRosInterface::WhyconRosInterface(rclcpp::Node* node)
  : node_(node)
{
    if (node_ == nullptr)
    {
        throw std::invalid_argument("WhyconRosInterface received null node pointer");
    }

    // Load all parameters from the config file
    loadParameters();

    // Initialize core WhyCon modules
    initializeWhyConModules();

    // Setup ROS subscribers and publishers
    setupROSTopics();

    // Pre-allocate buffers for performance
    whycode_pose_array.poses.reserve(targets_);
    valid_detections_buffer_.reserve(targets_);
    removed_ids_buffer_.reserve(targets_);
}

void whycon::WhyconRosInterface::loadParameters()
{
    std::string default_config_file;
    try
    {
        default_config_file = ament_index_cpp::get_package_share_directory("whycode_vision") +
                              "/config/whycon_config_rs.yaml";
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node_->get_logger(),
            "Failed to resolve package share directory for whycode_vision: %s",
            e.what());
        rclcpp::shutdown();
        return;
    }

    std::string config_file_path =
        node_->declare_parameter<std::string>("config_file", default_config_file);
    if (config_file_path.empty())
    {
        config_file_path = default_config_file;
        RCLCPP_WARN(
            node_->get_logger(),
            "Empty config_file parameter, using default: %s",
            config_file_path.c_str());
    }

    try
    {
        whycon::WhyConConfig::initialize(config_file_path);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node_->get_logger(),
            "Failed to initialize WhyCon configuration from %s: %s",
            config_file_path.c_str(),
            e.what());
        rclcpp::shutdown();
        return;
    }

    const auto& params = whycon::WhyConConfig::getParamLoader();

    // System parameters
    targets_         = params.getParams<int>("system", "targets");
    process_rate_hz_ = params.getParams<double>("system", "process_rate_hz");

    // Detector parameters
    parameters_.min_size = params.getParams<int>("detector", "min_size");
    parameters_.max_size = params.getParams<int>("detector", "max_size");
    parameters_.center_distance_tolerance_ratio =
        params.getParams<double>("detector", "center_distance_tolerance_ratio");
    parameters_.center_distance_tolerance_abs =
        params.getParams<double>("detector", "center_distance_tolerance_abs");
    parameters_.roundness_tolerance = params.getParams<double>("detector", "roundness_tolerance");
    parameters_.circularity_tolerance =
        params.getParams<double>("detector", "circularity_tolerance");
    parameters_.ratio_tolerance    = params.getParams<double>("detector", "ratio_tolerance");
    parameters_.max_eccentricity   = params.getParams<double>("detector", "max_eccentricity");
    parameters_.inner_diameter     = params.getParams<double>("detector", "inner_diameter");
    parameters_.outer_diameter     = params.getParams<double>("detector", "outer_diameter");
    parameters_.image_gamma        = params.getParams<double>("detector", "image_gamma");
    parameters_.full_scan_interval = params.getParams<int>("detector", "full_scan_interval");

    // Identification parameters
    parameters_.identify         = params.getParams<bool>("identification", "enabled");
    parameters_.id_bits          = params.getParams<int>("identification", "id_bits");
    parameters_.id_samples       = params.getParams<int>("identification", "id_samples");
    parameters_.hamming_distance = params.getParams<int>("identification", "hamming_distance");
    parameters_.ambiguity_noise_px =
        params.getParams<float>("identification", "ambiguity_noise_px");
    parameters_.min_marker_pixels = params.getParams<int>("identification", "min_marker_pixels");
    parameters_.diameter_ratio_correction =
        params.getParams<float>("identification", "diameter_ratio_correction");

    // Tracking parameters
    min_track_age_ = params.getParams<int>("tracking", "min_track_age");

    // ROS interface parameters
    publish_poses_        = params.getParams<bool>("ros_interface", "publish_poses");
    publish_images_       = params.getParams<bool>("ros_interface", "publish_images");
    publish_debug_images_ = params.getParams<bool>("ros_interface", "publish_debug_images");
    publish_tf_           = params.getParams<bool>("ros_interface", "publish_tf");
    publish_visualization_markers_ =
        params.getParams<bool>("ros_interface", "publish_visualization_markers");

    tf_frame_prefix_ = params.getParams<std::string>("ros_interface", "tf_frame_prefix");
    parent_frame_id_ = params.getParams<std::string>("ros_interface", "parent_frame_id");

    // Camera parameters
    cam_height_ = params.getParams<int>("camera", "image_height");
    cam_width_  = params.getParams<int>("camera", "image_width");

    auto camera_package_name = params.getParams<std::string>("camera", "package_name");
    auto camera_config_path  = params.getParams<std::string>("camera", "config_path");

    std::string camera_file_path;
    try
    {
        camera_file_path = ament_index_cpp::get_package_share_directory(camera_package_name) + "/" +
                           camera_config_path;
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(
            node_->get_logger(),
            "Failed to resolve package share directory for %s: %s",
            camera_package_name.c_str(),
            e.what());
        rclcpp::shutdown();
        return;
    }

    if (params.loadCameraIntrinsics(camera_file_path, camera_matrix_, distortion_coeffs_))
    {
        camera_params_loaded_ = true;
        RCLCPP_INFO(
            node_->get_logger(),
            "Camera parameters loaded successfully from: %s",
            camera_file_path.c_str());
    }
    else
    {
        RCLCPP_FATAL(
            node_->get_logger(),
            "Failed to load camera parameters from %s. Shutting down.",
            camera_file_path.c_str());
        rclcpp::shutdown();
    }
}

void whycon::WhyconRosInterface::initializeWhyConModules()
{
    const auto& params               = whycon::WhyConConfig::getParamLoader();
    double      max_association_dist = params.getParams<double>("tracking", "max_association_dist");
    int         max_unseen_frames    = params.getParams<int>("tracking", "max_unseen_frames");
    int         id_switch_threshold  = params.getParams<int>("tracking", "id_switch_threshold");
    float       pose_alpha           = params.getParams<float>("tracking", "pose_alpha");
    std::string image_encoding = params.getParams<std::string>("ros_interface", "image_encoding");

    debug_manager_ = std::make_unique<DebugImageManager>(publish_debug_images_);
    tracker_ =
        std::make_unique<MarkerTracker>(max_unseen_frames, max_association_dist, min_track_age_);
    id_stabilizer_      = std::make_unique<IDStabilizer>(id_switch_threshold);
    pose_stabilizer_    = std::make_unique<PoseStabilizer>(pose_alpha);
    ambiguity_resolver_ = std::make_unique<AmbiguityResolver>();
    system_             = std::make_unique<whycon::LocalizationSystem>(
        targets_,
        cam_width_,
        cam_height_,
        camera_matrix_,
        distortion_coeffs_,
        parameters_);

    if (publish_tf_)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }

    image_handler_ = std::make_unique<ImageHandler>(cam_width_, cam_height_, image_encoding);
}

void whycon::WhyconRosInterface::setupROSTopics()
{
    const auto& params           = whycon::WhyConConfig::getParamLoader();
    int         input_queue_size = params.getParams<int>("ros_interface", "input_queue_size");
    bool        input_reliable   = params.getParams<bool>("ros_interface", "input_reliable");
    std::string image_topic = params.getParams<std::string>("ros_interface", "image_input_topic");
    std::string poses_topic = params.getParams<std::string>("ros_interface", "poses_output_topic");
    std::string image_out_topic =
        params.getParams<std::string>("ros_interface", "image_output_topic");
    std::string debug_images_topic =
        params.getParams<std::string>("ros_interface", "debug_images_topic");
    std::string visualization_markers_topic =
        params.getParams<std::string>("ros_interface", "visualization_markers_topic");

    std::string detection_enabled_service =
        params.getParams<std::string>("ros_interface", "detection_enabled_service_name");

    detection_control_service_ = node_->create_service<std_srvs::srv::SetBool>(
        detection_enabled_service,
        std::bind(
            &WhyconRosInterface::detectionControlCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // Best effort matches every camera driver but loses most large frames from a reliable
    // publisher (bag playback, Isaac); a reliable subscription cannot match a best-effort one.
    rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
    image_qos.depth             = static_cast<size_t>(input_queue_size);
    image_qos.reliability       = input_reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE :
                                                   RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    // Intra-process: a camera driver in the same container, publishing with intra-process on,
    // hands its frames over as pointers. Over DDS, the recycled message spares the zero-fill.
    rclcpp::SubscriptionOptions image_options;
    image_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    image_sub_                           = node_->create_subscription<sensor_msgs::msg::Image>(
        image_topic,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_qos), image_qos),
        std::bind(&WhyconRosInterface::onRosImageReceived, this, std::placeholders::_1),
        image_options,
        std::make_shared<RecycledImageStrategy>());

    if (publish_poses_)
    {
        whycode_pose_pub_ =
            node_->create_publisher<whycode_vision::msg::WhyCodePoseArray>(poses_topic, 1);
    }

    if (publish_images_)
    {
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(image_out_topic, 1);
    }

    if (publish_debug_images_)
    {
        debug_images_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(debug_images_topic, 1);
    }

    if (publish_visualization_markers_)
    {
        visualization_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            visualization_markers_topic,
            1);

        // Pre-allocate marker array buffer
        marker_array_buffer_.markers.reserve(targets_);
    }
}

void whycon::WhyconRosInterface::onRosImageReceived(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
    if (!detection_enabled_ || !camera_params_loaded_)
    {
        return;
    }

    // Update image handler
    if (!image_handler_->updateFromROS(*image_msg))
    {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "Dropping %ux%u %s frame (step %u): the detector is configured for %dx%d %s",
            image_msg->width,
            image_msg->height,
            image_msg->encoding.c_str(),
            image_msg->step,
            cam_width_,
            cam_height_,
            image_handler_->getEncoding().c_str());
        return;
    }

    // Store latest header
    latest_header_        = image_msg->header;
    latest_ros_timestamp_ = rclcpp::Time(image_msg->header.stamp);

    // Process immediately in subscription callback to avoid polling/timer latency
    processLatestFrame();
}

void whycon::WhyconRosInterface::processLatestFrame()
{
    if (debug_manager_)
    {
        debug_manager_->clearDebugImages();
        // Debug images cost several full-frame copies each: only build them for a subscriber
        debug_manager_->setEnabled(
            publish_debug_images_ && debug_images_pub_ &&
            debug_images_pub_->get_subscription_count() > 0U);
    }

    is_tracking_  = system_->localizeMarkers(*image_handler_, should_reset_, debug_manager_.get());
    should_reset_ = false;

    // Valid detections to avoid extra allocations
    valid_detections_buffer_.clear();
    for (auto& marker : system_->detector.outer_circles)
    {
        if (marker.valid)
        {
            valid_detections_buffer_.push_back(&marker);
        }
    }

    // Update trackers and stabilizers
    tracker_->update(valid_detections_buffer_, *image_handler_, removed_ids_buffer_);
    id_stabilizer_->removeTracks(removed_ids_buffer_);
    pose_stabilizer_->removeTracks(removed_ids_buffer_);
    ambiguity_resolver_->removeTracks(removed_ids_buffer_);

    // Publish results
    publishResults(latest_header_);

    // Publish consolidated debug images if enabled
    if (publish_debug_images_ && debug_manager_->isEnabled() && debug_images_pub_ &&
        debug_images_pub_->get_subscription_count() > 0U)
    {
        cv::Mat consolidated_debug = debug_manager_->createConsolidatedImage();
        if (!consolidated_debug.empty())
        {
            cv_bridge::CvImage cv_image(latest_header_, "rgb8", consolidated_debug);
            debug_images_pub_->publish(*cv_image.toImageMsg());
        }
    }
}

void whycon::WhyconRosInterface::detectionControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response>      res)
{
    detection_enabled_ = req->data;
    res->success       = true;
    res->message       = detection_enabled_ ? "Detection enabled." : "Detection disabled.";
    RCLCPP_INFO(
        node_->get_logger(),
        "[Whycon] Detection %s via service call.",
        detection_enabled_ ? "enabled" : "disabled");
}

void whycon::WhyconRosInterface::publishResults(const std_msgs::msg::Header& header)
{
    // Converting and drawing the frame costs time on every frame: only for a subscriber. The
    // frame is copied once, into the reused outgoing message, and drawn on there.
    const bool draw_image =
        publish_images_ && image_pub_ && image_pub_->get_subscription_count() > 0U;
    cv::Mat output_image;
    if (draw_image)
    {
        output_image_msg_.header   = header;
        output_image_msg_.height   = cam_height_;
        output_image_msg_.width    = cam_width_;
        output_image_msg_.encoding = image_handler_->colourEncoding();
        output_image_msg_.step     = cam_width_ * 3;
        output_image_msg_.data.resize(static_cast<size_t>(cam_height_) * cam_width_ * 3);
        output_image = cv::Mat(cam_height_, cam_width_, CV_8UC3, output_image_msg_.data.data());
        image_handler_->copyColour(output_image);
    }

    int marker_count = 0;
    marker_array_buffer_.markers.clear();

    // Clear and reuse pose array buffer
    whycode_pose_array.poses.clear();
    whycode_pose_array.header          = header;
    whycode_pose_array.header.frame_id = parent_frame_id_;

    // go through detected targets
    for (int i = 0; i < targets_; i++)
    {
        if (!system_->isMarkerDetected(i))
        {
            continue;
        }

        const auto& inner_circle = system_->getMarkerByID(i);
        const auto& outer_circle = system_->getOuterMarkerByID(i);

        if (!(outer_circle.tracking_id != -1 &&
              tracker_->getTrackAge(outer_circle.tracking_id) >= min_track_age_))
        {
            // Skip markers that are not tracked or too young
            WHYCON_DEBUG("Skipping marker ID " << i << " - not tracked or too young.");
            continue;
        }

        system_->estimateMarkerPose(
            *image_handler_,
            outer_circle,
            system_->getMarkerByID(i),
            pose_buffer_,
            debug_manager_.get());

        // Choose between the pose's two solutions over the whole track, then stabilize ID and pose
        ambiguity_resolver_->resolve(pose_buffer_);
        pose_buffer_.ID = id_stabilizer_->stabilize(pose_buffer_.tracking_id, pose_buffer_.ID);
        pose_buffer_.id_valid = (pose_buffer_.ID != -1);
        pose_buffer_.pos = pose_stabilizer_->stabilize(pose_buffer_.tracking_id, pose_buffer_.pos);

        // draw each target
        if (draw_image)
        {
            std::snprintf(
                label_buffer_.data(),
                label_buffer_.size(),
                "id %d  track %d  age %d\nxyz %.2f %.2f %.2f m\nrpy %.0f %.0f %.0f deg  p %.2f",
                pose_buffer_.ID,
                pose_buffer_.tracking_id,
                tracker_->getTrackAge(outer_circle.tracking_id).value_or(-1),
                pose_buffer_.pos(0),
                pose_buffer_.pos(1),
                pose_buffer_.pos(2),
                pose_buffer_.roll * RAD_TO_DEG,
                pose_buffer_.pitch * RAD_TO_DEG,
                pose_buffer_.yaw * RAD_TO_DEG,
                pose_buffer_.orientation_confidence);

            outer_circle.draw(output_image, label_buffer_.data(), cv::Vec3b(0, 255, 0));
            inner_circle.draw(output_image, {}, cv::Vec3b(255, 0, 255));
        }

        if (publish_poses_)
        {
            // Fill custom message
            whycode_pose_msg_buffer_.whycode_id             = pose_buffer_.ID;
            whycode_pose_msg_buffer_.tracking_id            = pose_buffer_.tracking_id;
            whycode_pose_msg_buffer_.id_valid               = pose_buffer_.id_valid;
            whycode_pose_msg_buffer_.orientation_confidence = pose_buffer_.orientation_confidence;
            whycode_pose_msg_buffer_.pose.position.x        = pose_buffer_.pos(0);
            whycode_pose_msg_buffer_.pose.position.y        = pose_buffer_.pos(1);
            whycode_pose_msg_buffer_.pose.position.z        = pose_buffer_.pos(2);
            whycode_pose_msg_buffer_.pose.orientation.x     = pose_buffer_.orientation.x;
            whycode_pose_msg_buffer_.pose.orientation.y     = pose_buffer_.orientation.y;
            whycode_pose_msg_buffer_.pose.orientation.z     = pose_buffer_.orientation.z;
            whycode_pose_msg_buffer_.pose.orientation.w     = pose_buffer_.orientation.w;

            whycode_pose_array.poses.push_back(whycode_pose_msg_buffer_);
        }

        // Create Marker visualization
        if (publish_visualization_markers_)
        {
            createMarkerVisualization(pose_buffer_, marker_count, header, marker_buffer_);
            marker_array_buffer_.markers.push_back(marker_buffer_);
            ++marker_count;
        }

        // PUBLISH TF IMMEDIATELY using the same pose data
        if (publish_tf_)
        {
            publishSingleTF(pose_buffer_, pose_buffer_.ID);
        }
    }

    if (draw_image)
    {
        image_pub_->publish(output_image_msg_);
    }

    // Publish RViz markers (delete stale ones to avoid leftovers)
    if (publish_visualization_markers_ && visualization_markers_pub_ &&
        visualization_markers_pub_->get_subscription_count() > 0U)
    {
        for (int i = marker_count; i < last_marker_count_; ++i)
        {
            visualization_msgs::msg::Marker del;
            del.header = header;
            del.ns     = "whycon_markers";
            del.id     = i;
            del.action = visualization_msgs::msg::Marker::DELETE;
            marker_array_buffer_.markers.push_back(std::move(del));
        }
        last_marker_count_ = marker_count;

        visualization_markers_pub_->publish(marker_array_buffer_);
    }

    if (publish_poses_ && whycode_pose_pub_)
    {
        whycode_pose_array.detected_marker_count =
            static_cast<int32_t>(whycode_pose_array.poses.size());
        whycode_pose_pub_->publish(whycode_pose_array);
    }
}

void whycon::WhyconRosInterface::publishSingleTF(
    const whycon::LocalizationSystem::MarkerPose& pose, int track_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = node_->now();
    transform_stamped.header.frame_id = parent_frame_id_;
    transform_stamped.child_frame_id  = tf_frame_prefix_ + std::to_string(track_id);

    transform_stamped.transform.translation.x = pose.pos(0);
    transform_stamped.transform.translation.y = pose.pos(1);
    transform_stamped.transform.translation.z = pose.pos(2);
    transform_stamped.transform.rotation.x    = pose.orientation.x;
    transform_stamped.transform.rotation.y    = pose.orientation.y;
    transform_stamped.transform.rotation.z    = pose.orientation.z;
    transform_stamped.transform.rotation.w    = pose.orientation.w;

    tf_broadcaster_->sendTransform(transform_stamped);
}

void whycon::WhyconRosInterface::createMarkerVisualization(
    const whycon::LocalizationSystem::MarkerPose& pose, int marker_index,
    const std_msgs::msg::Header& header, visualization_msgs::msg::Marker& marker)
{
    (void)marker_index;

    // Basic marker properties
    marker.header = header;
    marker.ns     = "whycon_markers";
    marker.id     = pose.ID;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker type and scale
    marker.type =
        visualization_msgs::msg::Marker::CYLINDER;  // Use cylinder to represent circular markers
    marker.scale.x = parameters_.outer_diameter;    // Diameter
    marker.scale.y = parameters_.outer_diameter;    // Diameter
    marker.scale.z = 0.01;                          // Height (thin disk)

    // Set position from pose
    marker.pose.position.x = pose.pos[0];
    marker.pose.position.y = pose.pos[1];
    marker.pose.position.z = pose.pos[2];

    // Set orientation from quaternion
    marker.pose.orientation.x = pose.orientation.x;
    marker.pose.orientation.y = pose.orientation.y;
    marker.pose.orientation.z = pose.orientation.z;
    marker.pose.orientation.w = pose.orientation.w;

    // Set color based on ID validity and ID value
    if (pose.id_valid && pose.ID != -1)
    {
        // Color based on ID (cycling through colors)
        float hue = (pose.ID * 60.0f) / 360.0f;  // Spread IDs across color wheel
        while (hue > 1.0f)
        {
            hue -= 1.0f;
        }

        // Convert HSV to RGB (simple approximation)
        if (hue < 1.0f / 6.0f)
        {
            marker.color.r = 1.0f;
            marker.color.g = hue * 6.0f;
            marker.color.b = 0.0f;
        }
        else if (hue < 2.0f / 6.0f)
        {
            marker.color.r = 1.0f - (hue - 1.0f / 6.0f) * 6.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        }
        else if (hue < 3.0f / 6.0f)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = (hue - 2.0f / 6.0f) * 6.0f;
        }
        else if (hue < 4.0f / 6.0f)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f - (hue - 3.0f / 6.0f) * 6.0f;
            marker.color.b = 1.0f;
        }
        else if (hue < 5.0f / 6.0f)
        {
            marker.color.r = (hue - 4.0f / 6.0f) * 6.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f - (hue - 5.0f / 6.0f) * 6.0f;
        }
        marker.color.a = 0.8f;  // Semi-transparent
    }
    else
    {
        // Gray for unknown ID
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 0.6f;
    }

    // Set marker lifetime
    // marker.lifetime = rclcpp::Duration::from_seconds(0.1).to_msg();  // Auto-delete after 100ms if not updated
    marker.lifetime =
        rclcpp::Duration::from_seconds(0.1);  // Auto-delete after 100ms if not updated

    // Add text marker for ID display
    if (pose.id_valid && pose.ID != -1)
    {
        marker.text = std::to_string(pose.ID);
    }
    else
    {
        marker.text = "?";
    }
}
