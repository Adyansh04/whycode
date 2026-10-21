#include "whycode/ros/whycon_ros_interface.hpp"

#include <rmw/qos_profiles.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
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

    debug_manager_ = std::make_unique<DebugImageManager>(targets_);
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
        for (int p = 0; p < DebugImageManager::PANEL_COUNT; p++)
        {
            debug_pubs_[p] = node_->create_publisher<sensor_msgs::msg::Image>(
                debug_images_topic + "/" + DebugImageManager::PANEL_NAMES[p],
                1);
        }
    }

    if (publish_visualization_markers_)
    {
        visualization_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            visualization_markers_topic,
            1);

        // Pre-allocate marker array buffer: a disc and a label per marker, after the clearing one
        marker_array_buffer_.markers.reserve(2 * targets_ + 1);
        // No namespace: RViz then clears every marker of the topic, discs and labels, and the
        // clearing marker cannot share an ns and id with one it precedes
        delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
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
    // A debug panel costs a full-frame copy and its drawing: only built for a subscriber
    DebugImageManager* debug = nullptr;
    if (publish_debug_images_)
    {
        for (int p = 0; p < DebugImageManager::PANEL_COUNT; p++)
        {
            debug_manager_->setWanted(
                static_cast<DebugImageManager::Panel>(p),
                debug_pubs_[p]->get_subscription_count() > 0U);
        }
        if (debug_manager_->any())
        {
            debug_manager_->beginFrame(*image_handler_);
            debug = debug_manager_.get();
        }
    }

    is_tracking_  = system_->localizeMarkers(*image_handler_, should_reset_, debug);
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
    publishResults(latest_header_, debug);

    if (debug != nullptr)
    {
        for (int p = 0; p < DebugImageManager::PANEL_COUNT; p++)
        {
            const auto panel = static_cast<DebugImageManager::Panel>(p);
            if (debug->wants(panel))
            {
                cv_bridge::CvImage(latest_header_, "rgb8", debug->image(panel))
                    .toImageMsg(debug_msgs_[p]);
                debug_pubs_[p]->publish(debug_msgs_[p]);
            }
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

void whycon::WhyconRosInterface::publishResults(
    const std_msgs::msg::Header& header, DebugImageManager* debug)
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

    // RViz markers likewise; each array first clears the last frame's
    const bool draw_markers = publish_visualization_markers_ && visualization_markers_pub_ &&
                              visualization_markers_pub_->get_subscription_count() > 0U;
    int marker_count = 0;
    if (draw_markers)
    {
        marker_array_buffer_.markers.clear();
        marker_array_buffer_.markers.push_back(delete_all_marker_);
    }

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
            debug);

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
        if (draw_markers)
        {
            createMarkerVisualization(
                pose_buffer_,
                marker_count,
                header,
                marker_buffer_,
                marker_label_buffer_);
            marker_array_buffer_.markers.push_back(marker_buffer_);
            marker_array_buffer_.markers.push_back(marker_label_buffer_);
            ++marker_count;
        }

        // PUBLISH TF IMMEDIATELY using the same pose data
        if (publish_tf_)
        {
            publishSingleTF(pose_buffer_, header);
        }
    }

    if (draw_image)
    {
        image_pub_->publish(output_image_msg_);
    }

    if (draw_markers)
    {
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
    const whycon::LocalizationSystem::MarkerPose& pose, const std_msgs::msg::Header& header)
{
    // Stamped with the frame's capture time, as the pose array is: the processing time is later,
    // and a lookup at the image's stamp would extrapolate. A marker without an ID (or WhyCon mode)
    // gets its track's frame, rather than every such marker sharing <prefix>-1.
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = header.stamp;
    transform_stamped.header.frame_id = parent_frame_id_;
    transform_stamped.child_frame_id =
        pose.id_valid ? tf_frame_prefix_ + std::to_string(pose.ID) :
                        tf_frame_prefix_ + "track_" + std::to_string(pose.tracking_id);

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
    const std_msgs::msg::Header& header, visualization_msgs::msg::Marker& marker,
    visualization_msgs::msg::Marker& label)
{
    // Basic marker properties; ids are per frame, so undecoded markers (ID -1) stay apart. The
    // pose is in the parent frame, as in the pose array, not in the image's frame
    marker.header.stamp    = header.stamp;
    marker.header.frame_id = parent_frame_id_;
    marker.ns              = "whycon_markers";
    marker.id              = marker_index;
    marker.action          = visualization_msgs::msg::Marker::ADD;

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

    // The pose's normal is its x axis and a cylinder's axis is its z: turn z onto x, 90 deg about y
    const float      half     = std::numbers::sqrt2_v<float> / 2;
    const Quaternion disc     = pose.orientation * Quaternion(0.0f, half, 0.0f, half);
    marker.pose.orientation.x = disc.x;
    marker.pose.orientation.y = disc.y;
    marker.pose.orientation.z = disc.z;
    marker.pose.orientation.w = disc.w;

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
    // Rounding leaves a component a hair outside [0, 1] (ID 6: blue -1.2e-7), and RViz then
    // rejects the marker
    marker.color.r = std::clamp(marker.color.r, 0.0f, 1.0f);
    marker.color.g = std::clamp(marker.color.g, 0.0f, 1.0f);
    marker.color.b = std::clamp(marker.color.b, 0.0f, 1.0f);

    // Set marker lifetime
    // marker.lifetime = rclcpp::Duration::from_seconds(0.1).to_msg();  // Auto-delete after 100ms if not updated
    marker.lifetime =
        rclcpp::Duration::from_seconds(0.1);  // Auto-delete after 100ms if not updated

    // The ID and range on a label under the disc, as a cylinder shows no text; the ground truth's
    // label sits above its plate
    label.header        = marker.header;
    label.ns            = "whycon_labels";
    label.id            = marker_index;
    label.action        = visualization_msgs::msg::Marker::ADD;
    label.type          = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position = marker.pose.position;
    label.pose.position.z -= parameters_.outer_diameter;
    label.pose.orientation.w = 1.0;
    label.scale.z            = 0.12;  // Text height (m), as the ground truth's labels
    label.color              = marker.color;
    label.color.a            = 1.0f;
    label.lifetime           = marker.lifetime;
    std::array<char, 32> text{};
    if (pose.id_valid && pose.ID != -1)
    {
        std::snprintf(text.data(), text.size(), "id%d %.2fm", pose.ID, cv::norm(pose.pos));
    }
    else
    {
        std::snprintf(text.data(), text.size(), "id? %.2fm", cv::norm(pose.pos));
    }
    label.text = text.data();
}
