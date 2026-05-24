#include "whycode/ros/whycon_ros_interface.hpp"

#include <ros/package.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>

#include "ros/forwards.h"
#include "whycode/utils/coord_lut.hpp"
#include "whycode/utils/whycon_config.h"

whycon::WhyconRosInterface::WhyconRosInterface(ros::NodeHandle& n) : it_(n) {
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

    // Build global coord LUT once for these dimensions
    whycon::lut::ensure(cam_width_, cam_height_);
}

void whycon::WhyconRosInterface::loadParameters() {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;  // Use global namespace

    std::string config_file_path;
    if (!private_nh.getParam("config_file", config_file_path)) {
        // Fallback to the exact parameter name that's set in launch file
        ROS_WARN("No config_file parameter specified in private namespace, checking global namespace.");
        // Try to get the config_file parameter from the global namespace
        bool found = nh.getParam("/whycon_nodelet/config_file", config_file_path);
        if (found) {
            ROS_WARN("Found config_file parameter in global namespace: %s", config_file_path.c_str());
        } else {
            // Default configuration file path
            config_file_path = ros::package::getPath("whycon_whycode_localization") + "/config/whycon_config_rs.yaml";
            ROS_WARN("No config_file parameter specified, using default: %s", config_file_path.c_str());
        }
    }
    try {
        whycon::WhyConConfig::initialize(config_file_path);
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to initialize WhyCon configuration from %s: %s", config_file_path.c_str(), e.what());
        ros::shutdown();
        return;
    }

    const auto& params = whycon::WhyConConfig::getParamLoader();

    // Input source and processing rate

    // System parameters
    targets_                     = params.getParams<int>("system", "targets");
    std::string input_source_str = params.getParams<std::string>("system", "input_source");
    if (input_source_str == "iceoryx") {
        input_source_ = InputSource::ICEORYX;
    } else {
        input_source_ = InputSource::ROS;
    }

    process_rate_hz_ = params.getParams<double>("system", "process_rate_hz");

    // Detector parameters
    parameters_.min_size = params.getParams<int>("detector", "min_size");
    parameters_.max_size = params.getParams<int>("detector", "max_size");
    parameters_.center_distance_tolerance_ratio =
            params.getParams<double>("detector", "center_distance_tolerance_ratio");
    parameters_.center_distance_tolerance_abs = params.getParams<double>("detector", "center_distance_tolerance_abs");
    parameters_.roundness_tolerance           = params.getParams<double>("detector", "roundness_tolerance");
    parameters_.circularity_tolerance         = params.getParams<double>("detector", "circularity_tolerance");
    parameters_.ratio_tolerance               = params.getParams<double>("detector", "ratio_tolerance");
    parameters_.max_eccentricity              = params.getParams<double>("detector", "max_eccentricity");
    parameters_.inner_diameter                = params.getParams<double>("detector", "inner_diameter");
    parameters_.outer_diameter                = params.getParams<double>("detector", "outer_diameter");

    double outer_diameter_multiplier = params.getParams<double>("detector", "outer_diameter_multiplier");
    parameters_.outer_diameter *= outer_diameter_multiplier;

    // Identification parameters
    parameters_.identify                  = params.getParams<bool>("identification", "enabled");
    parameters_.id_bits                   = params.getParams<int>("identification", "id_bits");
    parameters_.id_samples                = params.getParams<int>("identification", "id_samples");
    parameters_.hamming_distance          = params.getParams<int>("identification", "hamming_distance");
    parameters_.variance_threshold        = params.getParams<float>("identification", "variance_threshold");
    parameters_.min_marker_pixels         = params.getParams<int>("identification", "min_marker_pixels");
    parameters_.diameter_ratio_correction = params.getParams<float>("identification", "diameter_ratio_correction");

    // Tracking parameters
    min_track_age_ = params.getParams<int>("tracking", "min_track_age");

    // ROS interface parameters
    publish_poses_                 = params.getParams<bool>("ros_interface", "publish_poses");
    publish_images_                = params.getParams<bool>("ros_interface", "publish_images");
    publish_debug_images_          = params.getParams<bool>("ros_interface", "publish_debug_images");
    publish_tf_                    = params.getParams<bool>("ros_interface", "publish_tf");
    publish_visualization_markers_ = params.getParams<bool>("ros_interface", "publish_visualization_markers");

    tf_frame_prefix_ = params.getParams<std::string>("ros_interface", "tf_frame_prefix");
    parent_frame_id_ = params.getParams<std::string>("ros_interface", "parent_frame_id");

    // Camera parameters
    cam_height_ = params.getParams<int>("camera", "image_height");
    cam_width_  = params.getParams<int>("camera", "image_width");

    auto        camera_package_name = params.getParams<std::string>("camera", "package_name");
    auto        camera_config_path  = params.getParams<std::string>("camera", "config_path");
    std::string camera_file_path    = ros::package::getPath(camera_package_name) + "/" + camera_config_path;

    if (params.loadCameraIntrinsics(camera_file_path, camera_matrix_, distortion_coeffs_)) {
        camera_params_loaded_ = true;
        ROS_INFO("Camera parameters loaded successfully from: %s", camera_file_path.c_str());
    } else {
        ROS_FATAL("Failed to load camera parameters from %s. Shutting down.", camera_file_path.c_str());
        ros::shutdown();
    }
}

void whycon::WhyconRosInterface::initializeWhyConModules() {
    const auto& params               = whycon::WhyConConfig::getParamLoader();
    double      max_association_dist = params.getParams<double>("tracking", "max_association_dist");
    int         max_unseen_frames    = params.getParams<int>("tracking", "max_unseen_frames");
    int         id_switch_threshold  = params.getParams<int>("tracking", "id_switch_threshold");
    float       pose_alpha           = params.getParams<float>("tracking", "pose_alpha");
    std::string image_encoding       = params.getParams<std::string>("ros_interface", "image_encoding");

    debug_manager_   = std::make_unique<DebugImageManager>(publish_debug_images_);
    tracker_         = std::make_unique<MarkerTracker>(max_unseen_frames, max_association_dist, min_track_age_);
    id_stabilizer_   = std::make_unique<IDStabilizer>(id_switch_threshold);
    pose_stabilizer_ = std::make_unique<PoseStabilizer>(pose_alpha);
    system_          = std::make_unique<whycon::LocalizationSystem>(targets_, cam_width_, cam_height_, camera_matrix_,
                                                           distortion_coeffs_, parameters_);

    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
    }

    // Load Iceoryx parameters from config
    std::string runtime_name     = "";
    std::string service_instance = "";
    std::string service_method   = "";
    std::string service_event    = "";

    if (input_source_ == InputSource::ICEORYX) {
        const auto& params = whycon::WhyConConfig::getParamLoader();
        runtime_name       = params.getParams<std::string>("iceoryx", "runtime_name");
        service_instance   = params.getParams<std::string>("iceoryx", "service_instance");
        service_method     = params.getParams<std::string>("iceoryx", "service_method");
        service_event      = params.getParams<std::string>("iceoryx", "service_event");
    }

    ImageHandler::Mode handler_mode =
            (input_source_ == InputSource::ICEORYX) ? ImageHandler::Mode::ICEORYX : ImageHandler::Mode::ROS;
    image_handler_ = std::make_unique<ImageHandler>(cam_width_, cam_height_, image_encoding, handler_mode, runtime_name,
                                                    service_instance, service_method, service_event);
    if (input_source_ == InputSource::ICEORYX) {
        image_handler_->setIceoryxCallback([this]() { this->onIceoryxImageReceived(); });
    }
}

void whycon::WhyconRosInterface::setupROSTopics() {
    ros::NodeHandle nh;
    const auto&     params             = whycon::WhyConConfig::getParamLoader();
    int             input_queue_size   = params.getParams<int>("ros_interface", "input_queue_size");
    std::string     image_topic        = params.getParams<std::string>("ros_interface", "image_input_topic");
    std::string     poses_topic        = params.getParams<std::string>("ros_interface", "poses_output_topic");
    std::string     image_out_topic    = params.getParams<std::string>("ros_interface", "image_output_topic");
    std::string     debug_images_topic = params.getParams<std::string>("ros_interface", "debug_images_topic");
    std::string     visualization_markers_topic =
            params.getParams<std::string>("ros_interface", "visualization_markers_topic");

    std::string detection_enabled_service =
            params.getParams<std::string>("ros_interface", "detection_enabled_service_name");

    detection_control_service_ =
            nh.advertiseService(detection_enabled_service, &WhyconRosInterface::detectionControlCallback, this);

    process_timer_ =
            nh.createTimer(ros::Duration(1.0 / process_rate_hz_), &WhyconRosInterface::processTimerCallback, this);

    if (input_source_ == InputSource::ROS)
        image_sub_ = it_.subscribe(image_topic, input_queue_size, &WhyconRosInterface::onRosImageReceived, this);

    if (publish_poses_)
        whycode_pose_pub_ = nh.advertise<whycon_whycode_localization::WhyCodePoseArray>(poses_topic, 1);

    if (publish_images_)
        image_pub_ = nh.advertise<sensor_msgs::Image>(image_out_topic, 1);

    if (publish_debug_images_)
        debug_images_pub_ = nh.advertise<sensor_msgs::Image>(debug_images_topic, 1);

    if (publish_visualization_markers_) {
        visualization_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(visualization_markers_topic, 1);

        // Pre-allocate marker array buffer
        marker_array_buffer_.markers.reserve(targets_);
    }
}

void whycon::WhyconRosInterface::onRosImageReceived(const sensor_msgs::ImageConstPtr& image_msg) {
    if (!detection_enabled_ || !camera_params_loaded_)
        return;

    // Update image handler
    if (!image_handler_->updateFromROS(image_msg)) {
        ROS_ERROR("Failed to update image from ROS message");
        return;
    }

    // Store latest header and signal new frame
    latest_header_        = image_msg->header;
    latest_ros_timestamp_ = image_msg->header.stamp;

    // Signal new frame available
    new_frame_available_ = true;
}

void whycon::WhyconRosInterface::onIceoryxImageReceived() {
    if (!detection_enabled_ || !camera_params_loaded_)
        return;

    // Create header for Iceoryx input
    latest_header_.stamp    = ros::Time::now();
    latest_header_.frame_id = parent_frame_id_;

    // Store timestamp
    latest_ros_timestamp_ = latest_header_.stamp;

    // Signal new frame available
    new_frame_available_ = true;
}

void whycon::WhyconRosInterface::processTimerCallback(const ros::TimerEvent& event) {
    // Check new frame
    if (!new_frame_available_)
        return;

    // Process immediately
    processLatestFrame();

    // Reset flag
    new_frame_available_ = false;
}

void whycon::WhyconRosInterface::processLatestFrame() {
    if (debug_manager_) {
        debug_manager_->clearDebugImages();
    }

    is_tracking_  = system_->localizeMarkers(*image_handler_, should_reset_, debug_manager_.get());
    should_reset_ = false;

    // Valid detections to avoid extra allocations
    valid_detections_buffer_.clear();
    for (auto& marker : system_->detector.outer_circles) {
        if (marker.valid) {
            valid_detections_buffer_.push_back(&marker);
        }
    }

    // Update  trackers and stabalizers
    tracker_->update(valid_detections_buffer_, *image_handler_, removed_ids_buffer_);
    id_stabilizer_->removeTracks(removed_ids_buffer_);
    pose_stabilizer_->removeTracks(removed_ids_buffer_);

    // Publish results
    publishResults(latest_header_);  //? Update this if pub image only when markers are detected

    // Publish consolidated debug images if enabled
    if (publish_debug_images_ && debug_manager_->isEnabled() && debug_images_pub_.getNumSubscribers() > 0) {
        cv::Mat consolidated_debug = debug_manager_->createConsolidatedImage();
        if (!consolidated_debug.empty()) {
            cv_bridge::CvImage cv_image(latest_header_, "rgb8", consolidated_debug);
            debug_images_pub_.publish(cv_image.toImageMsg());
        }
    }
}

bool whycon::WhyconRosInterface::detectionControlCallback(std_srvs::SetBool::Request&  req,
                                                          std_srvs::SetBool::Response& res) {
    detection_enabled_ = req.data;
    res.success        = true;
    res.message        = detection_enabled_ ? "Detection enabled." : "Detection disabled.";
    ROS_INFO_STREAM("[Whycon] Detection " << (detection_enabled_ ? "enabled" : "disabled") << " via service call.");
    return true;
}

void whycon::WhyconRosInterface::publishResults(const std_msgs::Header& header) {
    // Prepare image output
    if (publish_images_) {
        image_handler_->toOpenCVMat(output_image_buffer_);
    }

    int marker_count = 0;
    marker_array_buffer_.markers.clear();

    // Clear and reuse pose array buffer
    whycode_pose_array.poses.clear();
    whycode_pose_array.header          = header;
    whycode_pose_array.header.frame_id = parent_frame_id_;

    // go through detected targets
    for (int i = 0; i < targets_; i++) {
        if (!system_->isMarkerDetected(i))
            continue;

        const auto& inner_circle = system_->getMarkerByID(i);
        const auto& outer_circle = system_->getOuterMarkerByID(i);

        if (!(outer_circle.tracking_id != -1 && tracker_->getTrackAge(outer_circle.tracking_id) >= min_track_age_)) {
            // Skip markers that are not tracked or too young
            WHYCON_DEBUG("Skipping marker ID " << i << " - not tracked or too young.");
            continue;
        }

        system_->estimateMarkerPose(*image_handler_, outer_circle, system_->getMarkerByID(i), pose_buffer_,
                                    debug_manager_.get());

        // Stabilize WhyCode ID and pose
        pose_buffer_.ID       = id_stabilizer_->stabilize(pose_buffer_.tracking_id, pose_buffer_.ID);
        pose_buffer_.id_valid = (pose_buffer_.ID != -1);
        pose_buffer_.pos      = pose_stabilizer_->stabilize(pose_buffer_.tracking_id, pose_buffer_.pos);

        // draw each target
        if (publish_images_) {
            string_stream_buffer_.str("");
            string_stream_buffer_.clear();
            string_stream_buffer_ << std::fixed << std::setprecision(2);
            string_stream_buffer_ << "TID: " << pose_buffer_.tracking_id << ","
                                  << tracker_->getTrackAge(outer_circle.tracking_id).value_or(-1)
                                  << " WID: " << pose_buffer_.ID << '\n'
                                  << pose_buffer_.pos << " " << pose_buffer_.roll << " " << pose_buffer_.pitch << " "
                                  << pose_buffer_.yaw << '\n';

            outer_circle.draw(output_image_buffer_, string_stream_buffer_.str(), cv::Vec3b(0, 255, 0));
            inner_circle.draw(output_image_buffer_, " ", cv::Vec3b(255, 0, 255));
        }

        if (publish_poses_) {
            // Fill custom message
            whycode_pose_msg_buffer_.whycode_id         = pose_buffer_.ID;
            whycode_pose_msg_buffer_.tracking_id        = pose_buffer_.tracking_id;
            whycode_pose_msg_buffer_.id_valid           = pose_buffer_.id_valid;
            whycode_pose_msg_buffer_.pose.position.x    = pose_buffer_.pos(0);
            whycode_pose_msg_buffer_.pose.position.y    = pose_buffer_.pos(1);
            whycode_pose_msg_buffer_.pose.position.z    = pose_buffer_.pos(2);
            whycode_pose_msg_buffer_.pose.orientation.x = pose_buffer_.orientation.x;
            whycode_pose_msg_buffer_.pose.orientation.y = pose_buffer_.orientation.y;
            whycode_pose_msg_buffer_.pose.orientation.z = pose_buffer_.orientation.z;
            whycode_pose_msg_buffer_.pose.orientation.w = pose_buffer_.orientation.w;

            whycode_pose_array.poses.push_back(whycode_pose_msg_buffer_);
        }

        // Create Marker visualization
        if (publish_visualization_markers_) {
            createMarkerVisualization(pose_buffer_, marker_count, header, marker_buffer_);
            marker_array_buffer_.markers.push_back(marker_buffer_);
            ++marker_count;
        }

        // PUBLISH TF IMMEDIATELY using the same pose data
        if (publish_tf_) {
            publishSingleTF(pose_buffer_, pose_buffer_.ID);
        }
    }

    if (publish_images_ && image_pub_.getNumSubscribers() > 0) {
        cv_bridge::CvImage cv_image(header, "rgb8", output_image_buffer_);
        image_pub_.publish(cv_image.toImageMsg());
    }

    // Publish RViz markers (delete stale ones to avoid leftovers)
    if (publish_visualization_markers_ && visualization_markers_pub_.getNumSubscribers() > 0) {
        for (int i = marker_count; i < last_marker_count_; ++i) {
            visualization_msgs::Marker del;
            del.header = header;
            del.ns     = "whycon_markers";
            del.id     = i;
            del.action = visualization_msgs::Marker::DELETE;
            marker_array_buffer_.markers.push_back(std::move(del));
        }
        last_marker_count_ = marker_count;

        visualization_markers_pub_.publish(marker_array_buffer_);
    }

    if (publish_poses_) {
        whycode_pose_array.detected_marker_count = whycode_pose_array.poses.size();
        whycode_pose_pub_.publish(whycode_pose_array);
    }
}

void whycon::WhyconRosInterface::publishSingleTF(const whycon::LocalizationSystem::MarkerPose& pose, int track_id) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)));
    transform.setRotation(
            tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

    tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id_,
                                                        tf_frame_prefix_ + std::to_string(track_id)));
}

void whycon::WhyconRosInterface::createMarkerVisualization(const whycon::LocalizationSystem::MarkerPose& pose,
                                                           int marker_index, const std_msgs::Header& header,
                                                           visualization_msgs::Marker& marker) {
    // Basic marker properties
    marker.header = header;
    marker.ns     = "whycon_markers";
    marker.id     = pose.ID;
    marker.action = visualization_msgs::Marker::ADD;

    // Set marker type and scale
    marker.type    = visualization_msgs::Marker::CYLINDER;  // Use cylinder to represent circular markers
    marker.scale.x = parameters_.outer_diameter;            // Diameter
    marker.scale.y = parameters_.outer_diameter;            // Diameter
    marker.scale.z = 0.01;                                  // Height (thin disk)

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
    if (pose.id_valid && pose.ID != -1) {
        // Color based on ID (cycling through colors)
        float hue = (pose.ID * 60.0f) / 360.0f;  // Spread IDs across color wheel
        while (hue > 1.0f)
            hue -= 1.0f;

        // Convert HSV to RGB (simple approximation)
        if (hue < 1.0f / 6.0f) {
            marker.color.r = 1.0f;
            marker.color.g = hue * 6.0f;
            marker.color.b = 0.0f;
        } else if (hue < 2.0f / 6.0f) {
            marker.color.r = 1.0f - (hue - 1.0f / 6.0f) * 6.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        } else if (hue < 3.0f / 6.0f) {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = (hue - 2.0f / 6.0f) * 6.0f;
        } else if (hue < 4.0f / 6.0f) {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f - (hue - 3.0f / 6.0f) * 6.0f;
            marker.color.b = 1.0f;
        } else if (hue < 5.0f / 6.0f) {
            marker.color.r = (hue - 4.0f / 6.0f) * 6.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        } else {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f - (hue - 5.0f / 6.0f) * 6.0f;
        }
        marker.color.a = 0.8f;  // Semi-transparent
    } else {
        // Gray for unknown ID
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 0.6f;
    }

    // Set marker lifetime
    marker.lifetime = ros::Duration(0.1);  // Auto-delete after 100ms if not updated

    // Add text marker for ID display
    if (pose.id_valid && pose.ID != -1) {
        marker.text = std::to_string(pose.ID);
    } else {
        marker.text = "?";
    }
}