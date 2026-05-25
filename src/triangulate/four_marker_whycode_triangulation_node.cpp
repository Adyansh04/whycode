#include <rclcpp/rclcpp.hpp>
#include <whycon_whycode_localization/msg/why_code_pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class FourMarkerWhyCodeTriangulationNode : public rclcpp::Node {
public:
  FourMarkerWhyCodeTriangulationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("four_marker_whycode_triangulation_node", options) {
    // Parameters
    this->declare_parameter<int>("marker_id_top_left", 3);
    this->declare_parameter<int>("marker_id_bottom_left", 1);
    this->declare_parameter<int>("marker_id_top_right", 4);
    this->declare_parameter<int>("marker_id_bottom_right", 2);
    this->declare_parameter<double>("known_distance_vertical", 0.3);
    this->declare_parameter<double>("known_distance_horizontal", 0.5);
    this->declare_parameter<double>("marker_timeout", 1.0);
    this->declare_parameter<bool>("publish_pose", true);
    this->declare_parameter<bool>("publish_tf", true);

    config_.marker_id_top_left = this->get_parameter("marker_id_top_left").as_int();
    config_.marker_id_bottom_left = this->get_parameter("marker_id_bottom_left").as_int();
    config_.marker_id_top_right = this->get_parameter("marker_id_top_right").as_int();
    config_.marker_id_bottom_right = this->get_parameter("marker_id_bottom_right").as_int();
    config_.known_distance_vertical = this->get_parameter("known_distance_vertical").as_double();
    config_.known_distance_horizontal = this->get_parameter("known_distance_horizontal").as_double();
    config_.marker_timeout = this->get_parameter("marker_timeout").as_double();
    config_.publish_pose = this->get_parameter("publish_pose").as_bool();
    config_.publish_tf = this->get_parameter("publish_tf").as_bool();

    if (config_.publish_pose) {
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("triangulation/pose", 10);
    }

    if (config_.publish_tf) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    poses_sub_ = this->create_subscription<whycon_whycode_localization::msg::WhyCodePoseArray>(
        "whycon/poses", rclcpp::QoS(10),
        std::bind(&FourMarkerWhyCodeTriangulationNode::posesCallback, this, std::placeholders::_1));

    processing_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / PROCESSING_RATE),
                                                std::bind(&FourMarkerWhyCodeTriangulationNode::processingTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "FourMarkerWhyCodeTriangulationNode initialized");
  }

private:
  struct MarkerData {
    geometry_msgs::msg::Pose pose;
    int whycode_id = -1;
    int tracking_id = -1;
    bool id_valid = false;
    bool found = false;
    std::chrono::steady_clock::time_point last_seen;
    void reset() { found = false; id_valid = false; }
    Eigen::Vector3d getPosition() const { return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z); }
  } top_left_, bottom_left_, top_right_, bottom_right_;

  struct TriangulationConfig {
    int marker_id_top_left = 3;
    int marker_id_bottom_left = 1;
    int marker_id_top_right = 4;
    int marker_id_bottom_right = 2;
    double known_distance_vertical = 0.3;
    double known_distance_horizontal = 0.5;
    double marker_timeout = 1.0;
    bool publish_pose = true;
    bool publish_tf = true;
  } config_;

  rclcpp::Subscription<whycon_whycode_localization::msg::WhyCodePoseArray>::SharedPtr poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  static constexpr double PROCESSING_RATE = 30.0;  // Hz

  void posesCallback(const whycon_whycode_localization::msg::WhyCodePoseArray::SharedPtr msg) {
    top_left_.reset(); bottom_left_.reset(); top_right_.reset(); bottom_right_.reset();
    auto now = std::chrono::steady_clock::now();
    for (const auto &m : msg->poses) {
      if (m.whycode_id == config_.marker_id_top_left) { top_left_.whycode_id = m.whycode_id; top_left_.pose = m.pose; top_left_.id_valid = m.id_valid; top_left_.found = true; top_left_.last_seen = now; }
      else if (m.whycode_id == config_.marker_id_bottom_left) { bottom_left_.whycode_id = m.whycode_id; bottom_left_.pose = m.pose; bottom_left_.id_valid = m.id_valid; bottom_left_.found = true; bottom_left_.last_seen = now; }
      else if (m.whycode_id == config_.marker_id_top_right) { top_right_.whycode_id = m.whycode_id; top_right_.pose = m.pose; top_right_.id_valid = m.id_valid; top_right_.found = true; top_right_.last_seen = now; }
      else if (m.whycode_id == config_.marker_id_bottom_right) { bottom_right_.whycode_id = m.whycode_id; bottom_right_.pose = m.pose; bottom_right_.id_valid = m.id_valid; bottom_right_.found = true; bottom_right_.last_seen = now; }
    }
  }

  void processingTimerCallback() {
    if (!areAllMarkersValid()) return;
    auto left = estimatePose(top_left_.getPosition(), bottom_left_.getPosition(), config_.known_distance_vertical);
    if (!left.is_valid) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Left estimation failed: %s", left.error_message.c_str()); return; }
    auto right = estimatePose(top_right_.getPosition(), bottom_right_.getPosition(), config_.known_distance_vertical);
    if (!right.is_valid) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Right estimation failed: %s", right.error_message.c_str()); return; }
    auto final_res = estimatePose(left.position, right.position, config_.known_distance_horizontal);
    if (!final_res.is_valid) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Final estimation failed: %s", final_res.error_message.c_str()); return; }
    if (config_.publish_tf && tf_broadcaster_) publishTransform(final_res, "marker_center_final");
    if (config_.publish_pose && pose_pub_) publishPose(final_res);
  }

  bool areAllMarkersValid() const {
    auto now = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(config_.marker_timeout);
    std::vector<MarkerData> markers = {top_left_, bottom_left_, top_right_, bottom_right_};
    for (const auto &m : markers) {
      if (!m.found || !m.id_valid) return false;
      if (now - m.last_seen > timeout) return false;
    }
    return true;
  }

  struct PoseEstimationResult { Eigen::Vector3d position; Eigen::Matrix3d rotation; bool is_valid=false; std::string error_message; };

  PoseEstimationResult estimatePose(const Eigen::Vector3d &P1, const Eigen::Vector3d &P2, double known_distance) const {
    PoseEstimationResult result; result.is_valid = false;
    if ((P2-P1).norm() < 1e-8) { result.error_message = "Markers too close"; return result; }
    Eigen::Vector3d marker_vector = P2 - P1; Eigen::Vector3d midpoint_rough = 0.5*(P1+P2);
    Eigen::Vector3d x_axis = marker_vector.normalized(); Eigen::Vector3d z_axis = P1.cross(P2);
    if (z_axis.norm() < 1e-8) { result.error_message = "Collinear markers"; return result; }
    z_axis.normalize(); Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    Eigen::Vector3d corrected_P1 = midpoint_rough - 0.5*known_distance*x_axis; Eigen::Vector3d corrected_P2 = midpoint_rough + 0.5*known_distance*x_axis;
    result.position = 0.5*(corrected_P1+corrected_P2); Eigen::Matrix3d rotation; rotation.col(0)=x_axis; rotation.col(1)=y_axis; rotation.col(2)=z_axis; result.rotation=rotation; result.is_valid=true; return result;
  }

  void publishTransform(const PoseEstimationResult &res, const std::string &frame_name) {
    geometry_msgs::msg::TransformStamped ts; ts.header.stamp = this->now(); ts.header.frame_id = "camera_link"; ts.child_frame_id = frame_name;
    ts.transform.translation.x = res.position.x(); ts.transform.translation.y = res.position.y(); ts.transform.translation.z = res.position.z();
    Eigen::Quaterniond q(res.rotation); ts.transform.rotation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())); tf_broadcaster_->sendTransform(ts);
  }

  void publishPose(const PoseEstimationResult &res) {
    geometry_msgs::msg::PoseStamped ps; ps.header.stamp = this->now(); ps.header.frame_id = "camera_link"; ps.pose = eigenToRosPose(res.position, res.rotation); pose_pub_->publish(ps);
  }

  geometry_msgs::msg::Pose eigenToRosPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation) const {
    geometry_msgs::msg::Pose p; p.position.x = position.x(); p.position.y = position.y(); p.position.z = position.z(); Eigen::Quaterniond q(rotation); p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w(); return p;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FourMarkerWhyCodeTriangulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
