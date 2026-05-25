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

using namespace std::chrono_literals;

class TwoMarkerWhyCodeTriangulationNode : public rclcpp::Node {
public:
  TwoMarkerWhyCodeTriangulationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("two_marker_whycode_triangulation_node", options) {
    // Parameters
    this->declare_parameter<int>("marker_id_1", 1);
    this->declare_parameter<int>("marker_id_2", 2);
    this->declare_parameter<double>("known_distance", 0.5);
    this->declare_parameter<double>("marker_timeout", 5.0);
    this->declare_parameter<bool>("publish_pose", true);
    this->declare_parameter<bool>("publish_tf", true);

    params_.marker_id_1 = this->get_parameter("marker_id_1").as_int();
    params_.marker_id_2 = this->get_parameter("marker_id_2").as_int();
    params_.known_distance = this->get_parameter("known_distance").as_double();
    params_.marker_timeout = this->get_parameter("marker_timeout").as_double();
    params_.publish_pose = this->get_parameter("publish_pose").as_bool();
    params_.publish_tf = this->get_parameter("publish_tf").as_bool();

    if (params_.publish_pose) {
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("two_marker_triangulation_whycode/pose", 10);
    }

    if (params_.publish_tf) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    poses_sub_ = this->create_subscription<whycon_whycode_localization::msg::WhyCodePoseArray>(
        "whycon/poses", rclcpp::QoS(10),
        std::bind(&TwoMarkerWhyCodeTriangulationNode::posesCallback, this, std::placeholders::_1));

    processing_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / PROCESSING_RATE),
                                                std::bind(&TwoMarkerWhyCodeTriangulationNode::processingTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "TwoMarkerWhyCodeTriangulationNode initialized (IDs %d,%d)", params_.marker_id_1,
                params_.marker_id_2);
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
  } marker1_, marker2_;

  struct TriangulationConfig {
    int marker_id_1 = 1;
    int marker_id_2 = 2;
    double known_distance = 0.5;
    double marker_timeout = 5.0;
    bool publish_pose = true;
    bool publish_tf = true;
  } params_;

  rclcpp::Subscription<whycon_whycode_localization::msg::WhyCodePoseArray>::SharedPtr poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  static constexpr double PROCESSING_RATE = 30.0;  // Hz

  void posesCallback(const whycon_whycode_localization::msg::WhyCodePoseArray::SharedPtr msg) {
    marker1_.reset();
    marker2_.reset();
    auto now = std::chrono::steady_clock::now();

    for (const auto &m : msg->poses) {
      if (m.whycode_id == params_.marker_id_1) {
        marker1_.whycode_id = m.whycode_id;
        marker1_.tracking_id = m.tracking_id;
        marker1_.id_valid = m.id_valid;
        marker1_.pose = m.pose;
        marker1_.found = true;
        marker1_.last_seen = now;
      } else if (m.whycode_id == params_.marker_id_2) {
        marker2_.whycode_id = m.whycode_id;
        marker2_.tracking_id = m.tracking_id;
        marker2_.id_valid = m.id_valid;
        marker2_.pose = m.pose;
        marker2_.found = true;
        marker2_.last_seen = now;
      }
    }
  }

  void processingTimerCallback() {
    if (!areTwoMarkersValid()) return;
    Eigen::Vector3d P1 = marker1_.getPosition();
    Eigen::Vector3d P2 = marker2_.getPosition();
    auto result = estimatePose(P1, P2, params_.known_distance);
    if (result.is_valid) {
      if (params_.publish_tf && tf_broadcaster_) {
        publishTransform(result);
      }
      if (params_.publish_pose && pose_pub_) {
        publishPose(result);
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Pose estimation failed: %s",
                           result.error_message.c_str());
    }
  }

  bool areTwoMarkersValid() const {
    auto now = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(params_.marker_timeout);
    if (!marker1_.found || !marker2_.found || !marker1_.id_valid || !marker2_.id_valid) return false;
    if (now - marker1_.last_seen > timeout || now - marker2_.last_seen > timeout) return false;
    return true;
  }

  struct PoseEstimationResult {
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
    bool is_valid = false;
    std::string error_message;
  };

  PoseEstimationResult estimatePose(const Eigen::Vector3d &P1, const Eigen::Vector3d &P2, double known_distance) const {
    PoseEstimationResult result;
    result.is_valid = false;
    if ((P2 - P1).norm() < 1e-8) { result.error_message = "Markers too close"; return result; }
    Eigen::Vector3d marker_vector = P2 - P1;
    Eigen::Vector3d midpoint_rough = 0.5 * (P1 + P2);
    Eigen::Vector3d x_axis = marker_vector.normalized();
    Eigen::Vector3d z_axis = P1.cross(P2);
    if (z_axis.norm() < 1e-8) { result.error_message = "Collinear markers"; return result; }
    z_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    Eigen::Vector3d corrected_P1 = midpoint_rough - 0.5 * known_distance * x_axis;
    Eigen::Vector3d corrected_P2 = midpoint_rough + 0.5 * known_distance * x_axis;
    Eigen::Vector3d corrected_center = 0.5 * (corrected_P1 + corrected_P2);
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis; rotation.col(1) = y_axis; rotation.col(2) = z_axis;
    result.position = corrected_center;
    result.rotation = rotation;
    result.is_valid = true;
    return result;
  }

  void publishTransform(const PoseEstimationResult &res) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->now();
    ts.header.frame_id = "camera_link";
    ts.child_frame_id = "two_marker_triangulation_whycode";
    ts.transform.translation.x = res.position.x();
    ts.transform.translation.y = res.position.y();
    ts.transform.translation.z = res.position.z();
    Eigen::Quaterniond q(res.rotation);
    ts.transform.rotation = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster_->sendTransform(ts);
  }

  void publishPose(const PoseEstimationResult &res) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();
    ps.header.frame_id = "camera_link";
    ps.pose = eigenToRosPose(res.position, res.rotation);
    pose_pub_->publish(ps);
  }

  geometry_msgs::msg::Pose eigenToRosPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation) const {
    geometry_msgs::msg::Pose p;
    p.position.x = position.x(); p.position.y = position.y(); p.position.z = position.z();
    Eigen::Quaterniond q(rotation);
    p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
    return p;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwoMarkerWhyCodeTriangulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
