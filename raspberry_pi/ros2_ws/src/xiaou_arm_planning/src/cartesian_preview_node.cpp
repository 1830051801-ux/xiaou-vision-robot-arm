#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

class CartesianPreviewNode : public rclcpp::Node
{
public:
  CartesianPreviewNode() : Node("xiaou_cartesian_preview")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    eef_step_m_ = declare_parameter<double>("eef_step_m", 0.010);
    jump_threshold_ = declare_parameter<double>("jump_threshold", 0.0);
    min_fraction_ = declare_parameter<double>("min_fraction", 1.0);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 0.05);
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.05);
    avoid_collisions_ = declare_parameter<bool>("avoid_collisions", false);
    preview_start_joint_deg_ = declare_parameter<std::vector<double>>(
      "preview_start_joint_deg", std::vector<double>{0.0, -50.0, -55.0, -70.0, 110.0, 0.0});
    if (!std::isfinite(eef_step_m_) || eef_step_m_ < 0.001 || eef_step_m_ > 0.050) {
      throw std::invalid_argument("eef_step_m must be in [0.001, 0.050]");
    }
    if (!std::isfinite(jump_threshold_) || jump_threshold_ < 0.0) {
      throw std::invalid_argument("jump_threshold must be finite and non-negative");
    }
    if (!std::isfinite(min_fraction_) || min_fraction_ <= 0.0 || min_fraction_ > 1.0) {
      throw std::invalid_argument("min_fraction must be in (0, 1]");
    }
    if (!std::isfinite(velocity_scale_) || velocity_scale_ <= 0.0 || velocity_scale_ > 1.0 ||
      !std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 || acceleration_scale_ > 1.0)
    {
      throw std::invalid_argument("velocity and acceleration scales must be in (0, 1]");
    }
    if (preview_start_joint_deg_.size() != 6) {
      throw std::invalid_argument("preview_start_joint_deg must contain six values");
    }
    for (const double value : preview_start_joint_deg_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("preview_start_joint_deg must contain finite values");
      }
    }
    trajectory_publisher_ = create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "/xiaou/cartesian_preview_trajectory", rclcpp::QoS(1).reliable().transient_local());
    waypoint_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/xiaou/cartesian_waypoints", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&CartesianPreviewNode::on_waypoints, this, std::placeholders::_1));
    preview_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1).reliable().transient_local());
  }

  void set_move_group(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  {
    move_group_ = move_group;
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("grasp_tcp");
  }

  const std::string & planning_group() const { return planning_group_; }

  void publish_preview_start_state()
  {
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    state.position.reserve(preview_start_joint_deg_.size());
    for (const double degrees : preview_start_joint_deg_) {
      state.position.push_back(degrees * kPi / 180.0);
    }
    preview_joint_state_publisher_->publish(state);
  }

  moveit::core::RobotStatePtr make_preview_start_state() const
  {
    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      return nullptr;
    }
    auto state = std::make_shared<moveit::core::RobotState>(robot_model);
    const std::vector<std::string> names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    std::vector<double> positions;
    positions.reserve(preview_start_joint_deg_.size());
    for (const double degrees : preview_start_joint_deg_) {
      positions.push_back(degrees * kPi / 180.0);
    }
    state->setToDefaultValues();
    state->setVariablePositions(names, positions);
    state->update();
    return state;
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  static bool finite_pose(const geometry_msgs::msg::Pose & pose)
  {
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      return false;
    }
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    return std::abs(norm - 1.0) <= 1e-3;
  }

  bool validate_trajectory(const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    static constexpr std::array<const char *, 6> expected_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    const auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.joint_names.size() != expected_names.size() || joint_trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Rejected Cartesian result with invalid joint names or no points");
      return false;
    }
    for (std::size_t index = 0; index < expected_names.size(); ++index) {
      if (joint_trajectory.joint_names[index] != expected_names[index]) {
        RCLCPP_ERROR(get_logger(), "Rejected Cartesian result with unexpected joint order");
        return false;
      }
    }
    int64_t previous_time_ns = -1;
    for (const auto & point : joint_trajectory.points) {
      const auto time_ns = rclcpp::Duration(point.time_from_start).nanoseconds();
      if (time_ns <= previous_time_ns || point.positions.size() != expected_names.size()) {
        RCLCPP_ERROR(get_logger(), "Rejected Cartesian result with invalid timing or positions");
        return false;
      }
      previous_time_ns = time_ns;
      for (const double value : point.positions) {
        if (!std::isfinite(value)) {
          RCLCPP_ERROR(get_logger(), "Rejected Cartesian result with non-finite joint position");
          return false;
        }
      }
    }
    return true;
  }

  void on_waypoints(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    if (!move_group_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not initialized");
      return;
    }
    if (message->header.frame_id != "base_link" || message->poses.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Rejected waypoint batch: expected base_link and at least two poses");
      return;
    }
    publish_preview_start_state();
    for (const auto & pose : message->poses) {
      if (!finite_pose(pose)) {
        RCLCPP_ERROR(get_logger(), "Rejected waypoint batch with non-finite or non-unit pose");
        return;
      }
    }
    const auto preview_state = make_preview_start_state();
    if (!preview_state) {
      RCLCPP_ERROR(get_logger(), "MoveIt has no robot model for Cartesian preview");
      return;
    }
    move_group_->setStartState(*preview_state);
    moveit_msgs::msg::RobotTrajectory raw_trajectory;
    const double fraction = move_group_->computeCartesianPath(
      message->poses, eef_step_m_, jump_threshold_, raw_trajectory, avoid_collisions_);
    if (!std::isfinite(fraction) || fraction + 1e-9 < min_fraction_) {
      RCLCPP_WARN(
        get_logger(), "Cartesian preview incomplete: fraction %.6f, required %.6f", fraction, min_fraction_);
      return;
    }
    const auto timed_trajectory = move_group_->retimeTrajectory(
      preview_state, raw_trajectory, velocity_scale_, acceleration_scale_);
    if (!validate_trajectory(timed_trajectory)) {
      return;
    }
    trajectory_publisher_->publish(timed_trajectory);
    RCLCPP_INFO(
      get_logger(), "Published %zu-pose Cartesian preview, fraction %.6f; hardware execution is disabled.",
      message->poses.size(), fraction);
  }

  std::string planning_group_;
  double eef_step_m_{0.010};
  double jump_threshold_{0.0};
  double min_fraction_{1.0};
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  bool avoid_collisions_{false};
  std::vector<double> preview_start_joint_deg_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr preview_joint_state_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPreviewNode>();
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, node->planning_group());
  node->set_move_group(move_group);
  node->publish_preview_start_state();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
