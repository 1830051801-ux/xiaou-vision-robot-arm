#include <chrono>
#include <cmath>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class TargetPlannerNode : public rclcpp::Node
{
public:
  TargetPlannerNode() : Node("xiaou_target_planner")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    allow_execution_ = declare_parameter<bool>("allow_execution", false);
    target_max_age_s_ = declare_parameter<double>("target_max_age_s", 0.5);
    velocity_scale_ = declare_parameter<double>("velocity_scale", 0.05);
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.05);
    planning_time_s_ = declare_parameter<double>("planning_time_s", 5.0);

    if (!std::isfinite(target_max_age_s_) || target_max_age_s_ <= 0.0) {
      throw std::invalid_argument("target_max_age_s must be finite and positive");
    }
    if (!std::isfinite(velocity_scale_) || velocity_scale_ <= 0.0 || velocity_scale_ > 1.0) {
      throw std::invalid_argument("velocity_scale must be in (0, 1]");
    }
    if (!std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 ||
      acceleration_scale_ > 1.0)
    {
      throw std::invalid_argument("acceleration_scale must be in (0, 1]");
    }
    if (!std::isfinite(planning_time_s_) || planning_time_s_ <= 0.0) {
      throw std::invalid_argument("planning_time_s must be finite and positive");
    }

    trajectory_publisher_ = create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "/xiaou/planned_trajectory", 10);
    hardware_ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/xiaou/hardware_ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) { hardware_ready_ = message->data; });
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xiaou/target_pose", 10,
      std::bind(&TargetPlannerNode::on_target, this, std::placeholders::_1));
  }

  void set_move_group(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  {
    move_group_ = move_group;
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("grasp_tcp");
  }

  const std::string & planning_group() const { return planning_group_; }

private:
  bool validate_trajectory(const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    static constexpr std::array<const char *, 6> expected_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    const auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.joint_names.size() != expected_names.size() ||
      joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Rejected trajectory with invalid joint names or no points");
      return false;
    }
    for (std::size_t i = 0; i < expected_names.size(); ++i) {
      if (joint_trajectory.joint_names[i] != expected_names[i]) {
        RCLCPP_ERROR(get_logger(), "Rejected trajectory with unexpected joint order");
        return false;
      }
    }
    int64_t previous_time_ns = -1;
    for (const auto & point : joint_trajectory.points) {
      const auto time_ns = rclcpp::Duration(point.time_from_start).nanoseconds();
      if (time_ns <= previous_time_ns || point.positions.size() != expected_names.size()) {
        RCLCPP_ERROR(get_logger(), "Rejected trajectory with non-monotonic time or invalid positions");
        return false;
      }
      previous_time_ns = time_ns;
      const auto finite = [](const auto & values) {
          for (const auto value : values) {
            if (!std::isfinite(value)) {
              return false;
            }
          }
          return true;
        };
      if (!finite(point.positions) ||
        (!point.velocities.empty() &&
        (point.velocities.size() != expected_names.size() || !finite(point.velocities))) ||
        (!point.accelerations.empty() &&
        (point.accelerations.size() != expected_names.size() || !finite(point.accelerations))))
      {
        RCLCPP_ERROR(get_logger(), "Rejected trajectory containing non-finite or incomplete values");
        return false;
      }
    }
    return true;
  }

  void on_target(const geometry_msgs::msg::PoseStamped::SharedPtr target)
  {
    if (!move_group_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not initialized");
      return;
    }
    if (target->header.frame_id != "base_link") {
      RCLCPP_ERROR(get_logger(), "Rejected target in frame '%s'; expected base_link",
        target->header.frame_id.c_str());
      return;
    }
    const auto stamp = rclcpp::Time(target->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      RCLCPP_WARN(get_logger(), "Rejected target without a valid timestamp");
      return;
    }
    const double age = (now() - stamp).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > target_max_age_s_) {
      RCLCPP_WARN(get_logger(), "Rejected stale target, age %.3f s", age);
      return;
    }

    const auto & position = target->pose.position;
    const auto & orientation = target->pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      RCLCPP_WARN(get_logger(), "Rejected target containing non-finite pose values");
      return;
    }
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (std::abs(quaternion_norm - 1.0) > 1e-3) {
      RCLCPP_WARN(get_logger(), "Rejected target with non-unit quaternion (norm %.6f)",
        quaternion_norm);
      return;
    }

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(*target, "grasp_tcp");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    move_group_->clearPoseTargets();
    if (!planned) {
      RCLCPP_WARN(get_logger(), "MoveIt could not find a collision-free IK trajectory");
      return;
    }
    if (!validate_trajectory(plan.trajectory)) {
      return;
    }

    trajectory_publisher_->publish(plan.trajectory);
    RCLCPP_INFO(get_logger(), "Published a planned trajectory; execution gate=%s, hardware=%s",
      allow_execution_ ? "requested" : "off", hardware_ready_ ? "ready" : "locked");

    if (!allow_execution_ || !hardware_ready_) {
      return;
    }
    const auto result = move_group_->execute(plan);
    if (!static_cast<bool>(result)) {
      RCLCPP_ERROR(get_logger(), "Trajectory execution failed");
    }
  }

  std::string planning_group_;
  bool allow_execution_{false};
  bool hardware_ready_{false};
  double target_max_age_s_{0.5};
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  double planning_time_s_{5.0};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_ready_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetPlannerNode>();
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, node->planning_group());
  node->set_move_group(move_group);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
