#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.82116;
    msg.orientation.x = 0.18106;
    msg.orientation.x = -0.11654;
    msg.orientation.w = 0.52851;
    msg.position.x = 0.037535;
    msg.position.y = 0.080977;
    msg.position.z = 0.57325;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setGoalTolerance(0.5);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}