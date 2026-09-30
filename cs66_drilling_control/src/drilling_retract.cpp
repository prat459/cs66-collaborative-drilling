#include <memory>
#include <cmath>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "drilling_retract",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute_motion = false;
  node->get_parameter_or("execute", execute_motion, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface move_group(
    node, "cs_manipulator");

  move_group.startStateMonitor(5.0);

  auto state = move_group.getCurrentState(5.0);

  if (!state)
  {
    RCLCPP_ERROR(node->get_logger(),
      "Cannot get robot state");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  geometry_msgs::msg::Pose current_pose =
    move_group.getCurrentPose().pose;

  RCLCPP_INFO(node->get_logger(),
    "Current TCP: x=%.6f y=%.6f z=%.6f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);


  // Current tool +Z direction
  double qx = current_pose.orientation.x;
  double qy = current_pose.orientation.y;
  double qz = current_pose.orientation.z;
  double qw = current_pose.orientation.w;

  double tool_z_x = 2.0 * (qx*qz + qw*qy);
  double tool_z_y = 2.0 * (qy*qz - qw*qx);
  double tool_z_z = 1.0 - 2.0*(qx*qx + qy*qy);


  // Retract opposite direction (+10 mm away)
  constexpr double distance = 0.010;

  geometry_msgs::msg::Pose target_pose = current_pose;

  target_pose.position.x -= distance * tool_z_x;
  target_pose.position.y -= distance * tool_z_y;
  target_pose.position.z -= distance * tool_z_z;


  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);


  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group.computeCartesianPath(
    waypoints,
    0.001,
    0.0,
    trajectory,
    true);


  RCLCPP_INFO(node->get_logger(),
    "Retract Cartesian completion: %.1f%%",
    fraction*100.0);


  if (fraction >= 0.999)
  {
    RCLCPP_INFO(node->get_logger(),
      "Retract plan successful.");

    if (execute_motion)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      auto result = move_group.execute(plan);

      if(result == moveit::core::MoveItErrorCode::SUCCESS)
        RCLCPP_INFO(node->get_logger(),
          "Retract executed successfully.");
      else
        RCLCPP_ERROR(node->get_logger(),
          "Retract execution failed.");
    }
    else
    {
      RCLCPP_INFO(node->get_logger(),
        "PLAN ONLY. Use execute:=true to move.");
    }
  }

  rclcpp::shutdown();

  if(spinner.joinable())
    spinner.join();

  return 0;
}
