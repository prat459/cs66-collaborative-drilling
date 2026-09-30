#include <memory>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "drilling_approach",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Physical execution is disabled by default.
  bool execute_motion = false;
  node->get_parameter_or("execute", execute_motion, false);

  // Spin the node in the background so MoveIt can receive
  // /joint_states and other ROS messages.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // Planning group used by the Elite CS66 MoveIt configuration.
  moveit::planning_interface::MoveGroupInterface move_group(node, "cs_manipulator");

  // Start monitoring the physical robot state and give ROS time
  // to receive the first /joint_states messages.
  move_group.startStateMonitor(5.0);

  auto current_state = move_group.getCurrentState(5.0);

  if (!current_state)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not obtain the current physical robot state. Aborting.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Current physical robot state received successfully.");

  // Read the current physical TCP pose.
  geometry_msgs::msg::Pose current_pose =
    move_group.getCurrentPose().pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current TCP: x=%.6f y=%.6f z=%.6f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);

  // Quaternion representing the current tool orientation.
  const double qx = current_pose.orientation.x;
  const double qy = current_pose.orientation.y;
  const double qz = current_pose.orientation.z;
  const double qw = current_pose.orientation.w;

  // Tool-local +Z axis expressed in the base frame.
  const double tool_z_x = 2.0 * (qx * qz + qw * qy);
  const double tool_z_y = 2.0 * (qy * qz - qw * qx);
  const double tool_z_z = 1.0 - 2.0 * (qx * qx + qy * qy);

  RCLCPP_INFO(
    node->get_logger(),
    "Tool +Z direction: [%.6f, %.6f, %.6f]",
    tool_z_x, tool_z_y, tool_z_z);

  // Exactly 10 mm along the current tool +Z direction.
  constexpr double approach_distance = 0.010;

  geometry_msgs::msg::Pose target_pose = current_pose;

  target_pose.position.x += approach_distance * tool_z_x;
  target_pose.position.y += approach_distance * tool_z_y;
  target_pose.position.z += approach_distance * tool_z_z;

  // Orientation is unchanged.
  //
  // For drilling we want a straight Cartesian displacement rather than
  // allowing a general pose planner to choose a distant equivalent
  // joint configuration.

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

  constexpr double eef_step = 0.001;       // 1 mm
  constexpr double jump_threshold = 0.0;  // reject large joint-space jumps

  const double fraction = move_group.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    cartesian_trajectory,
    true);

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian path completion: %.1f%%",
    fraction * 100.0);

  const bool success = fraction >= 0.999;

  if (success)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "CARTESIAN PLAN SUCCESSFUL -- physical robot was NOT commanded to execute.");

    const auto & trajectory = cartesian_trajectory.joint_trajectory;

    RCLCPP_INFO(
      node->get_logger(),
      "Trajectory contains %zu points.",
      trajectory.points.size());

    if (!trajectory.points.empty())
    {
      const auto & start_point = trajectory.points.front();
      const auto & finish_point = trajectory.points.back();

      RCLCPP_INFO(node->get_logger(), "Planned joint movement:");

      for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
      {
        const double q_start = start_point.positions[i];
        const double q_finish = finish_point.positions[i];
        const double delta = q_finish - q_start;

        RCLCPP_INFO(
          node->get_logger(),
          "  %s: %.6f -> %.6f   delta=%+.6f rad (%.3f deg)",
          trajectory.joint_names[i].c_str(),
          q_start,
          q_finish,
          delta,
          delta * 180.0 / M_PI);
      }
    }
    if (execute_motion)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "EXECUTION ENABLED -- sending validated Cartesian trajectory to the robot.");

      moveit::planning_interface::MoveGroupInterface::Plan execution_plan;
      execution_plan.trajectory_ = cartesian_trajectory;

      const auto result = move_group.execute(execution_plan);

      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Physical Cartesian motion completed successfully.");
      }
      else
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Physical trajectory execution failed.");
      }
    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(),
        "PLAN ONLY. Use execute:=true only when ready for physical motion.");
    }
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cartesian planning incomplete (%.1f%%). Nothing will be executed.",
      fraction * 100.0);
  }

  move_group.clearPoseTargets();

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return success ? 0 : 1;
}
