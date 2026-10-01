#include <memory>
#include <cmath>
#include <thread>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "drilling_cycle_v2",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  // =====================================================
  // User-configurable experiment parameters
  // =====================================================

  bool execute_motion = false;

  double stroke = 0.025;          // meters
  double down_velocity = 0.010;   // m/s
  double up_velocity = 0.020;     // m/s
  int cycles = 5;

  double bottom_dwell = 0.0;      // seconds
  double top_dwell = 0.0;         // seconds

  node->get_parameter_or("execute", execute_motion, false);
  node->get_parameter_or("stroke", stroke, 0.025);
  node->get_parameter_or("down_velocity", down_velocity, 0.010);
  node->get_parameter_or("up_velocity", up_velocity, 0.020);
  node->get_parameter_or("cycles", cycles, 5);
  node->get_parameter_or("bottom_dwell", bottom_dwell, 0.0);
  node->get_parameter_or("top_dwell", top_dwell, 0.0);

  // =====================================================
  // Validate parameters
  // =====================================================

  if (stroke <= 0.0 ||
      down_velocity <= 0.0 ||
      up_velocity <= 0.0 ||
      cycles < 1 ||
      bottom_dwell < 0.0 ||
      top_dwell < 0.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid drilling parameters.");

    rclcpp::shutdown();
    return 1;
  }

  // =====================================================
  // Commanded timing
  // =====================================================

  const double down_time =
    stroke / down_velocity;

  const double up_time =
    stroke / up_velocity;

  const double commanded_cycle_time =
    down_time +
    bottom_dwell +
    up_time +
    top_dwell;

  const double commanded_total_time =
    commanded_cycle_time *
    static_cast<double>(cycles);

  // =====================================================
  // ROS executor
  // =====================================================

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // =====================================================
  // MoveIt
  // =====================================================

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "cs_manipulator");

  move_group.startStateMonitor(5.0);

  auto display_pub =
    node->create_publisher<
      moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path",
        10);

  std::this_thread::sleep_for(
    std::chrono::seconds(1));

  // =====================================================
  // Current robot pose = UP
  // =====================================================

  auto current_state =
    move_group.getCurrentState(5.0);

  if (!current_state)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cannot obtain current robot state.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 1;
  }

  geometry_msgs::msg::Pose up_pose =
    move_group.getCurrentPose().pose;

  // =====================================================
  // Tool +Z direction
  // =====================================================

  const double qx = up_pose.orientation.x;
  const double qy = up_pose.orientation.y;
  const double qz = up_pose.orientation.z;
  const double qw = up_pose.orientation.w;

  const double tool_z_x =
    2.0 * (qx * qz + qw * qy);

  const double tool_z_y =
    2.0 * (qy * qz - qw * qx);

  const double tool_z_z =
    1.0 - 2.0 * (qx * qx + qy * qy);

  // =====================================================
  // DOWN pose
  // =====================================================

  geometry_msgs::msg::Pose down_pose = up_pose;

  down_pose.position.x +=
    stroke * tool_z_x;

  down_pose.position.y +=
    stroke * tool_z_y;

  down_pose.position.z +=
    stroke * tool_z_z;

  // =====================================================
  // Print experiment configuration
  // =====================================================

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "CS66 DRILLING CYCLE V2");

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "UP:   x=%.6f y=%.6f z=%.6f",
    up_pose.position.x,
    up_pose.position.y,
    up_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "DOWN: x=%.6f y=%.6f z=%.6f",
    down_pose.position.x,
    down_pose.position.y,
    down_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Tool +Z: [%.6f %.6f %.6f]",
    tool_z_x,
    tool_z_y,
    tool_z_z);

  RCLCPP_INFO(
    node->get_logger(),
    "----------------------------------------");

  RCLCPP_INFO(
    node->get_logger(),
    "COMMAND PARAMETERS");

  RCLCPP_INFO(
    node->get_logger(),
    "Stroke: %.2f mm",
    stroke * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Down velocity: %.2f mm/s",
    down_velocity * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Up velocity: %.2f mm/s",
    up_velocity * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Cycles: %d",
    cycles);

  RCLCPP_INFO(
    node->get_logger(),
    "Bottom dwell: %.2f sec",
    bottom_dwell);

  RCLCPP_INFO(
    node->get_logger(),
    "Top dwell: %.2f sec",
    top_dwell);

  RCLCPP_INFO(
    node->get_logger(),
    "----------------------------------------");

  RCLCPP_INFO(
    node->get_logger(),
    "Down time: %.3f sec",
    down_time);

  RCLCPP_INFO(
    node->get_logger(),
    "Up time: %.3f sec",
    up_time);

  RCLCPP_INFO(
    node->get_logger(),
    "Commanded cycle time: %.3f sec",
    commanded_cycle_time);

  RCLCPP_INFO(
    node->get_logger(),
    "Commanded total time: %.3f sec",
    commanded_total_time);

  // =====================================================
  // Generate ONE complete DOWN -> UP Cartesian trajectory
  // =====================================================

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(down_pose);
  waypoints.push_back(up_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      0.0005,
      0.0,
      trajectory,
      true);

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian completion: %.1f%%",
    fraction * 100.0);

  if (fraction < 0.999)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Incomplete Cartesian path. Nothing executed.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 1;
  }

  // =====================================================
  // Explicit timing for complete trajectory
  //
  // UP -> DOWN       : down_time
  // bottom dwell     : bottom_dwell
  // DOWN -> UP       : up_time
  //
  // One trajectory, one execute() per cycle.
  // =====================================================

  auto & points =
    trajectory.joint_trajectory.points;

  if (points.size() < 3)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Trajectory contains too few points.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 1;
  }

  const std::size_t last_index =
    points.size() - 1;

  const std::size_t middle_index =
    last_index / 2;

  const double up_start_time =
    down_time + bottom_dwell;

  for (std::size_t i = 0;
       i < points.size();
       ++i)
  {
    double t = 0.0;

    if (i <= middle_index)
    {
      if (middle_index == 0)
      {
        t = 0.0;
      }
      else
      {
        t =
          down_time *
          static_cast<double>(i) /
          static_cast<double>(middle_index);
      }
    }
    else
    {
      const std::size_t second_half_points =
        last_index - middle_index;

      t =
        up_start_time +
        up_time *
        static_cast<double>(
          i - middle_index) /
        static_cast<double>(
          second_half_points);
    }

    const int32_t sec =
      static_cast<int32_t>(
        std::floor(t));

    const uint32_t nanosec =
      static_cast<uint32_t>(
        (t - static_cast<double>(sec)) *
        1e9);

    points[i].time_from_start.sec =
      sec;

    points[i].time_from_start.nanosec =
      nanosec;

    points[i].velocities.clear();
    points[i].accelerations.clear();
    points[i].effort.clear();
  }

  // =====================================================
  // Display complete trajectory in RViz
  // =====================================================

  moveit_msgs::msg::DisplayTrajectory display_msg;

  display_msg.model_id =
    move_group.getRobotModel()->getName();

  moveit::core::robotStateToRobotStateMsg(
    *current_state,
    display_msg.trajectory_start);

  display_msg.trajectory.push_back(
    trajectory);

  RCLCPP_INFO(
    node->get_logger(),
    "Publishing complete DOWN -> UP trajectory to RViz...");

  for (int i = 0; i < 10; ++i)
  {
    display_pub->publish(display_msg);

    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));
  }

  // =====================================================
  // PLAN ONLY
  // =====================================================

  if (!execute_motion)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "========================================");

    RCLCPP_INFO(
      node->get_logger(),
      "PLAN ONLY.");

    RCLCPP_INFO(
      node->get_logger(),
      "Physical robot did NOT move.");

    RCLCPP_INFO(
      node->get_logger(),
      "========================================");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 0;
  }

  // =====================================================
  // Create ONE MoveIt plan
  // =====================================================

  moveit::planning_interface::
    MoveGroupInterface::Plan cycle_plan;

  cycle_plan.trajectory_ =
    trajectory;

  // =====================================================
  // PHYSICAL EXECUTION
  // =====================================================

  RCLCPP_WARN(
    node->get_logger(),
    "========================================");

  RCLCPP_WARN(
    node->get_logger(),
    "PHYSICAL EXECUTION ENABLED");

  RCLCPP_WARN(
    node->get_logger(),
    "ONE trajectory execution per cycle");

  RCLCPP_WARN(
    node->get_logger(),
    "Requested cycles: %d",
    cycles);

  RCLCPP_WARN(
    node->get_logger(),
    "========================================");

  const auto experiment_start =
    std::chrono::steady_clock::now();

  int completed_cycles = 0;

  for (int cycle = 1;
       cycle <= cycles && rclcpp::ok();
       ++cycle)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "----------------------------------------");

    RCLCPP_INFO(
      node->get_logger(),
      "Starting cycle %d / %d",
      cycle,
      cycles);

    RCLCPP_INFO(
      node->get_logger(),
      "Executing complete DOWN -> DWELL -> UP trajectory");

    const auto cycle_start =
      std::chrono::steady_clock::now();

    const auto result =
      move_group.execute(cycle_plan);

    const auto cycle_end =
      std::chrono::steady_clock::now();

    const double actual_cycle_time =
      std::chrono::duration<double>(
        cycle_end - cycle_start).count();

    if (result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Cycle %d execution failed.",
        cycle);

      break;
    }

    completed_cycles++;

    RCLCPP_INFO(
      node->get_logger(),
      "Cycle %d complete. Commanded: %.3f s | Actual: %.3f s",
      cycle,
      down_time + bottom_dwell + up_time,
      actual_cycle_time);

    // Top dwell happens after the complete trajectory.
    if (top_dwell > 0.0)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "State: TOP_DWELL (%.3f s)",
        top_dwell);

      std::this_thread::sleep_for(
        std::chrono::duration<double>(
          top_dwell));
    }
  }

  // =====================================================
  // Experiment summary
  // =====================================================

  const auto experiment_end =
    std::chrono::steady_clock::now();

  const double actual_total_time =
    std::chrono::duration<double>(
      experiment_end -
      experiment_start).count();

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "DRILLING V2 TEST COMPLETE");

  RCLCPP_INFO(
    node->get_logger(),
    "Completed cycles: %d / %d",
    completed_cycles,
    cycles);

  RCLCPP_INFO(
    node->get_logger(),
    "Commanded stroke: %.2f mm",
    stroke * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Commanded down velocity: %.2f mm/s",
    down_velocity * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Commanded up velocity: %.2f mm/s",
    up_velocity * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Expected total time: %.3f sec",
    commanded_total_time);

  RCLCPP_INFO(
    node->get_logger(),
    "Actual total time: %.3f sec",
    actual_total_time);

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  rclcpp::shutdown();

  if (spinner.joinable())
    spinner.join();

  return 0;
}
