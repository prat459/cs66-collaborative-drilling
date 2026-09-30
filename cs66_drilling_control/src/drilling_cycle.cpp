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
    "drilling_cycle",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  bool execute_motion = false;
  double duration_sec = 10.0;

  node->get_parameter_or("execute", execute_motion, false);
  node->get_parameter_or("duration", duration_sec, 10.0);

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
    node, "cs_manipulator");

  move_group.startStateMonitor(5.0);

  auto display_pub =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10);

  std::this_thread::sleep_for(
    std::chrono::seconds(1));

  // =====================================================
  // Current state = UP position
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
  // Calculate drill/tool +Z direction
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
  // Experiment settings
  // =====================================================

  constexpr double STROKE = 0.040;       // 5 mm
  constexpr double HALF_STROKE_TIME = 1.0; // 1 second
  constexpr double FULL_CYCLE_TIME = 2.0;  // ~2 seconds

  // =====================================================
  // DOWN pose
  // =====================================================

  geometry_msgs::msg::Pose down_pose =
    up_pose;

  down_pose.position.x +=
    STROKE * tool_z_x;

  down_pose.position.y +=
    STROKE * tool_z_y;

  down_pose.position.z +=
    STROKE * tool_z_z;

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "CS66 REPEATED DRILLING TEST");

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
    "Stroke: %.1f mm",
    STROKE * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "Half-stroke time: %.1f sec",
    HALF_STROKE_TIME);

  RCLCPP_INFO(
    node->get_logger(),
    "Approx cycle time: %.1f sec",
    FULL_CYCLE_TIME);

  RCLCPP_INFO(
    node->get_logger(),
    "Requested experiment duration: %.1f sec",
    duration_sec);

  // =====================================================
  // Generate ONE geometric DOWN -> UP trajectory
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
  // Explicitly assign trajectory timing
  //
  // Find approximately where DOWN is reached.
  // Since path is UP -> DOWN -> UP, midpoint corresponds
  // approximately to the DOWN position.
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

  for (std::size_t i = 0;
       i < points.size();
       ++i)
  {
    double t;

    if (i <= middle_index)
    {
      if (middle_index == 0)
      {
        t = 0.0;
      }
      else
      {
        t =
          HALF_STROKE_TIME *
          static_cast<double>(i) /
          static_cast<double>(middle_index);
      }
    }
    else
    {
      const std::size_t second_half_points =
        last_index - middle_index;

      t =
        HALF_STROKE_TIME +
        HALF_STROKE_TIME *
        static_cast<double>(i - middle_index) /
        static_cast<double>(second_half_points);
    }

    const int32_t sec =
      static_cast<int32_t>(std::floor(t));

    const uint32_t nanosec =
      static_cast<uint32_t>(
        (t - static_cast<double>(sec)) * 1e9);

    points[i].time_from_start.sec =
      sec;

    points[i].time_from_start.nanosec =
      nanosec;

    // Clear velocities and accelerations.
    // Controller will follow position/time trajectory.
    points[i].velocities.clear();
    points[i].accelerations.clear();
    points[i].effort.clear();
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Trajectory explicitly timed to approximately %.1f sec.",
    FULL_CYCLE_TIME);

  // =====================================================
  // Show trajectory in RViz
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
    "Publishing DOWN -> UP trajectory to RViz...");

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
      "PLAN ONLY.");

    RCLCPP_INFO(
      node->get_logger(),
      "Physical robot did NOT move.");

    RCLCPP_INFO(
      node->get_logger(),
      "Use execute:=true only after visual verification.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 0;
  }

  // =====================================================
  // REPEATED PHYSICAL EXECUTION
  // =====================================================

  RCLCPP_WARN(
    node->get_logger(),
    "========================================");

  RCLCPP_WARN(
    node->get_logger(),
    "PHYSICAL EXECUTION ENABLED");

  RCLCPP_WARN(
    node->get_logger(),
    "Repeated UP <-> DOWN drilling starting.");

  RCLCPP_WARN(
    node->get_logger(),
    "Duration: %.1f seconds",
    duration_sec);

  RCLCPP_WARN(
    node->get_logger(),
    "========================================");

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  plan.trajectory_ =
    trajectory;

  const auto experiment_start =
    std::chrono::steady_clock::now();

  int completed_cycles = 0;

  while (rclcpp::ok())
  {
    const auto now =
      std::chrono::steady_clock::now();

    const double elapsed =
      std::chrono::duration<double>(
        now - experiment_start).count();

    if (elapsed >= duration_sec)
    {
      break;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Starting cycle %d",
      completed_cycles + 1);

    const auto result =
      move_group.execute(plan);

    if (result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Cycle execution failed. Stopping experiment.");

      break;
    }

    completed_cycles++;

    RCLCPP_INFO(
      node->get_logger(),
      "Completed cycle %d",
      completed_cycles);
  }

  const auto experiment_end =
    std::chrono::steady_clock::now();

  const double actual_duration =
    std::chrono::duration<double>(
      experiment_end -
      experiment_start).count();

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "DRILLING TEST COMPLETE");

  RCLCPP_INFO(
    node->get_logger(),
    "Completed cycles: %d",
    completed_cycles);

  RCLCPP_INFO(
    node->get_logger(),
    "Actual duration: %.2f sec",
    actual_duration);

  RCLCPP_INFO(
    node->get_logger(),
    "Stroke: %.1f mm",
    STROKE * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  rclcpp::shutdown();

  if (spinner.joinable())
    spinner.join();

  return 0;
}
