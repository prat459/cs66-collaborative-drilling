#include <memory>
#include <cmath>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class ForceMonitor
{
public:
  void callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_fz_ = msg->wrench.force.z;
    received_ = true;
  }

  bool getFz(double & fz)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!received_)
      return false;

    fz = current_fz_;
    return true;
  }

private:
  std::mutex mutex_;
  double current_fz_{0.0};
  bool received_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "drilling_force_approach",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute_motion = false;
  node->get_parameter_or("execute", execute_motion, false);

  // Safety parameters
  constexpr double STEP_DISTANCE = 0.0005;      // 0.5 mm
  constexpr double MAX_DISTANCE = 0.010;        // 10 mm maximum
  constexpr double FORCE_THRESHOLD = 5.0;       // 5 N
  constexpr int CALIBRATION_SAMPLES = 100;

  ForceMonitor force_monitor;

  auto wrench_sub =
    node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/force_torque_sensor_broadcaster/wrench",
      10,
      [&force_monitor](
        const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
      {
        force_monitor.callback(msg);
      });

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
    RCLCPP_ERROR(
      node->get_logger(),
      "Cannot obtain current physical robot state.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 1;
  }

  /*
   * ----------------------------------------------------------
   * F/T BASELINE CALIBRATION
   * ----------------------------------------------------------
   */

  RCLCPP_INFO(
    node->get_logger(),
    "Keep tool stationary and IN AIR.");

  RCLCPP_INFO(
    node->get_logger(),
    "Collecting %d Fz samples...",
    CALIBRATION_SAMPLES);

  double baseline_sum = 0.0;
  int collected = 0;

  while (rclcpp::ok() && collected < CALIBRATION_SAMPLES)
  {
    double fz;

    if (force_monitor.getFz(fz))
    {
      baseline_sum += fz;
      collected++;

      if (collected % 20 == 0)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Calibration %d/%d   Fz=%.3f N",
          collected,
          CALIBRATION_SAMPLES,
          fz);
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(10));
    }
    else
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(20));
    }
  }

  if (collected == 0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No force sensor data received.");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 1;
  }

  const double baseline_fz =
    baseline_sum / static_cast<double>(collected);

  RCLCPP_INFO(
    node->get_logger(),
    "Baseline Fz = %.3f N",
    baseline_fz);

  /*
   * ----------------------------------------------------------
   * CURRENT TCP AND TOOL +Z
   * ----------------------------------------------------------
   */

  geometry_msgs::msg::Pose start_pose =
    move_group.getCurrentPose().pose;

  const double qx = start_pose.orientation.x;
  const double qy = start_pose.orientation.y;
  const double qz = start_pose.orientation.z;
  const double qw = start_pose.orientation.w;

  const double tool_z_x =
    2.0 * (qx * qz + qw * qy);

  const double tool_z_y =
    2.0 * (qy * qz - qw * qx);

  const double tool_z_z =
    1.0 - 2.0 * (qx * qx + qy * qy);

  RCLCPP_INFO(
    node->get_logger(),
    "Start TCP: x=%.6f y=%.6f z=%.6f",
    start_pose.position.x,
    start_pose.position.y,
    start_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Tool +Z: [%.6f %.6f %.6f]",
    tool_z_x,
    tool_z_y,
    tool_z_z);

  if (!execute_motion)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "PLAN/TEST MODE ONLY -- robot will NOT move.");

    RCLCPP_INFO(
      node->get_logger(),
      "Step = 0.5 mm | Max = 10 mm | Threshold = 5 N");

    rclcpp::shutdown();

    if (spinner.joinable())
      spinner.join();

    return 0;
  }

  /*
   * ----------------------------------------------------------
   * INCREMENTAL FORCE-MONITORED APPROACH
   * ----------------------------------------------------------
   */

  RCLCPP_WARN(
    node->get_logger(),
    "PHYSICAL FORCE APPROACH ENABLED.");

  double travelled = 0.0;

  while (rclcpp::ok() &&
         travelled < MAX_DISTANCE)
  {
    /*
     * Check force BEFORE issuing another movement.
     */

    double fz;

    if (!force_monitor.getFz(fz))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Lost force sensor data. STOPPING.");

      break;
    }

    const double external_force =
      std::abs(fz - baseline_fz);

    RCLCPP_INFO(
      node->get_logger(),
      "Travelled %.1f mm | Fz %.3f N | external %.3f N",
      travelled * 1000.0,
      fz,
      external_force);

    if (external_force >= FORCE_THRESHOLD)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "CONTACT DETECTED -- stopping approach.");

      RCLCPP_WARN(
        node->get_logger(),
        "External force = %.3f N",
        external_force);

      break;
    }

    /*
     * Get the robot's CURRENT pose before every step.
     */

    geometry_msgs::msg::Pose current_pose =
      move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose =
      current_pose;

    target_pose.position.x +=
      STEP_DISTANCE * tool_z_x;

    target_pose.position.y +=
      STEP_DISTANCE * tool_z_y;

    target_pose.position.z +=
      STEP_DISTANCE * tool_z_z;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(current_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    const double fraction =
      move_group.computeCartesianPath(
        waypoints,
        0.00025,     // 0.25 mm interpolation
        0.0,
        trajectory,
        true);

    if (fraction < 0.999)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Cartesian step planning failed (%.1f%%). STOPPING.",
        fraction * 100.0);

      break;
    }

    moveit::planning_interface::
      MoveGroupInterface::Plan plan;

    plan.trajectory_ = trajectory;

    const auto result =
      move_group.execute(plan);

    if (result !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Step execution failed. STOPPING.");

      break;
    }

    travelled += STEP_DISTANCE;

    /*
     * Give the F/T stream a moment to update after the step.
     */
    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Approach finished. Total commanded travel = %.1f mm",
    travelled * 1000.0);

  RCLCPP_INFO(
    node->get_logger(),
    "No further forward motion will be commanded.");

  rclcpp::shutdown();

  if (spinner.joinable())
    spinner.join();

  return 0;
}
