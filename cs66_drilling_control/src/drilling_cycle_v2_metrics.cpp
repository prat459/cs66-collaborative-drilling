#include <memory>
#include <cmath>
#include <thread>
#include <vector>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>


// =====================================================
// TCP sample stored during physical execution
// =====================================================

struct TcpSample
{
  double time_s;

  int cycle;

  double x;
  double y;
  double z;

  double depth_m;
  double lateral_error_m;
};


// =====================================================
// Per-cycle experimental results
// =====================================================

struct CycleMetrics
{
  int cycle = 0;

  bool success = false;

  double commanded_stroke_m = 0.0;
  double actual_stroke_m = 0.0;

  double commanded_down_velocity_m_s = 0.0;
  double actual_down_velocity_m_s = 0.0;

  double commanded_up_velocity_m_s = 0.0;
  double actual_up_velocity_m_s = 0.0;

  double commanded_down_time_s = 0.0;
  double actual_down_time_s = 0.0;

  double commanded_up_time_s = 0.0;
  double actual_up_time_s = 0.0;

  double commanded_bottom_dwell_s = 0.0;
  double actual_bottom_dwell_s = 0.0;

  double commanded_top_dwell_s = 0.0;
  double actual_top_dwell_s = 0.0;

  double commanded_cycle_time_s = 0.0;

  // Total duration of move_group.execute() + top dwell.
  double actual_cycle_time_s = 0.0;

  // Duration covered by TCP samples.
  double tcp_sampled_cycle_time_s = 0.0;

  double max_lateral_error_m = 0.0;

  double start_x = 0.0;
  double start_y = 0.0;
  double start_z = 0.0;

  double bottom_x = 0.0;
  double bottom_y = 0.0;
  double bottom_z = 0.0;

  double end_x = 0.0;
  double end_y = 0.0;
  double end_z = 0.0;

  double return_error_m = 0.0;

  std::size_t sample_count = 0;
};


// =====================================================
// Analyze TCP samples for one cycle
// =====================================================

CycleMetrics analyzeCycle(
  int cycle_number,
  const std::vector<TcpSample> & samples,
  double commanded_stroke,
  double commanded_down_velocity,
  double commanded_up_velocity,
  double commanded_down_time,
  double commanded_up_time,
  double commanded_bottom_dwell,
  double commanded_top_dwell,
  double commanded_cycle_time,
  double actual_cycle_time,
  double actual_top_dwell,
  bool success)
{
  CycleMetrics metrics;

  metrics.cycle = cycle_number;
  metrics.success = success;

  metrics.commanded_stroke_m =
    commanded_stroke;

  metrics.commanded_down_velocity_m_s =
    commanded_down_velocity;

  metrics.commanded_up_velocity_m_s =
    commanded_up_velocity;

  metrics.commanded_down_time_s =
    commanded_down_time;

  metrics.commanded_up_time_s =
    commanded_up_time;

  metrics.commanded_bottom_dwell_s =
    commanded_bottom_dwell;

  metrics.commanded_top_dwell_s =
    commanded_top_dwell;

  metrics.commanded_cycle_time_s =
    commanded_cycle_time;

  metrics.actual_cycle_time_s =
    actual_cycle_time;

  metrics.actual_top_dwell_s =
    actual_top_dwell;

  metrics.sample_count =
    samples.size();


  if (samples.size() < 3)
  {
    return metrics;
  }


  // -----------------------------------------------------
  // Start/end TCP
  // -----------------------------------------------------

  metrics.start_x = samples.front().x;
  metrics.start_y = samples.front().y;
  metrics.start_z = samples.front().z;

  metrics.end_x = samples.back().x;
  metrics.end_y = samples.back().y;
  metrics.end_z = samples.back().z;

  metrics.tcp_sampled_cycle_time_s =
    samples.back().time_s -
    samples.front().time_s;


  // -----------------------------------------------------
  // Find deepest measured TCP point
  // -----------------------------------------------------

  auto max_depth_it =
    std::max_element(
      samples.begin(),
      samples.end(),
      [](const TcpSample & a,
         const TcpSample & b)
      {
        return a.depth_m < b.depth_m;
      });

  if (max_depth_it == samples.end())
  {
    return metrics;
  }

  const std::size_t max_index =
    static_cast<std::size_t>(
      std::distance(
        samples.begin(),
        max_depth_it));

  metrics.actual_stroke_m =
    max_depth_it->depth_m;

  metrics.bottom_x =
    max_depth_it->x;

  metrics.bottom_y =
    max_depth_it->y;

  metrics.bottom_z =
    max_depth_it->z;


  // -----------------------------------------------------
  // Maximum lateral deviation
  // -----------------------------------------------------

  for (const auto & sample : samples)
  {
    metrics.max_lateral_error_m =
      std::max(
        metrics.max_lateral_error_m,
        sample.lateral_error_m);
  }


  // -----------------------------------------------------
  // Return error
  // -----------------------------------------------------

  const double return_dx =
    metrics.end_x - metrics.start_x;

  const double return_dy =
    metrics.end_y - metrics.start_y;

  const double return_dz =
    metrics.end_z - metrics.start_z;

  metrics.return_error_m =
    std::sqrt(
      return_dx * return_dx +
      return_dy * return_dy +
      return_dz * return_dz);


  // -----------------------------------------------------
  // Motion thresholds
  //
  // Measure between 2% and 98% of actual stroke.
  // -----------------------------------------------------

  if (metrics.actual_stroke_m > 0.001)
  {
    const double low_threshold =
      0.02 * metrics.actual_stroke_m;

    const double high_threshold =
      0.98 * metrics.actual_stroke_m;


    // ===================================================
    // DOWN motion
    // ===================================================

    std::size_t down_start_index = 0;
    std::size_t down_end_index = max_index;

    bool found_down_start = false;
    bool found_down_end = false;


    for (std::size_t i = 0;
         i <= max_index;
         ++i)
    {
      if (!found_down_start &&
          samples[i].depth_m >= low_threshold)
      {
        down_start_index = i;
        found_down_start = true;
      }

      if (!found_down_end &&
          samples[i].depth_m >= high_threshold)
      {
        down_end_index = i;
        found_down_end = true;
        break;
      }
    }


    if (found_down_start &&
        found_down_end &&
        down_end_index > down_start_index)
    {
      metrics.actual_down_time_s =
        samples[down_end_index].time_s -
        samples[down_start_index].time_s;

      const double measured_down_distance =
        samples[down_end_index].depth_m -
        samples[down_start_index].depth_m;

      if (metrics.actual_down_time_s > 0.0)
      {
        metrics.actual_down_velocity_m_s =
          measured_down_distance /
          metrics.actual_down_time_s;
      }
    }


    // ===================================================
    // UP motion
    // ===================================================

    std::size_t up_start_index = max_index;
    std::size_t up_end_index =
      samples.size() - 1;

    bool found_up_start = false;
    bool found_up_end = false;


    for (std::size_t i = max_index;
         i < samples.size();
         ++i)
    {
      if (!found_up_start &&
          samples[i].depth_m <= high_threshold)
      {
        up_start_index = i;
        found_up_start = true;
      }

      if (found_up_start &&
          !found_up_end &&
          samples[i].depth_m <= low_threshold)
      {
        up_end_index = i;
        found_up_end = true;
        break;
      }
    }


    if (found_up_start &&
        found_up_end &&
        up_end_index > up_start_index)
    {
      metrics.actual_up_time_s =
        samples[up_end_index].time_s -
        samples[up_start_index].time_s;

      const double measured_up_distance =
        samples[up_start_index].depth_m -
        samples[up_end_index].depth_m;

      if (metrics.actual_up_time_s > 0.0)
      {
        metrics.actual_up_velocity_m_s =
          measured_up_distance /
          metrics.actual_up_time_s;
      }
    }


    // ===================================================
    // Bottom dwell
    //
    // Actual measured time within 0.5 mm of maximum depth.
    // ===================================================

    const double dwell_tolerance_m =
      0.00001;

    std::size_t dwell_start_index =
      max_index;

    std::size_t dwell_end_index =
      max_index;


    while (dwell_start_index > 0)
    {
      const double difference =
        metrics.actual_stroke_m -
        samples[dwell_start_index - 1].depth_m;

      if (difference <= dwell_tolerance_m)
      {
        dwell_start_index--;
      }
      else
      {
        break;
      }
    }


    while (dwell_end_index + 1 <
           samples.size())
    {
      const double difference =
        metrics.actual_stroke_m -
        samples[dwell_end_index + 1].depth_m;

      if (difference <= dwell_tolerance_m)
      {
        dwell_end_index++;
      }
      else
      {
        break;
      }
    }


    if (dwell_end_index >
        dwell_start_index)
    {
      metrics.actual_bottom_dwell_s =
        samples[dwell_end_index].time_s -
        samples[dwell_start_index].time_s;
    }
  }


  return metrics;
}


// =====================================================
// Main
// =====================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp::Node>(
      "drilling_cycle_v2_metrics",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true));


  // =====================================================
  // User-configurable experiment parameters
  // =====================================================

  bool execute_motion = false;

  double stroke = 0.025;
  double down_velocity = 0.010;
  double up_velocity = 0.020;

  int cycles = 5;

  double bottom_dwell = 0.0;
  double top_dwell = 0.0;


  node->get_parameter_or(
    "execute",
    execute_motion,
    false);

  node->get_parameter_or(
    "stroke",
    stroke,
    0.025);

  node->get_parameter_or(
    "down_velocity",
    down_velocity,
    0.010);

  node->get_parameter_or(
    "up_velocity",
    up_velocity,
    0.020);

  node->get_parameter_or(
    "cycles",
    cycles,
    5);

  node->get_parameter_or(
    "bottom_dwell",
    bottom_dwell,
    0.0);

  node->get_parameter_or(
    "top_dwell",
    top_dwell,
    0.0);


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


  std::thread spinner(
    [&executor]()
    {
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
    {
      spinner.join();
    }

    return 1;
  }


  geometry_msgs::msg::Pose up_pose =
    move_group.getCurrentPose().pose;


  // =====================================================
  // Tool +Z direction
  // =====================================================

  const double qx =
    up_pose.orientation.x;

  const double qy =
    up_pose.orientation.y;

  const double qz =
    up_pose.orientation.z;

  const double qw =
    up_pose.orientation.w;


  const double tool_z_x =
    2.0 * (qx * qz + qw * qy);

  const double tool_z_y =
    2.0 * (qy * qz - qw * qx);

  const double tool_z_z =
    1.0 -
    2.0 * (qx * qx + qy * qy);


  // =====================================================
  // DOWN pose
  // =====================================================

  geometry_msgs::msg::Pose down_pose =
    up_pose;


  down_pose.position.x +=
    stroke * tool_z_x;

  down_pose.position.y +=
    stroke * tool_z_y;

  down_pose.position.z +=
    stroke * tool_z_z;


  // =====================================================
  // TCP recorder
  // =====================================================

  std::mutex sample_mutex;

  std::vector<TcpSample> tcp_samples;

  std::atomic<int> active_cycle{0};

  std::atomic<bool> recording{false};


  const auto recording_clock_start =
    std::chrono::steady_clock::now();


  auto tcp_subscription =
    node->create_subscription<
      geometry_msgs::msg::PoseStamped>(
        "/tcp_pose_broadcaster/pose",
        rclcpp::SensorDataQoS(),

        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          if (!recording.load())
          {
            return;
          }


          const int cycle_number =
            active_cycle.load();


          if (cycle_number <= 0)
          {
            return;
          }


          const auto now =
            std::chrono::steady_clock::now();


          const double time_s =
            std::chrono::duration<double>(
              now -
              recording_clock_start).count();


          // IMPORTANT:
          // Restore the known-working reference.
          //
          // Do NOT use the experimental tcp_reference_x/y/z
          // modification from the previous test.

          const double dx =
            msg->pose.position.x -
            up_pose.position.x;

          const double dy =
            msg->pose.position.y -
            up_pose.position.y;

          const double dz =
            msg->pose.position.z -
            up_pose.position.z;


          const double depth_m =
            dx * tool_z_x +
            dy * tool_z_y +
            dz * tool_z_z;


          const double lateral_x =
            dx -
            depth_m * tool_z_x;

          const double lateral_y =
            dy -
            depth_m * tool_z_y;

          const double lateral_z =
            dz -
            depth_m * tool_z_z;


          const double lateral_error_m =
            std::sqrt(
              lateral_x * lateral_x +
              lateral_y * lateral_y +
              lateral_z * lateral_z);


          TcpSample sample;

          sample.time_s =
            time_s;

          sample.cycle =
            cycle_number;

          sample.x =
            msg->pose.position.x;

          sample.y =
            msg->pose.position.y;

          sample.z =
            msg->pose.position.z;

          sample.depth_m =
            depth_m;

          sample.lateral_error_m =
            lateral_error_m;


          std::lock_guard<std::mutex> lock(
            sample_mutex);


          tcp_samples.push_back(
            sample);
        });


  (void)tcp_subscription;
  // =====================================================
  // Print experiment configuration
  // =====================================================

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "CS66 DRILLING CYCLE V2 + METRICS");

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
    "Bottom dwell: %.3f sec",
    bottom_dwell);


  RCLCPP_INFO(
    node->get_logger(),
    "Top dwell: %.3f sec",
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
// Generate DOWN and UP as TWO Cartesian paths,
// then merge them into ONE trajectory for execution.
//
// This guarantees that the DOWN endpoint is explicitly
// represented in the trajectory.
// =====================================================

moveit_msgs::msg::RobotTrajectory down_trajectory;
moveit_msgs::msg::RobotTrajectory up_trajectory;


// =====================================================
// 1. Generate current UP -> DOWN
// =====================================================

std::vector<geometry_msgs::msg::Pose> down_waypoints;

down_waypoints.push_back(
  down_pose);


const double down_fraction =
  move_group.computeCartesianPath(
    down_waypoints,
    0.0005,
    0.0,
    down_trajectory,
    true);


RCLCPP_INFO(
  node->get_logger(),
  "DOWN Cartesian completion: %.1f%%",
  down_fraction * 100.0);


if (down_fraction < 0.999)
{
  RCLCPP_ERROR(
    node->get_logger(),
    "Incomplete DOWN Cartesian path. Nothing executed.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


if (down_trajectory.joint_trajectory.points.empty())
{
  RCLCPP_ERROR(
    node->get_logger(),
    "DOWN trajectory contains no points.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


// =====================================================
// 2. Verify exact DOWN endpoint with FK
// =====================================================

const std::string end_effector_link =
  move_group.getEndEffectorLink();


moveit::core::RobotState down_end_state(
  move_group.getRobotModel());

down_end_state.setToDefaultValues();

down_end_state.setVariablePositions(
  down_trajectory.joint_trajectory.joint_names,
  down_trajectory.joint_trajectory.points.back().positions);

down_end_state.update();


const Eigen::Isometry3d & down_end_tf =
  down_end_state.getGlobalLinkTransform(
    end_effector_link);


const double down_error_x =
  down_end_tf.translation().x() -
  down_pose.position.x;

const double down_error_y =
  down_end_tf.translation().y() -
  down_pose.position.y;

const double down_error_z =
  down_end_tf.translation().z() -
  down_pose.position.z;


const double down_endpoint_error =
  std::sqrt(
    down_error_x * down_error_x +
    down_error_y * down_error_y +
    down_error_z * down_error_z);


RCLCPP_INFO(
  node->get_logger(),
  "DOWN endpoint FK: x=%.6f y=%.6f z=%.6f",
  down_end_tf.translation().x(),
  down_end_tf.translation().y(),
  down_end_tf.translation().z());


RCLCPP_INFO(
  node->get_logger(),
  "DOWN endpoint error from requested DOWN: %.3f mm",
  down_endpoint_error * 1000.0);


// =====================================================
// 3. Generate DOWN -> UP
//
// Force this Cartesian path to start from the exact
// final joint state of the DOWN trajectory.
// =====================================================

move_group.setStartState(
  down_end_state);


std::vector<geometry_msgs::msg::Pose> up_waypoints;

up_waypoints.push_back(
  up_pose);


const double up_fraction =
  move_group.computeCartesianPath(
    up_waypoints,
    0.0005,
    0.0,
    up_trajectory,
    true);


// Restore normal MoveIt start-state behavior.
move_group.setStartStateToCurrentState();


RCLCPP_INFO(
  node->get_logger(),
  "UP Cartesian completion: %.1f%%",
  up_fraction * 100.0);


if (up_fraction < 0.999)
{
  RCLCPP_ERROR(
    node->get_logger(),
    "Incomplete UP Cartesian path. Nothing executed.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


if (up_trajectory.joint_trajectory.points.empty())
{
  RCLCPP_ERROR(
    node->get_logger(),
    "UP trajectory contains no points.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


// =====================================================
// 4. Merge DOWN + dwell + UP into ONE trajectory
// =====================================================

moveit_msgs::msg::RobotTrajectory trajectory;


trajectory.joint_trajectory.joint_names =
  down_trajectory.joint_trajectory.joint_names;

trajectory.joint_trajectory.header =
  down_trajectory.joint_trajectory.header;


auto & points =
  trajectory.joint_trajectory.points;


const auto & down_points =
  down_trajectory.joint_trajectory.points;

const auto & up_points =
  up_trajectory.joint_trajectory.points;


// Copy complete DOWN trajectory.
points.insert(
  points.end(),
  down_points.begin(),
  down_points.end());


if (points.empty())
{
  RCLCPP_ERROR(
    node->get_logger(),
    "Merged trajectory contains no DOWN points.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


// The final DOWN point is now the real bottom.
const std::size_t bottom_index =
  points.size() - 1;


// =====================================================
// 5. TRUE BOTTOM DWELL
//
// Duplicate the exact final DOWN joint position.
// =====================================================

if (bottom_dwell > 0.0)
{
  points.push_back(
    points[bottom_index]);
}


const std::size_t dwell_index =
  points.size() - 1;


// =====================================================
// 6. Append UP trajectory
//
// If the first UP point is identical to the bottom,
// skip that duplicate.
// =====================================================

std::size_t up_begin_index = 0;


if (!up_points.empty())
{
  const auto & bottom_positions =
    points[bottom_index].positions;

  const auto & first_up_positions =
    up_points.front().positions;


  if (bottom_positions.size() ==
      first_up_positions.size())
  {
    double joint_difference_squared =
      0.0;


    for (std::size_t j = 0;
         j < bottom_positions.size();
         ++j)
    {
      const double difference =
        first_up_positions[j] -
        bottom_positions[j];

      joint_difference_squared +=
        difference * difference;
    }


    if (std::sqrt(
          joint_difference_squared) <
        1e-9)
    {
      up_begin_index = 1;
    }
  }
}


points.insert(
  points.end(),
  up_points.begin() +
    static_cast<std::ptrdiff_t>(
      up_begin_index),
  up_points.end());


if (points.size() < 3)
{
  RCLCPP_ERROR(
    node->get_logger(),
    "Merged trajectory contains too few points.");

  rclcpp::shutdown();

  if (spinner.joinable())
  {
    spinner.join();
  }

  return 1;
}


RCLCPP_INFO(
  node->get_logger(),
  "DOWN trajectory points: %zu",
  down_points.size());


RCLCPP_INFO(
  node->get_logger(),
  "UP trajectory points: %zu",
  up_points.size());


RCLCPP_INFO(
  node->get_logger(),
  "Exact bottom index in merged trajectory: %zu",
  bottom_index);


RCLCPP_INFO(
  node->get_logger(),
  "Merged trajectory points: %zu",
  points.size());


// =====================================================
// 7. Explicit timing
//
// UP -> DOWN : down_time
// HOLD DOWN  : bottom_dwell
// DOWN -> UP : up_time
// =====================================================

const std::size_t last_index =
  points.size() - 1;


const std::size_t up_motion_start_index =
  (bottom_dwell > 0.0)
    ? dwell_index
    : bottom_index;


for (std::size_t i = 0;
     i < points.size();
     ++i)
{
  double t = 0.0;


  // -------------------------
  // DOWN portion
  // -------------------------

  if (i <= bottom_index)
  {
    if (bottom_index == 0)
    {
      t = 0.0;
    }
    else
    {
      t =
        down_time *
        static_cast<double>(i) /
        static_cast<double>(
          bottom_index);
    }
  }

  // -------------------------
  // Bottom dwell point
  // -------------------------

  else if (
    bottom_dwell > 0.0 &&
    i == dwell_index)
  {
    t =
      down_time +
      bottom_dwell;
  }

  // -------------------------
  // UP portion
  // -------------------------

  else
  {
    const std::size_t up_point_count =
      last_index -
      up_motion_start_index;


    if (up_point_count == 0)
    {
      t =
        down_time +
        bottom_dwell +
        up_time;
    }
    else
    {
      t =
        down_time +
        bottom_dwell +
        up_time *
        static_cast<double>(
          i - up_motion_start_index) /
        static_cast<double>(
          up_point_count);
    }
  }


  const int32_t sec =
    static_cast<int32_t>(
      std::floor(t));


  const uint32_t nanosec =
    static_cast<uint32_t>(
      (t -
       static_cast<double>(sec)) *
      1e9);


  points[i].time_from_start.sec =
    sec;

  points[i].time_from_start.nanosec =
    nanosec;


  points[i].velocities.clear();
  points[i].accelerations.clear();
  points[i].effort.clear();
}

RCLCPP_INFO(
  node->get_logger(),
  "TRAJECTORY TIMESTAMPS: bottom=%.3f s | dwell_end=%.3f s | final=%.3f s",
  points[bottom_index].time_from_start.sec +
    points[bottom_index].time_from_start.nanosec * 1e-9,
  points[dwell_index].time_from_start.sec +
    points[dwell_index].time_from_start.nanosec * 1e-9,
  points[last_index].time_from_start.sec +
    points[last_index].time_from_start.nanosec * 1e-9);

// =====================================================
// 8. Verify merged trajectory bottom
// =====================================================

moveit::core::RobotState merged_bottom_state(
  move_group.getRobotModel());

merged_bottom_state.setToDefaultValues();

merged_bottom_state.setVariablePositions(
  trajectory.joint_trajectory.joint_names,
  points[bottom_index].positions);

merged_bottom_state.update();


const Eigen::Isometry3d & merged_bottom_tf =
  merged_bottom_state.getGlobalLinkTransform(
    end_effector_link);


const double merged_dx =
  merged_bottom_tf.translation().x() -
  down_pose.position.x;

const double merged_dy =
  merged_bottom_tf.translation().y() -
  down_pose.position.y;

const double merged_dz =
  merged_bottom_tf.translation().z() -
  down_pose.position.z;


const double merged_bottom_error =
  std::sqrt(
    merged_dx * merged_dx +
    merged_dy * merged_dy +
    merged_dz * merged_dz);


RCLCPP_INFO(
  node->get_logger(),
  "MERGED bottom FK: x=%.6f y=%.6f z=%.6f",
  merged_bottom_tf.translation().x(),
  merged_bottom_tf.translation().y(),
  merged_bottom_tf.translation().z());


RCLCPP_INFO(
  node->get_logger(),
  "MERGED bottom error from requested DOWN: %.3f mm",
  merged_bottom_error * 1000.0);


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


  for (int i = 0;
       i < 10;
       ++i)
  {
    display_pub->publish(
      display_msg);

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
      "No physical metrics recorded.");

    RCLCPP_INFO(
      node->get_logger(),
      "========================================");


    rclcpp::shutdown();


    if (spinner.joinable())
    {
      spinner.join();
    }


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
  // CSV files
  // =====================================================

  const std::string raw_csv_name =
    "drilling_raw.csv";


  const std::string summary_csv_name =
    "drilling_summary.csv";


  std::ofstream raw_csv(
    raw_csv_name);


  std::ofstream summary_csv(
    summary_csv_name);


  if (!raw_csv.is_open() ||
      !summary_csv.is_open())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not create CSV output files.");

    rclcpp::shutdown();

    if (spinner.joinable())
    {
      spinner.join();
    }

    return 1;
  }


  raw_csv
    << std::fixed
    << std::setprecision(9);


  summary_csv
    << std::fixed
    << std::setprecision(6);


  raw_csv
    << "time_s,"
    << "cycle,"
    << "tcp_x_m,"
    << "tcp_y_m,"
    << "tcp_z_m,"
    << "depth_mm,"
    << "lateral_error_mm"
    << "\n";


  summary_csv
    << "cycle,"
    << "success,"
    << "commanded_stroke_mm,"
    << "actual_stroke_mm,"
    << "stroke_error_mm,"
    << "stroke_error_percent,"
    << "commanded_down_velocity_mm_s,"
    << "actual_down_velocity_mm_s,"
    << "down_velocity_error_mm_s,"
    << "commanded_up_velocity_mm_s,"
    << "actual_up_velocity_mm_s,"
    << "up_velocity_error_mm_s,"
    << "commanded_down_time_s,"
    << "actual_down_time_s,"
    << "commanded_up_time_s,"
    << "actual_up_time_s,"
    << "commanded_bottom_dwell_s,"
    << "actual_bottom_dwell_s,"
    << "commanded_top_dwell_s,"
    << "actual_top_dwell_s,"
    << "commanded_cycle_time_s,"
    << "actual_cycle_time_s,"
    << "tcp_sampled_cycle_time_s,"
    << "max_lateral_error_mm,"
    << "return_error_mm,"
    << "start_x_m,"
    << "start_y_m,"
    << "start_z_m,"
    << "bottom_x_m,"
    << "bottom_y_m,"
    << "bottom_z_m,"
    << "end_x_m,"
    << "end_y_m,"
    << "end_z_m,"
    << "sample_count"
    << "\n";

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
    "TCP METRICS RECORDING ENABLED");

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


  std::vector<CycleMetrics> all_metrics;


  for (int cycle = 1;
       cycle <= cycles &&
       rclcpp::ok();
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


    std::size_t sample_start_index = 0;


    {
      std::lock_guard<std::mutex> lock(
        sample_mutex);

      sample_start_index =
        tcp_samples.size();
    }


    active_cycle.store(
      cycle);

    recording.store(
      true);


    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));


    RCLCPP_INFO(
      node->get_logger(),
      "Executing complete DOWN -> DWELL -> UP trajectory");


    const auto cycle_start =
      std::chrono::steady_clock::now();


    const auto result =
      move_group.execute(
        cycle_plan);


    const bool cycle_success =
      result ==
      moveit::core::MoveItErrorCode::SUCCESS;


    // ---------------------------------------------------
    // TOP DWELL
    // ---------------------------------------------------

    double actual_top_dwell =
      0.0;


    if (cycle_success &&
        top_dwell > 0.0)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "State: TOP_DWELL (%.3f s)",
        top_dwell);


      const auto top_dwell_start =
        std::chrono::steady_clock::now();


      std::this_thread::sleep_for(
        std::chrono::duration<double>(
          top_dwell));


      const auto top_dwell_end =
        std::chrono::steady_clock::now();


      actual_top_dwell =
        std::chrono::duration<double>(
          top_dwell_end -
          top_dwell_start).count();
    }


    const auto cycle_end =
      std::chrono::steady_clock::now();


    const double actual_cycle_time =
      std::chrono::duration<double>(
        cycle_end -
        cycle_start).count();


    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));


    recording.store(
      false);

    active_cycle.store(
      0);


    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));


    // ---------------------------------------------------
    // Copy samples belonging to this cycle
    // ---------------------------------------------------

    std::vector<TcpSample> cycle_samples;


    {
      std::lock_guard<std::mutex> lock(
        sample_mutex);


      const std::size_t sample_end_index =
        tcp_samples.size();


      if (sample_end_index >
          sample_start_index)
      {
        cycle_samples.assign(
          tcp_samples.begin() +
            static_cast<std::ptrdiff_t>(
              sample_start_index),
          tcp_samples.begin() +
            static_cast<std::ptrdiff_t>(
              sample_end_index));
      }
    }


    // ---------------------------------------------------
    // Analyze measured data
    // ---------------------------------------------------

    CycleMetrics metrics =
      analyzeCycle(
        cycle,
        cycle_samples,
        stroke,
        down_velocity,
        up_velocity,
        down_time,
        up_time,
        bottom_dwell,
        top_dwell,
        commanded_cycle_time,
        actual_cycle_time,
        actual_top_dwell,
        cycle_success);


    // ---------------------------------------------------
    // Physical completion check
    // ---------------------------------------------------

    const bool physical_cycle_complete =
      metrics.actual_stroke_m >=
        0.90 * stroke &&
      metrics.return_error_m <=
        0.002;


    const bool moveit_success =
      cycle_success;


    const bool final_cycle_success =
      moveit_success ||
      physical_cycle_complete;


    metrics.success =
      final_cycle_success;


    if (!moveit_success &&
        physical_cycle_complete)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "MoveIt reported execution failure, but TCP confirms "
        "physical cycle completion.");
    }


    all_metrics.push_back(
      metrics);


    // ---------------------------------------------------
    // Raw CSV
    // ---------------------------------------------------

    for (const auto & sample :
         cycle_samples)
    {
      raw_csv
        << sample.time_s << ","
        << sample.cycle << ","
        << sample.x << ","
        << sample.y << ","
        << sample.z << ","
        << sample.depth_m * 1000.0 << ","
        << sample.lateral_error_m * 1000.0
        << "\n";
    }


    raw_csv.flush();


    // ---------------------------------------------------
    // Errors
    // ---------------------------------------------------

    const double stroke_error_mm =
      (metrics.actual_stroke_m -
       metrics.commanded_stroke_m) *
      1000.0;


    double stroke_error_percent =
      0.0;


    if (metrics.commanded_stroke_m > 0.0)
    {
      stroke_error_percent =
        100.0 *
        (metrics.actual_stroke_m -
         metrics.commanded_stroke_m) /
        metrics.commanded_stroke_m;
    }


    const double down_velocity_error_mm_s =
      (metrics.actual_down_velocity_m_s -
       metrics.commanded_down_velocity_m_s) *
      1000.0;


    const double up_velocity_error_mm_s =
      (metrics.actual_up_velocity_m_s -
       metrics.commanded_up_velocity_m_s) *
      1000.0;


    // ---------------------------------------------------
    // Summary CSV
    // ---------------------------------------------------

    summary_csv
      << metrics.cycle << ","
      << (metrics.success ? 1 : 0) << ","
      << metrics.commanded_stroke_m * 1000.0 << ","
      << metrics.actual_stroke_m * 1000.0 << ","
      << stroke_error_mm << ","
      << stroke_error_percent << ","
      << metrics.commanded_down_velocity_m_s * 1000.0 << ","
      << metrics.actual_down_velocity_m_s * 1000.0 << ","
      << down_velocity_error_mm_s << ","
      << metrics.commanded_up_velocity_m_s * 1000.0 << ","
      << metrics.actual_up_velocity_m_s * 1000.0 << ","
      << up_velocity_error_mm_s << ","
      << metrics.commanded_down_time_s << ","
      << metrics.actual_down_time_s << ","
      << metrics.commanded_up_time_s << ","
      << metrics.actual_up_time_s << ","
      << metrics.commanded_bottom_dwell_s << ","
      << metrics.actual_bottom_dwell_s << ","
      << metrics.commanded_top_dwell_s << ","
      << metrics.actual_top_dwell_s << ","
      << metrics.commanded_cycle_time_s << ","
      << metrics.actual_cycle_time_s << ","
      << metrics.tcp_sampled_cycle_time_s << ","
      << metrics.max_lateral_error_m * 1000.0 << ","
      << metrics.return_error_m * 1000.0 << ","
      << metrics.start_x << ","
      << metrics.start_y << ","
      << metrics.start_z << ","
      << metrics.bottom_x << ","
      << metrics.bottom_y << ","
      << metrics.bottom_z << ","
      << metrics.end_x << ","
      << metrics.end_y << ","
      << metrics.end_z << ","
      << metrics.sample_count
      << "\n";


    summary_csv.flush();


    // ---------------------------------------------------
    // Terminal metrics
    // ---------------------------------------------------

    RCLCPP_INFO(
      node->get_logger(),
      "Cycle %d metrics:",
      cycle);


    RCLCPP_INFO(
      node->get_logger(),
      "  Stroke: command %.2f mm | actual %.2f mm | error %+.2f mm",
      stroke * 1000.0,
      metrics.actual_stroke_m * 1000.0,
      stroke_error_mm);


    RCLCPP_INFO(
      node->get_logger(),
      "  DOWN velocity: command %.2f mm/s | actual %.2f mm/s",
      down_velocity * 1000.0,
      metrics.actual_down_velocity_m_s * 1000.0);


    RCLCPP_INFO(
      node->get_logger(),
      "  UP velocity: command %.2f mm/s | actual %.2f mm/s",
      up_velocity * 1000.0,
      metrics.actual_up_velocity_m_s * 1000.0);


    RCLCPP_INFO(
      node->get_logger(),
      "  DOWN time: command %.3f s | measured %.3f s",
      down_time,
      metrics.actual_down_time_s);


    RCLCPP_INFO(
      node->get_logger(),
      "  UP time: command %.3f s | measured %.3f s",
      up_time,
      metrics.actual_up_time_s);


    RCLCPP_INFO(
      node->get_logger(),
      "  Bottom dwell: command %.3f s | measured %.3f s",
      bottom_dwell,
      metrics.actual_bottom_dwell_s);


    RCLCPP_INFO(
      node->get_logger(),
      "  Cycle: command %.3f s | execute+top dwell %.3f s",
      commanded_cycle_time,
      metrics.actual_cycle_time_s);


    RCLCPP_INFO(
      node->get_logger(),
      "  Max lateral deviation: %.3f mm",
      metrics.max_lateral_error_m * 1000.0);


    RCLCPP_INFO(
      node->get_logger(),
      "  Return error: %.3f mm",
      metrics.return_error_m * 1000.0);


    RCLCPP_INFO(
      node->get_logger(),
      "  TCP samples: %zu",
      metrics.sample_count);


    if (!final_cycle_success)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Cycle %d physically incomplete - stopping experiment.",
        cycle);

      break;
    }


    completed_cycles++;


    RCLCPP_INFO(
      node->get_logger(),
      "Cycle %d complete.",
      cycle);
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


  raw_csv.close();
  summary_csv.close();


  // =====================================================
  // Calculate experiment averages
  // =====================================================

  double mean_stroke_m = 0.0;
  double mean_down_velocity_m_s = 0.0;
  double mean_up_velocity_m_s = 0.0;
  double mean_bottom_dwell_s = 0.0;
  double mean_cycle_time_s = 0.0;
  double mean_lateral_error_m = 0.0;
  double mean_return_error_m = 0.0;


  if (!all_metrics.empty())
  {
    for (const auto & metrics :
         all_metrics)
    {
      mean_stroke_m +=
        metrics.actual_stroke_m;

      mean_down_velocity_m_s +=
        metrics.actual_down_velocity_m_s;

      mean_up_velocity_m_s +=
        metrics.actual_up_velocity_m_s;

      mean_bottom_dwell_s +=
        metrics.actual_bottom_dwell_s;

      mean_cycle_time_s +=
        metrics.actual_cycle_time_s;

      mean_lateral_error_m +=
        metrics.max_lateral_error_m;

      mean_return_error_m +=
        metrics.return_error_m;
    }


    const double count =
      static_cast<double>(
        all_metrics.size());


    mean_stroke_m /=
      count;

    mean_down_velocity_m_s /=
      count;

    mean_up_velocity_m_s /=
      count;

    mean_bottom_dwell_s /=
      count;

    mean_cycle_time_s /=
      count;

    mean_lateral_error_m /=
      count;

    mean_return_error_m /=
      count;
  }


  RCLCPP_INFO(
    node->get_logger(),
    "========================================");


  RCLCPP_INFO(
    node->get_logger(),
    "DRILLING V2 METRICS TEST COMPLETE");


  RCLCPP_INFO(
    node->get_logger(),
    "Completed cycles: %d / %d",
    completed_cycles,
    cycles);


  RCLCPP_INFO(
    node->get_logger(),
    "----------------------------------------");


  RCLCPP_INFO(
    node->get_logger(),
    "COMMAND vs MEASURED MEAN");


  RCLCPP_INFO(
    node->get_logger(),
    "Stroke: %.2f mm -> %.2f mm",
    stroke * 1000.0,
    mean_stroke_m * 1000.0);


  RCLCPP_INFO(
    node->get_logger(),
    "DOWN velocity: %.2f mm/s -> %.2f mm/s",
    down_velocity * 1000.0,
    mean_down_velocity_m_s * 1000.0);


  RCLCPP_INFO(
    node->get_logger(),
    "UP velocity: %.2f mm/s -> %.2f mm/s",
    up_velocity * 1000.0,
    mean_up_velocity_m_s * 1000.0);


  RCLCPP_INFO(
    node->get_logger(),
    "Bottom dwell: %.3f s -> %.3f s",
    bottom_dwell,
    mean_bottom_dwell_s);


  RCLCPP_INFO(
    node->get_logger(),
    "Cycle time: %.3f s -> %.3f s",
    commanded_cycle_time,
    mean_cycle_time_s);


  RCLCPP_INFO(
    node->get_logger(),
    "Mean maximum lateral deviation: %.3f mm",
    mean_lateral_error_m * 1000.0);


  RCLCPP_INFO(
    node->get_logger(),
    "Mean return error: %.3f mm",
    mean_return_error_m * 1000.0);


  RCLCPP_INFO(
    node->get_logger(),
    "Commanded total time: %.3f sec",
    commanded_total_time);


  RCLCPP_INFO(
    node->get_logger(),
    "Actual total time: %.3f sec",
    actual_total_time);


  RCLCPP_INFO(
    node->get_logger(),
    "----------------------------------------");


  RCLCPP_INFO(
    node->get_logger(),
    "CSV FILES SAVED:");


  RCLCPP_INFO(
    node->get_logger(),
    "  %s",
    raw_csv_name.c_str());


  RCLCPP_INFO(
    node->get_logger(),
    "  %s",
    summary_csv_name.c_str());


  RCLCPP_INFO(
    node->get_logger(),
    "========================================");


  rclcpp::shutdown();


  if (spinner.joinable())
  {
    spinner.join();
  }


  return 0;
}
