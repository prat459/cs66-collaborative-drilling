# Elite CS66 Collaborative Drilling

Collaborative robotic drilling research using the Elite Robots CS66, ROS 2 Humble, MoveIt, and force/torque sensing.

## Current Progress

- Connected the physical Elite CS66 to ROS 2 Humble.
- Integrated the Elite ROS 2 driver with MoveIt and RViz.
- Verified physical joint and Cartesian robot motion.
- Verified TCP pose and joint-state feedback.
- Enabled force/torque sensor feedback.
- Developed Cartesian approach and retract motions.
- Implemented motion along the tool's local drilling axis.
- Developed automatic repeated UP-DOWN drilling motion.
- Added RViz trajectory visualization before physical execution.
- Current robot TCP pose can be used automatically as the UP/start position.
- Successfully tested a 25 mm drilling stroke.
- Successfully completed 5 consecutive physical drilling cycles.
- 5-cycle test completed in approximately 10.40 seconds.
- Created a shell script for repeatable demo execution and controller checks.

## ROS 2 Package

The main package is:

`cs66_drilling_control`

Current programs:

- `drilling_approach.cpp`
- `drilling_retract.cpp`
- `drilling_contact_test.cpp`
- `drilling_force_approach.cpp`
- `drilling_cycle.cpp`

## Current Control Architecture

The current system uses Cartesian position-controlled motion for the drilling trajectory.

The repeated drilling cycle follows the current tool orientation and generates motion along the tool's local Z-axis.

## Force/Torque Development

Force/torque feedback from the CS66 is available in ROS 2.

Initial contact-detection experiments have been performed. An offset/change in the F/T signal was observed after robot motion and must be investigated before using the measurement for closed-loop force control.

## Next Work

1. Characterize and stabilize force/torque measurements.
2. Implement filtering and bias compensation.
3. Develop reliable contact detection.
4. Add force, velocity, and displacement safety limits.
5. Implement admittance control during physical contact.
6. Develop a hybrid position/admittance control architecture.
7. Use position control during free-space approach and retract.
8. Use force-responsive control during drilling/contact.
9. Integrate control-state transitions:
   - Approach
   - Contact detection
   - Force-responsive drilling
   - Retract
10. Integrate the hybrid controller with repeated drilling cycles.
11. Record force, position, joint-state, timing, and experimental performance data.
12. Perform repeatable collaborative drilling experiments.

## Tested Platform

- Robot: Elite Robots CS66
- ROS: ROS 2 Humble
- Motion planning: MoveIt
- Visualization: RViz
- Current tested drilling stroke: 25 mm
- Tested repeated cycles: 5
- Test duration: approximately 10.40 seconds

## Safety

Physical robot experiments should only be performed after verifying the robot state, External Control connection, active trajectory controller, workspace clearance, and emergency-stop availability.
