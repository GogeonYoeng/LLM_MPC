/*********************************************************************
 * Software License Agreement (BSD License)
 * ...
 *********************************************************************/

/* Author: (Your Name)
   Description: Minimal skeleton of a local planner plugin using MPC
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/action/local_planner.hpp>

#include "mpc_controller.h"  // Minimal MPC class

namespace moveit::hybrid_planning
{

class ForwardTrajectory : public LocalConstraintSolverInterface
{
public:
  ForwardTrajectory();
  ~ForwardTrajectory() override = default;

  // LocalConstraintSolverInterface overrides
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                  const std::string& group_name) override;

  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  rclcpp::Node::SharedPtr node_;   // ROS node handle (for logging, params)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  MPCController mpc_;             // Minimal MPC object
  

  // Optional: store some parameters
  double time_horizon_;
  int num_steps_;
};

}  // namespace moveit::hybrid_planning