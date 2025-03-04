#include <moveit/local_constraint_solver_plugins/forward_trajectory.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/logger.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit::hybrid_planning
{

ForwardTrajectory::ForwardTrajectory()
  : node_(nullptr), mpc_(2.0, 20), time_horizon_(2.0), num_steps_(20)
{
}

bool ForwardTrajectory::initialize(const rclcpp::Node::SharedPtr& node,
                                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const std::string& /* group_name */)
{
  node_ = node;
  planning_scene_monitor_ = planning_scene_monitor;  // 장애물 정보 활용을 위한 Planning Scene Monitor 설정

  // (Optional) load parameters
  if (!node_->has_parameter("time_horizon"))
    node_->declare_parameter<double>("time_horizon", time_horizon_);
  node_->get_parameter("time_horizon", time_horizon_);

  if (!node_->has_parameter("num_steps"))
    node_->declare_parameter<int>("num_steps", num_steps_);
  node_->get_parameter("num_steps", num_steps_);

  // Update internal MPC
  mpc_ = MPCController(time_horizon_, num_steps_);

  RCLCPP_INFO(node_->get_logger(), "ForwardTrajectory(MPC) initialized: horizon=%.2f, steps=%d",
              time_horizon_, num_steps_);

  return true;
}

bool ForwardTrajectory::reset()
{
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
ForwardTrajectory::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                         const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /* goal */,
                         trajectory_msgs::msg::JointTrajectory& local_solution)
{
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;
  RCLCPP_INFO(node_->get_logger(), "MPC-based local planner solve() called");

  // 1) 현재 로봇 상태 가져오기
  const moveit::core::RobotState& first_waypoint = local_trajectory.getWayPoint(0);
  std::vector<double> joint_positions;
  first_waypoint.copyJointGroupPositions(local_trajectory.getGroupName(), joint_positions);

  // 2) Eigen 벡터로 변환
  Eigen::VectorXd current_state_eig(joint_positions.size());
  for (size_t i = 0; i < joint_positions.size(); ++i)
    current_state_eig(i) = joint_positions[i];

  // 3) 장애물 정보 가져오기
  Eigen::VectorXd obstacle_position(3);
  bool obstacle_detected = false;

  if (planning_scene_monitor_)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    const auto& world_objects = scene->getWorld()->getObjectIds();
    
    if (!world_objects.empty())
    {
      const auto& obstacle = scene->getWorld()->getObject(world_objects.front());
      obstacle_position << obstacle->pose_.translation().x(),
                           obstacle->pose_.translation().y(),
                           obstacle->pose_.translation().z();
      mpc_.setObstaclePosition(obstacle_position);
      obstacle_detected = true;
    }
  }

  // 4) 충돌 검사를 수행하여 장애물이 있으면 MPC 보정
  bool is_path_valid = true;
  if (planning_scene_monitor_ && obstacle_detected)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    is_path_valid = scene->isStateValid(first_waypoint, local_trajectory.getGroupName(), true);
  }

  Eigen::VectorXd mpc_output;
  if (is_path_valid)
  {
    // 정상적으로 MPC 실행
    mpc_output = mpc_.computeOptimalControl(current_state_eig);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Obstacle detected! Adjusting trajectory...");
    feedback_result.feedback = "Obstacle detected! Adjusting trajectory...";

    // 장애물을 피하도록 회피 조작 추가
    Eigen::VectorXd adjusted_state = current_state_eig;
    adjusted_state.head(3) += Eigen::VectorXd::Constant(3, 0.05);  // 장애물 반대 방향으로 이동

    mpc_output = mpc_.computeOptimalControl(adjusted_state);
  }

  // 5) 새로운 waypoint 생성
  moveit::core::RobotState next_state(first_waypoint);
  for (size_t i = 0; i < joint_positions.size() && i < (size_t)mpc_output.size(); ++i)
  {
    next_state.setVariablePosition(i, mpc_output(i));
  }

  // 6) 로봇 궤적에 추가
  double duration = 0.1; // 100ms
  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());
  robot_command.addSuffixWayPoint(next_state, duration);

  // 7) 결과를 ROS 메시지로 변환
  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);
  local_solution = robot_command_msg.joint_trajectory;

  return feedback_result;
}

}  // namespace moveit::hybrid_planning

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::ForwardTrajectory,
                       moveit::hybrid_planning::LocalConstraintSolverInterface);