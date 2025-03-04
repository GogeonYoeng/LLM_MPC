#pragma once

#include <Eigen/Dense>
#include <vector>

class MPCController
{
public:
  MPCController(double time_horizon = 2.0, int num_steps = 20);

  // (옵션) 레퍼런스 트래젝터리 설정
  void setReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_trajectory);
  void setObstaclePosition(const Eigen::VectorXd& obstacle_position); // <-- 추가

  // 핵심: 현재 상태를 받아 최적 제어 입력(or 관절 위치) 반환
  Eigen::VectorXd computeOptimalControl(const Eigen::VectorXd& current_state);

private:
  double time_horizon_;
  int num_steps_;
  std::vector<Eigen::VectorXd> reference_trajectory_;
  Eigen::VectorXd obstacle_position_;
  
};