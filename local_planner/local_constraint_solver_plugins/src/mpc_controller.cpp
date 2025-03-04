#include <moveit/local_constraint_solver_plugins/mpc_controller.h>
#include <osqp/osqp.h>
#include <Eigen/Sparse>
#include <iostream>

// 판다 로봇의 관절 제한 (rad)
static const double PANDA_LOWER[7] = { -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 };
static const double PANDA_UPPER[7] = {  2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973 };

// 속도 및 가속도 제한 (rad/s, rad/s²)
static const double PANDA_VEL_LIMIT[7] = { 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61 };
static const double PANDA_ACC_LIMIT[7] = { 15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0 };

static const double SAFE_DISTANCE = 0.3;  // 장애물과의 최소 안전 거리 (m)
static const double INF = 1e9;

MPCController::MPCController(double time_horizon, int num_steps)
  : time_horizon_(time_horizon), num_steps_(num_steps)
{
}

void MPCController::setReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_trajectory)
{
  reference_trajectory_ = reference_trajectory;
}

void MPCController::setObstaclePosition(const Eigen::VectorXd& obstacle_position)
{
  obstacle_position_ = obstacle_position;
}

Eigen::VectorXd MPCController::computeOptimalControl(const Eigen::VectorXd& current_state)
{
  const int n = 7;  // 7-DOF 로봇
  const int m = 4 * n;  // 위치, 속도, 가속도, 장애물 회피 제약 포함

  // 현재 상태 벡터
  Eigen::VectorXd q = current_state.head(n);
  Eigen::VectorXd q_dot = current_state.tail(n);

  // 예측 모델 행렬 정의 (State Transition)
  double T_s = 0.1; // 샘플링 시간
  Eigen::MatrixXd A(2 * n, 2 * n);
  Eigen::MatrixXd B(2 * n, n);

  A.setIdentity();
  A.topRightCorner(n, n) = T_s * Eigen::MatrixXd::Identity(n, n);
  B.setZero();
  B.bottomRows(n) = T_s * Eigen::MatrixXd::Identity(n, n);

  // 비용 함수 행렬 설정
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  Q.topLeftCorner(n, n) = 10 * Eigen::MatrixXd::Identity(n, n);  // 위치 오차 가중치 W1
  Q.bottomRightCorner(n, n) = Eigen::MatrixXd::Identity(n, n);   // 속도 오차 가중치 W2
  Eigen::MatrixXd R = 0.01 * Eigen::MatrixXd::Identity(n, n);    // 제어 입력 가중치 W7

  // Soft Constraint (안전 거리 완화 변수)
  double epsilon_dist = 0.4; // 장애물과 가까워질 때 허용하는 거리 완화 정도
  Q.bottomRightCorner(n, n) += epsilon_dist * Eigen::MatrixXd::Identity(n, n); 

  // QP 행렬 변환
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
  P = B.transpose() * Q * B + R;
  Eigen::VectorXd q_vec = B.transpose() * Q * (A * current_state);

  // 제약 조건 설정
  Eigen::MatrixXd A_ineq(m, n);
  Eigen::VectorXd l_ineq(m);
  Eigen::VectorXd u_ineq(m);

  // 속도 제한
  A_ineq.topRows(n) = B.bottomRows(n);
  l_ineq.head(n) = -Eigen::Map<const Eigen::VectorXd>(PANDA_VEL_LIMIT, n);
  u_ineq.head(n) = Eigen::Map<const Eigen::VectorXd>(PANDA_VEL_LIMIT, n);

  // 가속도 제한
  A_ineq.middleRows(n, n) = Eigen::MatrixXd::Identity(n, n);
  l_ineq.segment(n, n) = -Eigen::Map<const Eigen::VectorXd>(PANDA_ACC_LIMIT, n);
  u_ineq.segment(n, n) = Eigen::Map<const Eigen::VectorXd>(PANDA_ACC_LIMIT, n);

  // 장애물 회피 제약 추가
  Eigen::VectorXd distance_vec = q - obstacle_position_;  // 로봇과 장애물의 거리
  A_ineq.bottomRows(n) = distance_vec.transpose();
  l_ineq.tail(n) = Eigen::VectorXd::Constant(n, SAFE_DISTANCE - epsilon_dist);
  u_ineq.tail(n) = Eigen::VectorXd::Constant(n, INF);

  // OSQP 행렬 변환 (Sparse)
  Eigen::SparseMatrix<double> P_sparse = P.sparseView();
  Eigen::SparseMatrix<double> A_sparse = A_ineq.sparseView();

  OSQPCscMatrix P_csc, A_csc;
  P_csc.m = P_sparse.rows();
  P_csc.n = P_sparse.cols();
  P_csc.nzmax = P_sparse.nonZeros();
  P_csc.p = reinterpret_cast<OSQPInt*>(P_sparse.outerIndexPtr());
  P_csc.i = reinterpret_cast<OSQPInt*>(P_sparse.innerIndexPtr());
  P_csc.x = P_sparse.valuePtr();
  P_csc.nz = -1;

  A_csc.m = A_sparse.rows();
  A_csc.n = A_sparse.cols();
  A_csc.nzmax = A_sparse.nonZeros();
  A_csc.p = reinterpret_cast<OSQPInt*>(A_sparse.outerIndexPtr());
  A_csc.i = reinterpret_cast<OSQPInt*>(A_sparse.innerIndexPtr());
  A_csc.x = A_sparse.valuePtr();
  A_csc.nz = -1;

  // OSQP 설정 및 실행
  OSQPSolver* solver = nullptr;
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.alpha = 1.0;
  settings.verbose = false;

  // if (osqp_setup(&solver, &P_csc, q_vec.data(), &A_csc, l_ineq.data(), u_ineq.data(), n, m, &settings) != 0)
  // {
  //   std::cerr << "[MPCController] OSQP setup failed!\n";
  //   return current_state;
  // }

  osqp_solve(solver);

  // 최적해 (제어 입력)
  Eigen::VectorXd solution(n);
  for (int i = 0; i < n; i++)
    solution(i) = solver->solution->x[i];

  osqp_cleanup(solver);

  return solution;
}