/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/controller/mpc_controller.h"

#include <glog/logging.h>

#include "common/utils/math.h"
#include "planning_core/simulation/controller/lmpc_osqp_solver.h"

namespace planning {
namespace simulation {

using Eigen::MatrixXd;

MpcController::MpcController() {}

void MpcController::Init() {
  ts_ = 0.05;
  matrix_a_piao_ = MatrixXd::Zero(nx_ + nu_, nx_ + nu_);
  matrix_b_piao_ = MatrixXd::Zero(nx_ + nu_, nu_);
  matrix_ad_ = MatrixXd::Zero(nx_, nx_);
  matrix_bd_ = MatrixXd::Zero(nx_, nu_);
  matrix_q_ = MatrixXd::Zero(nx_ + nu_, nx_ + nu_);
  matrix_r_ = MatrixXd::Zero(nu_, nu_);
  matrix_xmax_ = MatrixXd::Zero(nx_ + nu_, 1);
  matrix_xmin_ = MatrixXd::Zero(nx_ + nu_, 1);
  matrix_umax_ = MatrixXd::Zero(nu_, 1);
  matrix_umin_ = MatrixXd::Zero(nu_, 1);
  matrix_xmax_relative_ = MatrixXd::Zero(nx_ + nu_, 1);
  matrix_xmin_relative_ = MatrixXd::Zero(nx_ + nu_, 1);

  matrix_a_piao_.block(nx_, nx_, nu_, nu_) = MatrixXd::Identity(nu_, nu_);
  matrix_b_piao_.block(nx_, 0, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

  matrix_q_.block(0, 0, nx_, nx_) = 3 * Eigen::MatrixXd::Identity(nx_, nx_);
  matrix_q_.block(nx_, nx_, nu_, nu_) = 5 * Eigen::MatrixXd::Identity(nu_, nu_);

  for (int i = 0; i < nu_; ++i) {
    matrix_r_(i, i) = 50;
  }

  matrix_xmin_ << -OSQP_INFTY, -OSQP_INFTY, -OSQP_INFTY, min_speed_,
      -max_steer_;
  matrix_xmax_ << OSQP_INFTY, OSQP_INFTY, OSQP_INFTY, max_speed_, max_steer_;

  matrix_umin_ << min_acc_ * ts_, -max_srate_ * ts_;
  matrix_umax_ << max_acc_ * ts_, max_srate_ * ts_;

  last_control_.resize(2, 0.0);
  history_vel_.resize(predict_steps_, 0.0);
  history_steer_.resize(predict_steps_, 0.0);

  bf_steer_ = common::BoxCarFilter(5);
  bf_x_ = common::BoxCarFilter(5);
  bf_y_ = common::BoxCarFilter(5);
  bf_heading_ = common::BoxCarFilter(5);
}

bool MpcController::CalculateAckermannDrive(
    const common::State& state, const common::Trajectory& trajectory,
    ackermann_msgs::AckermannDrive* control_cmd) {
  if (trajectory.empty()) {
    LOG_EVERY_N(WARNING, 20) << "trajectory is empty";
    Reset(state);
    return false;
  }

  Eigen::Vector2d predict_pos = state.position;
  double predict_heading = state.heading;
  double predict_v = last_control_[0];
  double predict_steer = last_control_[1];

  for (int i = 0; i < predict_steps_; ++i) {
    predict_v = history_vel_[i];
    predict_steer = history_steer_[i];
    predict_pos.x() += predict_v * std::cos(predict_heading) * ts_;
    predict_pos.y() += predict_v * std::sin(predict_heading) * ts_;
    predict_heading += predict_v / ls_ * std::tan(predict_steer) * ts_;
  }
  auto ref_index = trajectory.GetNearsetIndex(predict_pos);
  if (ref_index == trajectory.size()) {
    LOG_EVERY_N(WARNING, 20) << "reached the end of trajectory";
    Reset(state);
    return false;
  }
  auto ref_state = trajectory[ref_index];
  auto preview_state =
      trajectory[std::min<int>(ref_index + 20, trajectory.size() - 1)];

  MatrixXd matrix_a = MatrixXd::Zero(nx_, nx_);
  MatrixXd matrix_b = MatrixXd::Zero(nx_, nu_);

  double ref_v = preview_state.velocity;
  double ref_theta = ref_state.heading;
  double ref_steer = std::atan(ls_ * ref_state.kappa);

  matrix_a(0, 2) = -ref_v * std::sin(ref_theta);
  matrix_a(1, 2) = ref_v * std::cos(ref_theta);

  matrix_b(0, 0) = std::cos(ref_theta);
  matrix_b(1, 0) = std::sin(ref_theta);
  matrix_b(2, 0) = std::tan(ref_steer) / ls_;
  matrix_b(2, 1) = ref_v / (ls_ * std::cos(ref_steer) * std::cos(ref_steer));

  Discretization(matrix_a, matrix_b);

  matrix_a_piao_.block(0, 0, nx_, nx_) = matrix_ad_;
  matrix_a_piao_.block(0, nx_, nx_, nu_) = matrix_bd_;
  matrix_b_piao_.block(0, 0, nx_, nu_) = matrix_bd_;

  matrix_xmax_relative_ = matrix_xmax_;
  matrix_xmin_relative_ = matrix_xmin_;
  matrix_xmax_relative_(3, 0) -= ref_v;
  matrix_xmax_relative_(4, 0) -= ref_steer;
  matrix_xmin_relative_(3, 0) -= ref_v;
  matrix_xmin_relative_(4, 0) -= ref_steer;

  MatrixXd matrix_initial_x = MatrixXd::Zero(nx_ + nu_, 1);
  matrix_initial_x(0, 0) = predict_pos.x() - ref_state.position.x();
  matrix_initial_x(1, 0) = predict_pos.y() - ref_state.position.y();
  matrix_initial_x(2, 0) =
      common::NormalizeAngle(predict_heading - ref_state.heading);
  matrix_initial_x(3, 0) =
      std::min(std::max(last_control_[0] - ref_v, matrix_xmin_relative_(3, 0)),
               matrix_xmax_relative_(3, 0));
  matrix_initial_x(4, 0) = std::min(
      std::max(last_control_[1] - ref_steer, matrix_xmin_relative_(4, 0)),
      matrix_xmax_relative_(4, 0));
  // std::cout << matrix_initial_x << std::endl;

  MatrixXd matrix_ref = MatrixXd::Zero(nu_ + nx_, 1);
  std::vector<double> optimal_control;

  LmpcOsqpSolver lmpc_solver(
      matrix_a_piao_, matrix_b_piao_, matrix_q_, matrix_r_, matrix_initial_x,
      matrix_umin_, matrix_umax_, matrix_xmin_relative_, matrix_xmax_relative_,
      matrix_ref, horizon_, 2000, 1e-6);

  if (!lmpc_solver.Solve(&optimal_control)) {
    SetStopCommand(control_cmd);
    return false;
  }

  double optimal_speed = std::min(last_control_[0] + optimal_control[0], ref_v);
  double optimal_accel = optimal_control[0];
  double optimal_steer = last_control_[1] + optimal_control[1];
  double optimal_steer_rate = optimal_control[1];

  control_cmd->acceleration = optimal_control[0] / ts_;
  // control_cmd->acceleration = 0.0;
  control_cmd->speed = optimal_speed;
  control_cmd->jerk = 0.0;
  control_cmd->steering_angle = bf_steer_.Update(optimal_steer);
  control_cmd->steering_angle_velocity = optimal_steer_rate;

  last_control_[0] = optimal_speed;
  last_control_[1] = optimal_steer;
  LOG(WARNING) << "ref speed: " << ref_v << ", speed: " << optimal_speed
               << ", steer: " << optimal_steer
               << ", acc: " << optimal_control[0] / ts_;
  RecordControlCommand(optimal_speed, optimal_steer);

  return true;
}

void MpcController::Discretization(const Eigen::MatrixXd& matrix_a,
                                   const Eigen::MatrixXd& matrix_b) {
  MatrixXd I = MatrixXd::Identity(nx_, nx_);
  matrix_ad_ = (I - ts_ * matrix_a / 2).inverse() * (I + ts_ * matrix_a / 2);
  matrix_bd_ = ts_ * matrix_b;
}

void MpcController::Reset(const common::State& state) {
  last_control_[0] = state.velocity;
  last_control_[1] = state.steer;
  RecordControlCommand(0.0, 0.0);
}

void MpcController::RecordControlCommand(const double speed,
                                         const double steer) {
  history_vel_.push_back(speed);
  history_steer_.push_back(steer);
  if (history_vel_.size() > predict_steps_) {
    history_vel_.pop_front();
    history_steer_.pop_front();
  }
}

void MpcController::SetStopCommand(
    ackermann_msgs::AckermannDrive* control_cmd) {
  control_cmd->acceleration = 0.0;
  control_cmd->jerk = 0.0;
  control_cmd->speed = 0.0;
  control_cmd->steering_angle = 0.0;
  control_cmd->steering_angle_velocity = 0.0;
}
}  // namespace simulation
}  // namespace planning
