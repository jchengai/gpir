/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/controller/lmpc_osqp_solver.h"

#include <glog/logging.h>
#include <osqp/osqp.h>

#include <Eigen/Sparse>
#include <iostream>

#include "common/solver/osqp/osqp_sparse_matrix.h"

// using namespace Eigen;  // NOLINT
using Eigen::MatrixXd;
using common::DenseToCSCMatrix;

namespace planning {
namespace simulation {

LmpcOsqpSolver::LmpcOsqpSolver(
    const MatrixXd &matrix_a, const MatrixXd &matrix_b,
    const MatrixXd &matrix_q, const MatrixXd &matrix_r,
    const MatrixXd &matrix_initial_x, const MatrixXd &matrix_u_lower,
    const MatrixXd &matrix_u_upper, const MatrixXd &matrix_x_lower,
    const MatrixXd &matrix_x_upper, const MatrixXd &matrix_x_ref,
    const int horizon, const int max_iter, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      horizon_(horizon),
      max_iteration_(max_iter),
      eps_abs_(eps_abs) {
  nx_ = matrix_a_.rows();
  nu_ = matrix_b_.cols();
  nv_ = nx_ * (horizon_ + 1) + nu_ * horizon_;

  CHECK_EQ(matrix_q_.rows(), nx_);
  CHECK_EQ(matrix_r_.rows(), nu_);
  CHECK_EQ(matrix_a.rows(), matrix_b.rows());

  hessian_ = MatrixXd::Zero(nv_, nv_);
  gradient_ = MatrixXd::Zero(nv_, 1);
  constrians_ = MatrixXd::Zero(nx_ * (horizon_ + 1) + nv_, nv_);
}

void LmpcOsqpSolver::CaculateHessian() {
  for (int i = 0; i < horizon_ + 1; ++i) {
    hessian_.block(i * nx_, i * nx_, nx_, nx_) = matrix_q_;
  }
  int offset = nx_ * (horizon_ + 1);
  for (int k = 0; k < horizon_; ++k) {
    hessian_.block(offset + k * nu_, offset + k * nu_, nu_, nu_) = matrix_r_;
  }
}

void LmpcOsqpSolver::CaculateGradient() {
  // Eigen::Matrix<double, 5, 1> gradient_x = Eigen::Matrix<double, 5,
  // 1>::Zero(); for (int i = 0; i < horizon_ + 1; ++i) {
  //   gradient_.block(i * nx_, 0, nx_, 1) = gradient_x;
  // }

  gradient_ = MatrixXd::Zero(nv_, 1);
}

void LmpcOsqpSolver::CaculateConstrains() {
  Eigen::MatrixXd x_indentity = Eigen::MatrixXd::Identity(nx_, nx_);
  Eigen::MatrixXd u_indentity = Eigen::MatrixXd::Identity(nu_, nu_);

  // x0 constrains
  int offset = nx_ * (horizon_ + 1);
  constrians_.block(0, 0, nx_, nx_) = x_indentity;
  // constrians_.block(offset, 0, nx_, nx_) = x_indentity;

  // x1 ~ XN, u0 ~ uN-1 constrains
  for (int i = 1; i < horizon_ + 1; ++i) {
    // equality constrains of Xk+1 = A*Xk + B*U
    constrians_.block(i * nx_, i * nx_, nx_, nx_) = -x_indentity;
    constrians_.block(i * nx_, (i - 1) * nx_, nx_, nx_) = matrix_a_;
    constrians_.block(i * nx_, offset + (i - 1) * nu_, nx_, nu_) = matrix_b_;

    // inequality constrains
    constrians_.block(offset + i * nx_, i * nx_, nx_, nx_) = x_indentity;
    constrians_.block(2 * offset + (i - 1) * nu_, offset + (i - 1) * nu_, nu_,
                      nu_) = u_indentity;
  }
}

void LmpcOsqpSolver::CaculateBounds() {
  lowwerbound_ = MatrixXd::Zero(nx_ * (horizon_ + 1) + nv_, 1);
  upperbound_ = MatrixXd::Zero(nx_ * (horizon_ + 1) + nv_, 1);

  // equality constain
  upperbound_.block(0, 0, nx_, 1) = matrix_initial_x_;
  lowwerbound_.block(0, 0, nx_, 1) = matrix_initial_x_;

  // inequality constrain
  int offset = nx_ * (horizon_ + 1);
  for (int i = 0; i < horizon_ + 1; ++i) {
    lowwerbound_.block(offset + i * (nx_), 0, nx_, 1) = matrix_x_lower_;
    upperbound_.block(offset + i * (nx_), 0, nx_, 1) = matrix_x_upper_;
  }
  for (int k = 0; k < horizon_; ++k) {
    lowwerbound_.block(2 * offset + k * (nu_), 0, nu_, 1) = matrix_u_lower_;
    upperbound_.block(2 * offset + k * (nu_), 0, nu_, 1) = matrix_u_upper_;
  }
}

bool LmpcOsqpSolver::Solve(std::vector<double> *control) {
  CaculateConstrains();
  CaculateHessian();
  CaculateGradient();
  CaculateBounds();

  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->warm_start = true;
  settings->max_iter = max_iteration_;
  settings->eps_abs = eps_abs_;
  settings->verbose = false;

  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  Eigen::MatrixXd P = hessian_.triangularView<Eigen::Upper>();
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  DenseToCSCMatrix(P, &P_data, &P_indices, &P_indptr);

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  DenseToCSCMatrix(constrians_, &A_data, &A_indices, &A_indptr);

  data->n = nv_;
  data->m = constrians_.rows();
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = gradient_.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lowwerbound_.data();
  data->u = upperbound_.data();

  c_int exitflag = 0;
  OSQPWorkspace *workspace;
  exitflag = osqp_setup(&workspace, data, settings);
  osqp_solve(workspace);

  auto status = workspace->info->status_val;

  bool problem_solved = true;

  if (status < 0 || (status != 1 && status != 2)) {
    LOG(ERROR) << "failed optimization status:\t" << workspace->info->status;
    problem_solved = false;
  } else if (workspace->solution == nullptr) {
    LOG(ERROR) << "The solution from OSQP is nullptr";
    problem_solved = false;
  } else if (std::isnan(workspace->solution->x[nx_ * (horizon_ + 1)])) {
    LOG(ERROR) << "The Hessian matrix is not semi-positive defined";
    problem_solved = false;
  }

  control->clear();
  for (int i = 0; i < nu_; ++i) {
    control->emplace_back(workspace->solution->x[nx_ * (horizon_ + 1) + i]);
  }

  osqp_cleanup(workspace);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return problem_solved;
}
}  // namespace simulation
}  // namespace planning
