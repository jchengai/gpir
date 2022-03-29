/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <osqp/osqp.h>

#include <Eigen/Dense>
#include <vector>

namespace planning {
namespace simulation {

class LmpcOsqpSolver {
 public:
  /**
   * @brief Solver for discrete-time linear model predictive control
   * problem.Sacrifice some performances to obtain readability and
   * maintainability
   * @param matrix_a The system dynamic matrix
   * @param matrix_b The control matrix
   * @param matrix_q The cost matrix for control state
   * @param matrix_lower The lower bound control constrain matrix
   * @param matrix_upper The upper bound control constrain matrix
   * @param matrix_initial_state The initial state matrix
   * @param max_iter The maximum iterations
   */
  LmpcOsqpSolver(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x,
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int horizon,
                 const int max_iter = 2000, const double eps_abs = 1e-6);

  /**
   * @brief solve QP problem 1/2*xTQx + qTx, Q is hessian matrix and q is
   * gradient
   * @return first control variable in predict sequence
   */
  bool Solve(std::vector<double> *control);

 protected:
  /**
   * @brief Convert dense matrix(Eigen::Matrixd) to csc matrix,
   * @param dense_matrix Eigen dense matrixd
   * @return matrix store in csc form (use malloc, need free)
   */
  // csc *ConvertMatirxToCSC(const Eigen::MatrixXd &dense_matrix);

  void CaculateHessian();

  void CaculateGradient();

  void CaculateConstrains();

  void CaculateBounds();

 private:
  // Discription of the lmpc problem
  Eigen::MatrixXd matrix_a_;
  Eigen::MatrixXd matrix_b_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_initial_x_;
  Eigen::MatrixXd matrix_u_lower_;
  Eigen::MatrixXd matrix_u_upper_;
  Eigen::MatrixXd matrix_x_lower_;
  Eigen::MatrixXd matrix_x_upper_;
  Eigen::MatrixXd matrix_x_ref_;
  int horizon_;

  // number of states, controls, opt-variables
  int nx_;
  int nu_;
  int nv_;

  // Discription of QP problem
  Eigen::MatrixXd hessian_;
  Eigen::VectorXd gradient_;
  Eigen::MatrixXd constrians_;
  Eigen::VectorXd lowwerbound_;
  Eigen::VectorXd upperbound_;

  double eps_abs_ = 1e-6;
  int max_iteration_ = 2000;
};
}  // namespace simulation
}  // namespace planning
