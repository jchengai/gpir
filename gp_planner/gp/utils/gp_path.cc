/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/utils/gp_path.h"

#include "gtsam/nonlinear/Symbol.h"
#include "planning_core/planning_common/vehicle_info.h"

namespace planning {

GPPath::GPPath(const int num_nodes, const double start_s, const double delta_s,
               const double length, const double qc,
               const ReferenceLine* reference_line)
    : num_nodes_(num_nodes),
      start_s_(start_s),
      delta_s_(delta_s),
      end_s_(start_s + length),
      qc_(qc),
      reference_line_(reference_line),
      interpolator_(delta_s, qc) {}

void GPPath::GetState(const double s, common::State* state) const {
  Eigen::Vector3d d;
  GetInterpolateNode(s, &d);
  reference_line_->FrenetToState(s, d, state);
  state->debug = d;
}

double GPPath::GetCurvature(const double s) const {
  double kappa_r, dkappa_r;
  reference_line_->GetCurvature(s, &kappa_r, &dkappa_r);
  Eigen::Vector3d node;
  GetInterpolateNode(s, &node);
  const double one_minus_kappa_rd = 1 - kappa_r * node(0);
  const double one_minus_kappa_rd_inv = 1.0 / one_minus_kappa_rd;
  const double theta = std::atan2(node(1), one_minus_kappa_rd);

  const double tan_theta = node(1) / one_minus_kappa_rd;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double cos_theta_sqr = cos_theta * cos_theta;

  const double kappa =
      ((node(2) - (dkappa_r * node(0) + kappa_r * node(1)) * tan_theta) *
           cos_theta_sqr * one_minus_kappa_rd_inv +
       kappa_r) *
      cos_theta * one_minus_kappa_rd_inv;
  return kappa;
}

void GPPath::GetInterpolateNode(const double s, Eigen::Vector3d* node) const {
  int index = std::max(
      std::min(static_cast<int>((s - start_s_) / delta_s_), num_nodes_ - 2), 0);
  interpolator_.Interpolate(nodes_[index], nodes_[index + 1],
                            s - start_s_ - index * delta_s_, node);
}

bool GPPath::HasOverlapWith(const common::State& state, const double length,
                            const double width, double* s_l,
                            double* s_u) const {
  static const VehicleParam& ego_param =
      VehicleInfo::Instance().vehicle_param();
  static double ego_half_length = ego_param.length / 2;

  double inital_s = reference_line_->GetArcLength(state.position);

  common::State ego_state;
  GetState(inital_s, &ego_state);
  common::Box2D ego_box;
  GetEgoBox(ego_state, &ego_box);
  common::Box2D obs_box(state.position, length, width * 1.2, state.heading);

  if (!ego_box.HasOverlapWith(obs_box)) {
    *s_l = 0.0;
    *s_u = 0.0;
    return false;
  }

  constexpr double kStepLength = 1;
  double forward_s = inital_s + kStepLength;
  double backward_s = inital_s - kStepLength;

  while (forward_s <= end_s_) {
    GetState(forward_s, &ego_state);
    GetEgoBox(ego_state, &ego_box);
    if (!ego_box.HasOverlapWith(obs_box)) break;
    forward_s += kStepLength;
  }
  while (backward_s >= start_s_) {
    GetState(backward_s, &ego_state);
    GetEgoBox(ego_state, &ego_box);
    if (!ego_box.HasOverlapWith(obs_box)) break;
    backward_s -= kStepLength;
  }

  *s_l = backward_s + kStepLength / 2.0;  
  *s_u = forward_s - kStepLength / 2.0;   

  return true;
}

void GPPath::GetEgoBox(const common::State& ego_state,
                       common::Box2D* ego_box) const {
  static const VehicleParam& ego_param =
      VehicleInfo::Instance().vehicle_param();
  *ego_box = common::Box2D(
      ego_state.position + Eigen::Vector2d(std::cos(ego_state.heading),
                                           std::sin(ego_state.heading)) *
                               ego_param.rear_axle_to_center,
      ego_param.length, ego_param.width * 1.2, ego_state.heading);
}

void GPPath::UpdateNodes(const gtsam::Values& values) {
  for (int i = 0; i < nodes_.size(); ++i) {
    nodes_[i] = values.at<gtsam::Vector3>(gtsam::Symbol('x', i));
  }
}

void GPPath::GetSamplePathPoints(const double delta_s,
                                 std::vector<common::State>* samples) {
  if (!samples->empty()) samples->clear();
  common::State state;
  for (double s = start_s_; s <= end_s_; s += delta_s) {
    GetState(s, &state);
    samples->emplace_back(state);
  }
}
}  // namespace planning
