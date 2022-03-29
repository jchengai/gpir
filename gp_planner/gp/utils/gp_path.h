/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "common/base/type.h"
#include "common/geometry/box2d.h"
#include "gp_planner/gp/interpolator/gp_interpolator.h"
#include "gtsam/nonlinear/Values.h"
#include "planning_core/navigation/reference_line.h"

namespace planning {

class GPPath {
 public:
  GPPath() = default;
  GPPath(const int num_nodes, const double start_s, const double delta_s,
         const double length, const double qc,
         const ReferenceLine* reference_line);

  void GetState(const double s, common::State* state) const;

  double GetCurvature(const double s) const;

  void GetInterpolateNode(const double s, Eigen::Vector3d* node) const;

  bool HasOverlapWith(const common::State& state, const double length,
                      const double width, double* s_l, double* s_u) const;

  void UpdateNodes(const gtsam::Values& values);

  void GetSamplePathPoints(const double delta_s,
                           std::vector<common::State>* samples);

  inline double MaximumArcLength() const { return end_s_; }
  inline double start_s() const { return start_s_; }
  inline vector_Eigen<Eigen::Vector3d>* mutable_nodes() { return &nodes_; }

 private:
  void GetEgoBox(const common::State& ego_state, common::Box2D* ego_box) const;

 private:
  int num_nodes_ = 0;
  double qc_ = 0.0;
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  double end_s_ = 0.0;

  GPInterpolator interpolator_;
  vector_Eigen<Eigen::Vector3d> nodes_;
  const ReferenceLine* reference_line_;
};

}  // namespace planning
