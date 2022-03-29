/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "planning_core/navigation/reference_line.h"

namespace planning {

class GPIncrementalPathPlanner {
 public:
  GPIncrementalPathPlanner() = default;
  GPIncrementalPathPlanner(std::shared_ptr<SignedDistanceField2D> sdf)
      : sdf_(sdf){};

  bool GenerateInitialGPPath(const ReferenceLine& reference_line,
                             const common::FrenetState& initial_state,
                             const double length,
                             const std::vector<double>& obstacle_location_hint,
                             GPPath* gp_path);

  bool TmpTest(const ReferenceLine& reference_line,
               const common::FrenetState& initial_state, const double length,
               const std::vector<double>& obstacle_location_hint,
               GPPath* gp_path);

  bool UpdateGPPath(const ReferenceLine& reference_line,
                    const vector_Eigen3d& frenet_s, GPPath* gp_path);

  bool UpdateGPPathNonIncremental(const ReferenceLine& reference_line,
                                  const vector_Eigen3d& frenet_s,
                                  GPPath* gp_path);

  inline void set_sdf(std::shared_ptr<SignedDistanceField2D> sdf) {
    sdf_ = sdf;
  }
  inline void set_enable_curvature_constraint(const bool option) {
    enable_curvature_constraint_ = option;
  }
  inline void set_enable_incremental_refinement(const bool option) {
    enable_incremental_refinemnt_ = option;
  }

 protected:
  bool DecideInitialPathBoundary(
      const Eigen::Vector2d& init_pos, const double init_angle,
      const std::vector<double>& obstacle_location_hint,
      const std::vector<std::vector<std::pair<double, double>>>& boundaries,
      std::vector<double>* lb, std::vector<double>* ub);

  int FindLocationIndex(const double s);

 private:
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values map_result_;

  int num_of_nodes_ = 21;
  double ego_width_ = 2.1;
  double ego_half_width_ = 1.0;
  const double kappa_limit_ = 0.2;
  double interval_ = 0.0;
  std::vector<double> node_locations_;
  std::shared_ptr<SignedDistanceField2D> sdf_;

  // options
  bool enable_curvature_constraint_ = true;
  bool enable_incremental_refinemnt_ = true;
};
}  // namespace planning
