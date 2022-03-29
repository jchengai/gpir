/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Dense>

#include "common/smoothing/spline1d.h"
#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/st_plan/st_node.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

struct StPoint {
  double s_l;
  double s_u;
  double t;

  StPoint(const double s_l, const double s_u, const double t)
      : s_l(s_l), s_u(s_u), t(t) {}
};

class StGraph {
 public:
  StGraph() = default;
  StGraph(const Eigen::Vector3d& init_s) : init_s_(init_s) {}

  void SetInitialState(const Eigen::Vector3d& init_s) { init_s_ = init_s; }
  void SetReferenceSpeed(const double ref_v) const;

  void BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                    const GPPath& gp_path);

  bool TopKSearch(const int k);

  bool SearchWithLocalTruncation(const int k, std::vector<StNode>* result);

  bool GenerateInitialSpeedProfile(const GPPath& gp_path);

  bool IsTrajectoryFeasible(const GPPath& gp_path, vector_Eigen3d* frenet_s);

  bool UpdateSpeedProfile(const GPPath& gp_path);

  bool OptimizeTest();

  void GenerateTrajectory(const ReferenceLine& reference_line,
                          const GPPath& gp_path,
                          common::Trajectory* trajectory);

  void VisualizeStGraph();

  void SaveSnapShot(const std::string& path);

  const OccupancyMap& grid_map() const { return sdf_->occupancy_map(); }

 private: 
  void GetObstacleBlockSegment(
      const Obstacle& obstacle, const GPPath& gp_path,
      std::vector<std::vector<StPoint>>* st_block_segment);

  void GetFrenetState(const double t, Eigen::Vector3d* s);

 private:
  Eigen::Vector3d init_s_;
  double stamp_now_ = 0.0;
  double ego_half_length_ = 0.0;
  double safety_margin_ = 0.0;
  double a_max_ = 2.0;
  double a_min_ = -4.0;
  const double lat_a_max_ = 4.0;

  double max_arc_length_ = 0.0;

  std::unique_ptr<SignedDistanceField2D> sdf_;
  std::vector<std::vector<std::vector<StPoint>>> st_block_segments_;
  std::vector<std::vector<std::unique_ptr<StNode>>> search_tree_;

  std::vector<StNode> st_nodes_;

  const double step_length_ = 0.2;
  std::vector<double> t_knots_;
  common::Spline1d st_spline_;
};

}  // namespace planning
