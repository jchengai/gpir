/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <geometry_msgs/Pose.h>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "common/base/type.h"
#include "common/frenet/frenet_state.h"
#include "common/smoothing/spline2d.h"
#include "planning_core/navigation/route_segment.h"

namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;

  inline double length() const { return paras_.back() - paras_.front(); }

  double GetAngle(const double s);

  bool GenerateReferenceLine(const std::vector<hdmap::WayPoint>& waypoints);

  double GetArcLength(const Eigen::Vector2d& position,
                      const double epsilon = 1e-3) const;

  Eigen::Vector3d GetSE2(const double s) const;

  void GetCurvature(const double s, double* kappa,
                    double* dkappa) const;
  double GetCurvature(const double s) const;

  common::FrenetReferencePoint GetFrenetReferncePoint(const double s) const;

  common::FrenetReferencePoint GetFrenetReferncePoint(
      const Eigen::Vector2d& pos) const;

  void ToFrenetState(const common::State& state,
                     common::FrenetState* frenet_state) const;

  void FrenetToCartesion(const double s, const double d,
                         Eigen::Vector2d* point) const;

  void FrenetToCartesion(const std::vector<double>& s,
                         const std::vector<double>& d,
                         vector_Eigen<Eigen::Vector2d>* points) const;

  void FrenetToState(const double s, const Eigen::Vector3d& d,
                     common::State* state) const;

  common::FrenetPoint GetProjection(const Eigen::Vector2d& pos) const;

  common::FrenetPoint GetRoughProjection(const Eigen::Vector2d& pos) const;

  geometry_msgs::Pose GetRosPose(const double s) const;

  inline const std::vector<int>& lane_id_list() const { return lane_id_list_; }
  inline std::vector<int>* mutable_lane_id_list() { return &lane_id_list_; }

  void set_behavior(const hdmap::LaneSegmentBehavior type) { behavior_ = type; }
  hdmap::LaneSegmentBehavior behavior() const { return behavior_; }

  void print();

 protected:
  void GetFrenetPoint(const Eigen::Vector2d& pos, const double s,
                      common::FrenetPoint* frenet_point) const;

 private:
  double max_interval_ = 10;
  Eigen::Vector2d origin_;

  std::vector<double> knots_;
  std::vector<double> paras_;
  vector_Eigen<Eigen::Vector2d> refs_;
  std::vector<int> lane_id_list_;

  common::Spline2d spline_;

  hdmap::LaneSegmentBehavior behavior_ = hdmap::LaneSegmentBehavior::kKeep;
};

}  // namespace planning
