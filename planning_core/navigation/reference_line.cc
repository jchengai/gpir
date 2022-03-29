/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/navigation/reference_line.h"

#include <glog/logging.h>

#include "common/frenet/frenet_transform.h"
#include "common/smoothing/osqp_spline2d_solver.h"
#include "common/utils/timer.h"
#include "hdmap/road_network/lane_map.h"
#include "tf/tf.h"

namespace planning {

using common::OsqpSpline2dSolver;

bool ReferenceLine::GenerateReferenceLine(
    const std::vector<hdmap::WayPoint>& waypoints) {
  refs_.clear();
  paras_.clear();
  knots_.clear();

  int size = waypoints.size();
  refs_.reserve(size);
  paras_.reserve(size);

  origin_ = waypoints.front().point;
  for (const auto& waypoint : waypoints) {
    refs_.emplace_back(waypoint.point - origin_);
    paras_.emplace_back(waypoint.s);
  }

  const double start_para = waypoints.front().s;
  const double total_length = waypoints.back().s;
  int num_of_segments = std::ceil(total_length / max_interval_);
  const double interval = total_length / num_of_segments;
  for (int i = 0; i <= num_of_segments; ++i) {
    knots_.emplace_back(start_para + i * interval);
  }

  std::unique_ptr<OsqpSpline2dSolver> spline2d_solver =
      std::make_unique<OsqpSpline2dSolver>(knots_, 5);

  auto mutable_kernel = spline2d_solver->mutable_kernel();
  auto mutable_constraint = spline2d_solver->mutable_constraint();

  mutable_kernel->Add2dReferenceLineKernelMatrix(paras_, refs_, 20);
  mutable_kernel->AddRegularization(1e-5);
  mutable_kernel->Add2dSecondOrderDerivativeMatrix(200);
  mutable_kernel->Add2dThirdOrderDerivativeMatrix(1000);
  mutable_constraint->Add2dThirdDerivativeSmoothConstraint();

  if (!spline2d_solver->Solve()) {
    LOG(ERROR) << "fail to solve spline lane problem";
    return false;
  }

  spline_ = spline2d_solver->spline();
  return true;
}

double ReferenceLine::GetArcLength(const Eigen::Vector2d& position,
                                   const double epsilon) const {
  Eigen::Vector2d local_pos = position - origin_;

  double lb = paras_.front();
  double ub = paras_.back();
  double step = (ub - lb) / 2.0;
  double mid = lb + step;

  double l_value = (spline_.pos(lb) - local_pos).squaredNorm();
  double r_value = (spline_.pos(ub) - local_pos).squaredNorm();
  double m_value = (spline_.pos(mid) - local_pos).squaredNorm();

  while (std::fabs(ub - lb) > epsilon) {
    double min = std::min(std::min(l_value, m_value), r_value);
    step *= 0.5;
    if (min == l_value) {
      ub = mid;
      mid = lb + step;
      m_value = (spline_.pos(mid) - local_pos).squaredNorm();
      r_value = (spline_.pos(ub) - local_pos).squaredNorm();
    } else if (min == m_value) {
      lb = mid - step;
      ub = mid + step;
      l_value = (spline_.pos(lb) - local_pos).squaredNorm();
      r_value = (spline_.pos(ub) - local_pos).squaredNorm();
    } else {
      lb = mid;
      mid = ub - step;
      l_value = (spline_.pos(lb) - local_pos).squaredNorm();
      m_value = (spline_.pos(mid) - local_pos).squaredNorm();
    }
  }
  return (lb + ub) / 2;
}

Eigen::Vector3d ReferenceLine::GetSE2(const double s) const {
  Eigen::Vector3d se2;
  se2.topRows(2) = spline_.pos(s) + origin_;
  se2(2) = spline_.theta(s);
  return se2;
}

common::FrenetReferencePoint ReferenceLine::GetFrenetReferncePoint(
    const double s) const {
  common::FrenetReferencePoint ref;
  ref.s = s;
  ref.point = spline_.pos(s) + origin_;
  ref.theta = spline_.theta(s);
  spline_.GetCurvature(s, &ref.kappa, &ref.dkappa);
  return ref;
}

common::FrenetReferencePoint ReferenceLine::GetFrenetReferncePoint(
    const Eigen::Vector2d& position) const {
  return GetFrenetReferncePoint(GetArcLength(position));
}

void ReferenceLine::ToFrenetState(const common::State& state,
                                  common::FrenetState* frenet_state) const {
  auto reference_point = GetFrenetReferncePoint(state.position);
  common::FrenetTransfrom::StateToFrenetState(state, reference_point,
                                              frenet_state);
}

void ReferenceLine::GetCurvature(const double s, double* kappa,
                                 double* dkappa) const {
  spline_.GetCurvature(s, kappa, dkappa);
}

double ReferenceLine::GetCurvature(const double s) const {
  return spline_.GetCurvature(s);
}

void ReferenceLine::FrenetToCartesion(const double s, const double d,
                                      Eigen::Vector2d* point) const {
  const double theta = spline_.theta(s);
  Eigen::Vector2d normal(-std::sin(theta), std::cos(theta));
  *point = spline_.pos(s) + normal * d + origin_;
}

void ReferenceLine::FrenetToCartesion(
    const std::vector<double>& s, const std::vector<double>& d,
    vector_Eigen<Eigen::Vector2d>* points) const {
  CHECK_EQ(s.size(), d.size());
  points->resize(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    FrenetToCartesion(s[i], d[i], &points->at(i));
  }
}

void ReferenceLine::FrenetToState(const double s, const Eigen::Vector3d& d,
                                  common::State* state) const {
  auto ref_point = GetFrenetReferncePoint(s);
  common::FrenetTransfrom::LateralFrenetStateToState(d, ref_point, state);
}

common::FrenetPoint ReferenceLine::GetProjection(
    const Eigen::Vector2d& pos) const {
  Eigen::Vector2d local_pos = pos - origin_;
  double arc_length = GetArcLength(pos);
  common::FrenetPoint frenet_point;
  GetFrenetPoint(local_pos, arc_length, &frenet_point);
  return frenet_point;
}

common::FrenetPoint ReferenceLine::GetRoughProjection(
    const Eigen::Vector2d& pos) const {
  Eigen::Vector2d local_pos = pos - origin_;
  double arc_length = GetArcLength(pos, 1.0);
  common::FrenetPoint frenet_point;
  GetFrenetPoint(local_pos, arc_length, &frenet_point);
  return frenet_point;
}

void ReferenceLine::GetFrenetPoint(const Eigen::Vector2d& pos, const double s,
                                   common::FrenetPoint* frenet_point) const {
  auto vec = pos - spline_.pos(s);
  const double theta = spline_.theta(s);
  const double cos = std::cos(theta);
  const double sin = std::sin(theta);
  frenet_point->s = vec.x() * cos + vec.y() * sin + s;
  frenet_point->d = vec.y() * cos - vec.x() * sin;
}

geometry_msgs::Pose ReferenceLine::GetRosPose(const double s) const {
  geometry_msgs::Pose pose;
  auto pos = spline_.pos(s) + origin_;
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = 0.5;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(spline_.theta(s)),
                        pose.orientation);
  return pose;
}

void ReferenceLine::print() {
  std::cout << "knots: [";
  for (const auto& k : knots_) {
    std::cout << k << ", ";
  }
  std::cout << "]\n";
}
}  // namespace planning
