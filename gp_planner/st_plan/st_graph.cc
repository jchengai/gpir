/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/st_plan/st_graph.h"

#include <glog/logging.h>
#include <omp.h>

#include <fstream>
#include <random>

#include "common/frenet/frenet_state.h"
#include "common/frenet/frenet_transform.h"
#include "common/smoothing/osqp_spline1d_solver.h"
#include "common/utils/timer.h"

namespace planning {

void StGraph::BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                           const GPPath& gp_path) {
  max_arc_length_ = gp_path.MaximumArcLength();

  int size = dynamic_obstacles.size();
  st_block_segments_.resize(size);

  omp_set_num_threads(size);
  {
#pragma omp parallel for
    for (int i = 0; i < size; ++i) {
      GetObstacleBlockSegment(dynamic_obstacles[i], gp_path,
                              &st_block_segments_[i]);
    }
  }

  OccupancyMap grid_map;
  grid_map.set_origin({0.0, init_s_[0]});
  grid_map.set_resolution({0.1, 0.1});
  grid_map.set_cell_number({82, 1500});

  vector_Eigen<Eigen::Vector2d> corners;
  corners.resize(4);

  for (const auto& st_block_segment : st_block_segments_) {
    if (st_block_segment.empty()) continue;
    for (const auto& st_points : st_block_segment) {
      if (st_points.empty()) continue;
      if (st_points.size() == 1) {
        corners[0] = Eigen::Vector2d(st_points[0].t, st_points[0].s_l);
        corners[1] = Eigen::Vector2d(st_points[0].t, st_points[0].s_u);
        corners[2] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_u);
        corners[3] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_l);
        grid_map.FillConvexPoly(corners);
        continue;
      }
      for (size_t i = 0; i < st_points.size() - 1; ++i) {
        corners[0] = Eigen::Vector2d(st_points[i].t, st_points[i].s_l);
        corners[1] = Eigen::Vector2d(st_points[i].t, st_points[i].s_u);
        corners[2] = Eigen::Vector2d(st_points[i + 1].t, st_points[i + 1].s_u);
        corners[3] = Eigen::Vector2d(st_points[i + 1].t, st_points[i + 1].s_l);
        grid_map.FillConvexPoly(corners);
      }
    }
  }

  sdf_ = std::make_unique<SignedDistanceField2D>(std::move(grid_map));
  sdf_->UpdateVerticalSDF();

  // cv::imshow("st", sdf_->occupancy_map().BinaryImage());
  // cv::imshow("st_sdf", sdf_->esdf().ImageSec());
  // cv::waitKey(0);
}

void StGraph::GetObstacleBlockSegment(
    const Obstacle& obstacle, const GPPath& gp_path,
    std::vector<std::vector<StPoint>>* st_block_segment) {
  const double length = obstacle.length();
  const double width = obstacle.width();
  double s_l = 0.0, s_u = 0.0;

  bool extend_from_previous = false;
  std::vector<StPoint>* current_seg = nullptr;
  for (const auto& future_point : obstacle.prediction()) {
    if (gp_path.HasOverlapWith(future_point, length, width, &s_l, &s_u)) {
      if (!extend_from_previous) {
        st_block_segment->emplace_back();
        current_seg = &st_block_segment->back();
        extend_from_previous = true;
      }
      current_seg->emplace_back(
          StPoint(s_l, s_u, future_point.stamp - stamp_now_));
    } else {
      extend_from_previous = false;
    }
  }
}

void StGraph::SetReferenceSpeed(const double ref_v) const {
  StNode::SetReferenceSpeed(ref_v);
}

bool StGraph::SearchWithLocalTruncation(const int k,
                                        std::vector<StNode>* result) {
  CHECK(k % 2 == 1) << "k needs to be an odd number, while k is " << k;

  int num_a_per_side = static_cast<int>(k / 2.0);
  std::vector<double> discrete_a;
  discrete_a.reserve(k);
  discrete_a.emplace_back(0.0);
  for (int i = 0; i < num_a_per_side; ++i) {
    discrete_a.emplace_back(a_max_ * (i + 1) / num_a_per_side);
    discrete_a.emplace_back(a_min_ * (i + 1) / num_a_per_side);
  }

  StNodeWeights weight;
  // weight.control = 0.5;
  weight.obstacle = 10;
  weight.ref_v = 3;
  StNode::SetWeights(weight);

  std::unique_ptr<StNode> inital_node =
      std::make_unique<StNode>(init_s_[0], init_s_[1], init_s_[2]);
  search_tree_.resize(9);
  search_tree_[0].emplace_back(std::move(inital_node));

  // LOG(INFO) << "[search tree] init velocity: " << init_s_[1];
  // LOG(INFO) << "[search tree] reference velocity: "
  // << StNode::reference_speed();

  TIC;
  double t_expand = 0.0;
  double t_sort = 0.0;
  double t_compare = 0.0;
  double kEpsilon = 0.1;

  for (int i = 0; i < 8; ++i) {
    std::vector<std::unique_ptr<StNode>> cache;

    for (int j = 0; j < search_tree_[i].size(); ++j) {
      for (const auto& a : discrete_a) {
        auto next_node = search_tree_[i][j]->Forward(1.0, a);
        if (next_node->v < 0) continue;  // TODO: can optimize
        for (int k = 1; k <= 5; ++k) {
          next_node->CalObstacleCost(sdf_->SignedDistance(
              Eigen::Vector2d(search_tree_[i][j]->t + k / 5.0,
                              search_tree_[i][j]->GetDistance(k / 5.0, a))));
        }
        if (next_node->cost < 1e9) {
          cache.emplace_back(std::move(next_node));
        }
      }
    }
    std::sort(cache.begin(), cache.end(),
              [](const std::unique_ptr<StNode>& n1,
                 const std::unique_ptr<StNode>& n2) { return n1->s < n2->s; });

    if (cache.empty()) {
      LOG(ERROR) << "cannot find valid path";
      result = nullptr;
      return false;
    }

    int min_index = 0;
    int min_cost = cache[0]->cost;
    double start_s = cache[0]->s;
    bool is_new_group = false;

    for (int j = 0; j < cache.size(); ++j) {
      if (cache[j]->s - start_s <= kEpsilon) {
        if (cache[j]->cost < min_cost) {
          min_cost = cache[j]->cost;
          min_index = j;
        }
      } else {
        is_new_group = true;
      }

      if (is_new_group || j == cache.size() - 1) {
        start_s = cache[j]->s;
        search_tree_[i + 1].emplace_back(std::move(cache[min_index]));
        min_index = j + 1;
        is_new_group = false;
      }
    }

    kEpsilon *= 1.3;
  }
  std::sort(
      search_tree_.back().begin(), search_tree_.back().end(),
      [](const std::unique_ptr<StNode>& n1, const std::unique_ptr<StNode>& n2) {
        return n1->cost < n2->cost;
      });

  // extract answer
  const StNode* current_node = search_tree_.back().front().get();
  st_nodes_.emplace_back(*current_node);
  while (current_node->parent != nullptr) {
    current_node = current_node->parent;
    st_nodes_.emplace_back(*current_node);
  }
  std::reverse(st_nodes_.begin(), st_nodes_.end());

  return true;
}

bool StGraph::GenerateInitialSpeedProfile(const GPPath& gp_path) {
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  double lb, ub;
  const auto& grid_map = sdf_->occupancy_map();
  for (const auto& node : st_nodes_) {
    t_knots_.emplace_back(node.t);
    grid_map.FindVerticalBoundary(node.t, node.s, &lb, &ub);
    lbs.emplace_back(lb);
    ubs.emplace_back(ub);
    refs.emplace_back(node.s);
  }

  std::vector<double> t_samples, v_min, v_max, a_max, a_min;
  for (double t = t_knots_.front() + step_length_; t < t_knots_.back();
       t += step_length_) {
    int index = static_cast<int>(t);
    double delta = t - st_nodes_[index].t;
    double s = st_nodes_[index].s + st_nodes_[index].v * delta +
               0.5 * st_nodes_[index + 1].a * delta * delta;
    double max_abs_v =
        1.2 * std::sqrt(lat_a_max_ / std::fabs(gp_path.GetCurvature(s)));
    t_samples.emplace_back(t);
    v_min.emplace_back(-max_abs_v);
    v_max.emplace_back(max_abs_v);
    a_min.emplace_back(a_min_);
    a_max.emplace_back(a_max_);
  }

  common::OsqpSpline1dSolver solver(t_knots_, 5);
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5);
  kernel->AddSecondOrderDerivativeMatrix(5);
  kernel->AddThirdOrderDerivativeMatrix(20);
  kernel->AddReferenceLineKernelMatrix(t_knots_, refs, 20);
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint();
  constraint->AddPointConstraint(t_knots_.front(), init_s_[0]);
  constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  // constraint->AddPointSecondDerivativeConstraint(t_knots_.front(),
  // init_s_[2]);
  constraint->AddBoundary(t_knots_, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    LOG(ERROR) << "fail to optimize";
    return false;
  }

  st_spline_ = solver.spline();

  return true;
}

bool StGraph::UpdateSpeedProfile(const GPPath& gp_path) {
  TIC;
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  double lb, ub;
  const auto& grid_map = sdf_->occupancy_map();
  for (const auto& node : st_nodes_) {
    grid_map.FindVerticalBoundary(node.t, node.s, &lb, &ub);
    lbs.emplace_back(lb);
    ubs.emplace_back(ub);
  }
  for (const auto& t : t_knots_) {
    refs.emplace_back(st_spline_(t));
  }

  std::vector<double> t_samples, v_min, v_max, a_max, a_min;
  for (double t = t_knots_.front() + 0.1; t < t_knots_.back(); t += 0.1) {
    double s = st_spline_(t);
    double max_abs_v =
        std::sqrt(lat_a_max_ / std::fabs(gp_path.GetCurvature(s))) + 1;
    t_samples.emplace_back(t);
    v_min.emplace_back(-max_abs_v);
    v_max.emplace_back(max_abs_v);
  }

  common::OsqpSpline1dSolver solver(t_knots_, 5);
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5);
  // kernel->AddSecondOrderDerivativeMatrix(5);
  kernel->AddThirdOrderDerivativeMatrix(30);
  kernel->AddReferenceLineKernelMatrix(t_knots_, refs, 10);
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint();
  constraint->AddPointConstraint(t_knots_.front(), init_s_[0]);
  constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  constraint->AddPointSecondDerivativeConstraint(t_knots_.front(), init_s_[2]);
  constraint->AddBoundary(t_knots_, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    LOG(ERROR) << "fail to optimize";
    return false;
  }

  st_spline_ = solver.spline();

  return true;
}

void StGraph::GetFrenetState(const double t, Eigen::Vector3d* s) {
  s->x() = st_spline_(t);
  s->y() = st_spline_.Derivative(t);
  s->z() = st_spline_.SecondOrderDerivative(t);
}

bool StGraph::IsTrajectoryFeasible(const GPPath& gp_path,
                                   vector_Eigen3d* frenet_s) {
  frenet_s->clear();
  double lat_acc = 0.0;
  Eigen::Vector3d s, d;
  for (double t = t_knots_.front() + step_length_; t < t_knots_.back();
       t += step_length_) {
    GetFrenetState(t, &s);
    if (s(0) > max_arc_length_) break;
    gp_path.GetInterpolateNode(s(0), &d);
    lat_acc = d(2) * s(1) * s(1) + d(1) * s(2);
    if (std::fabs(lat_acc) > 1.05 * lat_a_max_) {
      printf("invalid at %f, acc: %f\n", t, lat_acc);
      frenet_s->emplace_back(s);
    }
  }
  return frenet_s->empty() ? true : false;
}

void StGraph::GenerateTrajectory(const ReferenceLine& reference_line,
                                 const GPPath& gp_path,
                                 common::Trajectory* trajectory) {
  trajectory->clear();
  const double t_final = st_nodes_.back().t;
  const double maximum_s = gp_path.MaximumArcLength();
  Eigen::Vector3d d;
  common::State state;
  for (double t = 0.0; t <= t_final; t += 0.1) {
    Eigen::Vector3d s(st_spline_(t), st_spline_.Derivative(t),
                      st_spline_.SecondOrderDerivative(t));
    if (s[0] > maximum_s) break;
    gp_path.GetInterpolateNode(s[0], &d);
    auto ref = reference_line.GetFrenetReferncePoint(s[0]);
    auto ref_point = reference_line.GetFrenetReferncePoint(s[0]);
    common::FrenetTransfrom::FrenetStateToState(common::FrenetState(s, d),
                                                ref_point, &state);
    state.stamp = t;
    state.frenet_d = d;
    state.velocity = st_spline_.Derivative(t);

    const double one_minus_kappa_rd = 1 - ref.kappa * d[0];

    const double tan_delta_theta = d[1] / one_minus_kappa_rd;
    const double delta_theta = std::atan2(d[1], one_minus_kappa_rd);
    const double cos_delta_theta = std::cos(delta_theta);
    state.frenet_s = s;
    trajectory->emplace_back(state);
  }
}
}  // namespace planning
