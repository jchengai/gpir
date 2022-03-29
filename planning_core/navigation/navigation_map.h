/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <unordered_set>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "common/utils/timer.h"
#include "hdmap/routing/full_route.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/navigation/route_sequence.h"
#include "planning_core/planning_common/behavior.h"
#include "planning_core/planning_common/data_frame.h"

namespace planning {

class NavigationMap {
 public:
  NavigationMap() = default;
  NavigationMap(const hdmap::FullRoute fullroute)
      : full_route_(new hdmap::FullRoute(fullroute)) {}

  void Init();

  void Update(std::shared_ptr<DataFrame> data_frame);

  bool HasActiveTask();

  bool RandomlyUpdateRoute();

  bool SuggestLaneChange(const hdmap::LaneSegmentBehavior type);

  bool CreateTask(const geometry_msgs::PoseStamped& goal_pose);

  bool CreateTask(const Eigen::Vector2d& goal_pos, const double goal_heading);

  bool UpdateReferenceLine();

  void SetPlannerLCFeedback(bool is_planner_lc_ok);

  void AdjustReferenceSpeed(const double delta) { adjust_speed_ += delta; }

  void RandomlyAddVirtualObstacles(const int num = 1) {
    add_num_ = num;
    add_virtual_obstacles_ = true;
  }

  Eigen::Vector2d GetPoint(const double s);

  inline const common::State& ego_state() const { return data_frame_->state; }
  inline const ReferenceLine& reference_line() const { return reference_line_; }
  inline const double reference_speed() const { return refernce_speed_; }
  inline const common::Trajectory& trajectory() const { return trajectory_; }
  inline common::Trajectory* mutable_trajectory() { return &trajectory_; }
  inline const std::vector<ReferenceLine>& reference_lines() const {
    return reference_lines_;
  }
  inline const std::vector<Obstacle>& obstacles() const {
    return data_frame_->obstacles;
  }
  inline const std::vector<Eigen::Vector2d>& virtual_obstacles() const {
    return virtual_obstacles_;
  }

 protected:
  bool SelectRouteSequence(const common::State& state);

  void GetReferenceWaypoints(const common::State& state,
                             const RouteSequence& route_sequence,
                             const double forward, const double backward,
                             std::vector<hdmap::WayPoint>* waypoints,
                             std::vector<int>* lane_id_list);

  void GenerateWaypoints(const common::State& state, const double forward,
                         const double backward, int target_lane,
                         int forward_lane_hint, int backward_lane_hint,
                         std::vector<hdmap::WayPoint>* waypoints,
                         std::vector<int>* lane_id_list);

  bool UpdateRouteSequence(RouteSequence* route_sequence);
  void UpdateLaneChangeStatus();
  void UpdateVirtualObstacles();

  std::vector<RouteSequence*> GetRouteCandidate();
  void AddLaneToRouteSequence(
      const int lane_id, RouteSequence* route_sequence,
      hdmap::LaneSegmentBehavior type = hdmap::LaneSegmentBehavior::kKeep);
  void PublishReferenceLine();
  void PublishRouteSequence();
  void PublishVirtualObstacles();

 private:
  ros::Publisher reference_line_pub_;
  ros::Publisher route_sequence_pub_;
  ros::Publisher virtual_obstacle_pub_;

  double refernce_speed_ = 0.0;
  double adjust_speed_ = 0.0;

  ReferenceLine reference_line_;
  std::vector<ReferenceLine> reference_lines_;
  common::Trajectory trajectory_;

  bool planner_lc_ok_ = false;
  LCStatus lc_status_ = LCStatus::kIdle;
  common::Timer lc_timmer_;

  std::shared_ptr<DataFrame> data_frame_ = nullptr;
  std::unique_ptr<hdmap::FullRoute> full_route_ = nullptr;
  std::unique_ptr<RouteSequence> route_sequence_ = nullptr;
  std::unique_ptr<RouteSequence> route_sequence_lane_change_ = nullptr;

  // maintain position of virtual obstacles
  int add_num_ = 1;
  bool add_virtual_obstacles_ = false;
  std::vector<Eigen::Vector2d> virtual_obstacles_;
};
}  // namespace planning
