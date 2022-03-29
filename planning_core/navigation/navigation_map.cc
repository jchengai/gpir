/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/navigation/navigation_map.h"

#include <glog/logging.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/graph/dijkstra.hpp"
#include "common/utils/color_map.h"
#include "common/utils/contains.hpp"
#include "common/utils/math.h"
#include "common/utils/timer.h"
#include "hdmap/hdmap.h"
#include "hdmap/road_network/lane_map.h"
#include "planning_core/planning_common/planning_visual.h"

namespace planning {

using common::RandomDouble;
using common::RandomInt;
using common::Timer;
using hdmap::LaneSegmentBehavior;

void NavigationMap::Init() {
  ros::NodeHandle node;
  reference_line_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/reference_line", 1);
  route_sequence_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/route_sequence", 1);
  virtual_obstacle_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/virtual_obstacle", 1);
}

void NavigationMap::Update(std::shared_ptr<DataFrame> data_frame) {
  data_frame_ = data_frame;
  auto route_candidate = GetRouteCandidate();
  for (auto& route : route_candidate) {
    route->Update(data_frame_->state.position);
  }
}

bool NavigationMap::HasActiveTask() {
  if (full_route_ == nullptr || route_sequence_ == nullptr) {
    return false;
  } else if (route_sequence_->arrived()) {
    return false;
  }
  return true;
}

bool NavigationMap::CreateTask(const geometry_msgs::PoseStamped& goal_pose) {
  return CreateTask(
      Eigen::Vector2d(goal_pose.pose.position.x, goal_pose.pose.position.y),
      tf::getYaw(goal_pose.pose.orientation));
}

bool NavigationMap::CreateTask(const Eigen::Vector2d& goal_pos,
                               const double goal_heading) {
  full_route_.reset(new hdmap::FullRoute());
  if (!hdmap::HdMap::GetMap().CreateFullRoute(
          data_frame_->state.position, goal_pos, data_frame_->state.heading,
          goal_heading, full_route_.get())) {
    full_route_.reset(nullptr);
    return false;
  }
  if (!SelectRouteSequence(data_frame_->state)) {
    route_sequence_.reset(nullptr);
    LOG(ERROR) << "fail to generate valid route sequence";
    return false;
  }
  return true;
}

bool NavigationMap::RandomlyUpdateRoute() {
  if (!route_sequence_) {
    route_sequence_.reset(new RouteSequence());
    int lane_id = hdmap::HdMap::NearestLane(data_frame_->state.position,
                                            data_frame_->state.heading, false);
    if (lane_id == 0) {
      LOG(ERROR) << "Cannot find matched lane, update route failed";
      return false;
    }
    AddLaneToRouteSequence(lane_id, route_sequence_.get());
    auto lane = hdmap::LaneMap::GetLane(lane_id);
    int size = lane->next_lanes().size();
    if (size > 0) {
      int next_lane_id = lane->next_lanes()[RandomInt(size)];
      AddLaneToRouteSequence(next_lane_id, route_sequence_.get());
    }
  } else {
    auto route_candidate = GetRouteCandidate();
    bool result = true;
    for (const auto& route : route_candidate) {
      result &= UpdateRouteSequence(route);
    }
    if (!result) return false;
    UpdateLaneChangeStatus();
  }
  return true;
}

std::vector<RouteSequence*> NavigationMap::GetRouteCandidate() {
  std::vector<RouteSequence*> route_candidate;
  if (route_sequence_lane_change_)
    route_candidate.emplace_back(route_sequence_lane_change_.get());
  if (route_sequence_) route_candidate.emplace_back(route_sequence_.get());
  return route_candidate;
}

bool NavigationMap::UpdateRouteSequence(RouteSequence* route_sequence) {
  if (route_sequence->approaching()) {
    int lane_id = route_sequence->back().id();
    auto lane = hdmap::LaneMap::GetLane(lane_id);
    int size = lane->next_lanes().size();
    if (size > 0) {
      int next_lane_id = lane->next_lanes()[RandomInt(size)];
      route_sequence->RemoveOldestRoute();
      AddLaneToRouteSequence(next_lane_id, route_sequence);
    }
  } else if (route_sequence->arrived()) {
    return false;
  }
  return true;
}

void NavigationMap::AddLaneToRouteSequence(const int lane_id,
                                           RouteSequence* route_sequence,
                                           hdmap::LaneSegmentBehavior type) {
  RouteSegment route_segment;
  route_segment.set_id(lane_id);
  route_segment.set_main_action(type);
  auto lane = hdmap::LaneMap::GetLane(route_segment.id());
  if (lane->HasLeftNeighbor()) {
    route_segment.add_alternative_actions(
        hdmap::LaneSegmentBehavior::kLeftChange);
    route_segment.set_left_lane_id(lane->left_lane()->id());
  }
  if (lane->HasRightNeighbor()) {
    route_segment.add_alternative_actions(
        hdmap::LaneSegmentBehavior::kRightChange);
    route_segment.set_right_lane_id(lane->right_lane()->id());
  }
  route_sequence->AddRoute(route_segment);
}

bool NavigationMap::SuggestLaneChange(const hdmap::LaneSegmentBehavior type) {
  if (!route_sequence_) {
    LOG(ERROR) << "No route found";
    return false;
  }
  if (!std_enhancement::contains(
          route_sequence_->current_route().alternative_actions(), type) ||
      !std_enhancement::contains(
          route_sequence_->next_route().alternative_actions(), type)) {
    LOG(WARNING) << "Suggested behavior type: " << ToString(type)
                 << "is not allowed.";
    return false;
  }
  LOG(INFO) << "suggest lane change: " << ToString(type);
  if (route_sequence_lane_change_ == nullptr)
    route_sequence_lane_change_.reset(new RouteSequence());
  AddLaneToRouteSequence(route_sequence_->current_route().neighbor_id(type),
                         route_sequence_lane_change_.get(), type);
  AddLaneToRouteSequence(route_sequence_->next_route().neighbor_id(type),
                         route_sequence_lane_change_.get(), type);
  lc_status_ = LCStatus::kTryLaneChange;
  lc_timmer_ = common::Timer(3000);
  return true;
}

void NavigationMap::UpdateLaneChangeStatus() {
  if (lc_status_ == LCStatus::kTryLaneChange) {
    if (lc_timmer_.timeout()) {
      LOG(WARNING) << "Give up lane change";
      route_sequence_lane_change_.reset(nullptr);
      lc_status_ = LCStatus::kIdle;
    }
  } else if (lc_status_ == LCStatus::kLaneChanging) {
    if (planner_lc_ok_) {
      if (route_sequence_lane_change_->IsWithInLane(
              data_frame_->state.position)) {
        route_sequence_ = std::move(route_sequence_lane_change_);
        route_sequence_->ChangeMainAction(hdmap::LaneSegmentBehavior::kKeep);
        route_sequence_lane_change_.reset(nullptr);
        lc_status_ = LCStatus::kIdle;
      }
    } else {
      LOG(WARNING) << "Lane Change Canceled";
      route_sequence_lane_change_.reset(nullptr);
      lc_status_ = LCStatus::kIdle;
    }
  }
}

void NavigationMap::SetPlannerLCFeedback(bool is_planner_lc_ok) {
  planner_lc_ok_ = is_planner_lc_ok;
  if (is_planner_lc_ok) {
    if (lc_status_ == LCStatus::kTryLaneChange) {
      lc_status_ = LCStatus::kLaneChanging;
    }
  }
}

bool NavigationMap::UpdateReferenceLine() {
  reference_lines_.clear();
  std::vector<hdmap::WayPoint> waypoints;
  auto route_candidate = GetRouteCandidate();

  int count = 0;
  for (const auto& route : route_candidate) {
    reference_lines_.emplace_back();
    auto ref_line = &reference_lines_.back();
    GetReferenceWaypoints(data_frame_->state, *route, 100, 20, &waypoints,
                          ref_line->mutable_lane_id_list());
    ref_line->GenerateReferenceLine(waypoints);
    ref_line->set_behavior(route->main_action());
  }

  UpdateVirtualObstacles();

  // * update reference speed
  const double lat_acc_limit = 2.0;
  double kappa, dkappa = 0.0;
  const double forward_length = std::min(
      std::max(data_frame_->state.velocity * data_frame_->state.velocity / 6.0,
               30.0),
      reference_lines_.back().length());
  const double step_length = 0.15;
  refernce_speed_ = 15.0;
  for (double s = 0; s <= forward_length; s += step_length) {
    reference_lines_.back().GetCurvature(s, &kappa, &dkappa);
    refernce_speed_ =
        std::min(refernce_speed_, std::sqrt(lat_acc_limit / std::fabs(kappa)));
  }
  refernce_speed_ = std::max(0.0, refernce_speed_ + adjust_speed_);
  LOG(INFO) << "refernce speed: " << refernce_speed_;

  PublishRouteSequence();
  PublishReferenceLine();
  return true;
}

void NavigationMap::UpdateVirtualObstacles() {
  std::set<int> valid_obstacle_index;
  for (const auto& reference_line : reference_lines_) {
    auto ego_proj = reference_line.GetProjection(data_frame_->state.position);
    for (int i = 0; i < virtual_obstacles_.size(); ++i) {
      auto obs_proj = reference_line.GetProjection(virtual_obstacles_[i]);
      if (ego_proj.s - obs_proj.s <= 5) {
        valid_obstacle_index.insert(i);
      }
    }
  }
  std::vector<Eigen::Vector2d> remaining_obstacles;
  for (const auto& idx : valid_obstacle_index) {
    remaining_obstacles.emplace_back(virtual_obstacles_[idx]);
  }
  virtual_obstacles_ = remaining_obstacles;

  if (add_virtual_obstacles_) {
    static double sign = 1.0;
    const auto& target_lane = reference_lines_.front();
    common::FrenetPoint proj =
        virtual_obstacles_.empty()
            ? target_lane.GetProjection(data_frame_->state.position)
            : target_lane.GetProjection(virtual_obstacles_.back());
    double obs_s = proj.s + 50;
    double obs_d = sign * 0.6;
    sign = -sign;
    Eigen::Vector2d obs_pos;
    for (int i = 0; i < add_num_; ++i) {
      target_lane.FrenetToCartesion(obs_s + 1.5 * i, obs_d, &obs_pos);
      virtual_obstacles_.emplace_back(obs_pos);
    }
    add_virtual_obstacles_ = false;
  }
  PublishVirtualObstacles();
}

bool NavigationMap::SelectRouteSequence(const common::State& state) {
  TIC;
  common::Dijkstra<hdmap::LaneId, hdmap::LaneIdHasher> dijkstra;

  constexpr double left_change_cost = 1.0;
  constexpr double right_change_cost = 1.0;
  constexpr double lane_keep_cost = 1.0;

  for (const auto& road : full_route_->road_segments) {
    for (const auto& lane : road.lane_segments) {
      if (lane.left_neighbor_id != (hdmap::LaneId)0 &&
          std_enhancement::contains(lane.behavior,
                                    LaneSegmentBehavior::kLeftChange)) {
        dijkstra.AddEdge(
            lane.id, lane.left_neighbor_id,
            left_change_cost + 1.0 / lane.maximum_lane_keeping_length);
      }
      if (lane.right_neighbor_id != (hdmap::LaneId)0 &&
          std_enhancement::contains(lane.behavior,
                                    LaneSegmentBehavior::kRightChange)) {
        dijkstra.AddEdge(
            lane.id, lane.right_neighbor_id,
            right_change_cost + 1.0 / lane.maximum_lane_keeping_length);
      }
      for (const auto& successor : lane.successors_id) {
        dijkstra.AddEdge(lane.id, successor, lane_keep_cost);
      }
    }
  }

  auto start_lane = hdmap::HdMap::NearestLane(state.position, state.heading);
  std::vector<hdmap::LaneId> lane_sequence;
  if (!dijkstra.FindPath(start_lane, full_route_->target_lane_id,
                         &lane_sequence)) {
    LOG(ERROR) << "cannot find route sequence";
    return false;
  }

  route_sequence_.reset(new RouteSequence());
  for (int i = 0; i < lane_sequence.size(); ++i) {
    const auto& current_lane_segment =
        *full_route_->GetLaneSegment(lane_sequence[i]);
    RouteSegment segment(current_lane_segment);

    if (i != lane_sequence.size() - 1) {
      const auto& next_lane_segment =
          *full_route_->GetLaneSegment(lane_sequence[i + 1]);
      if (lane_sequence[i] == next_lane_segment.left_neighbor_id) {
        segment.set_main_action(LaneSegmentBehavior::kRightChange);
      } else if (lane_sequence[i] == next_lane_segment.right_neighbor_id) {
        segment.set_main_action(LaneSegmentBehavior::kLeftChange);
      } else if (std_enhancement::contains(next_lane_segment.presuccessor_id,
                                           lane_sequence[i])) {
        segment.set_main_action(LaneSegmentBehavior::kKeep);
      } else {
        segment.set_main_action(LaneSegmentBehavior::kNone);
      }
    } else {
      segment.set_main_action(LaneSegmentBehavior::kKeep);
    }

    for (const auto& action : current_lane_segment.behavior) {
      if (action != segment.main_action())
        segment.add_alternative_actions(action);
    }
    // LOG(INFO) << ToString(segment.main_action());
    route_sequence_->emplace_back(segment);
  }
  TOC("NavigationMap::SelectRouteSequence");
  return true;
}

void NavigationMap::GetReferenceWaypoints(
    const common::State& state, const RouteSequence& route_sequence,
    const double forward, const double backward,
    std::vector<hdmap::WayPoint>* waypoints, std::vector<int>* lane_id_list) {
  if (!lane_id_list->empty()) lane_id_list->clear();
  if (!waypoints->empty()) waypoints->clear();

  int index = route_sequence.current_index();
  std::deque<std::vector<hdmap::WayPoint>> waypoints_segments;

  int target_index = index;
  hdmap::LaneId target_lane = route_sequence.at(index).id();

  lane_id_list->emplace_back(target_lane);

  auto init_lane = hdmap::LaneMap::GetLane(target_lane);
  const double init_length = init_lane->GetArcLength(state.position);
  double current_length = 0.0;
  std::shared_ptr<hdmap::Lane> current_lane;

  // * forward search
  current_lane = init_lane;
  current_length = std::min(init_length, current_lane->length());
  int forward_index = target_index;
  double forward_remain = forward;
  while (true) {
    if (current_length + forward_remain > current_lane->length()) {
      waypoints_segments.emplace_back(std::move(
          current_lane->way_points(current_length, current_lane->length())));
      forward_remain -= current_lane->length() - current_length;

      if (!current_lane->HasSuccessor()) break;

      hdmap::LaneId next_lane_id = current_lane->next_lanes().front();
      if (forward_index != route_sequence.size() - 1) {
        for (const auto& id : current_lane->next_lanes()) {
          if (id == route_sequence_->at(forward_index + 1).id()) {
            next_lane_id = id;
            break;
          }
        }
        ++forward_index;
      }

      lane_id_list->emplace_back(next_lane_id);
      current_length = 0.0;
      current_lane = hdmap::LaneMap::GetLane(next_lane_id);
    } else {
      waypoints_segments.emplace_back(std::move(current_lane->way_points(
          current_length, current_length + forward_remain)));
      if (waypoints_segments.back().empty()) {
        LOG(WARNING) << "1find empty: " << current_length << " - "
                     << current_length + forward_remain
                     << " while length= " << current_lane->length();
      }
      break;
    }
  }

  // * backward search
  current_lane = init_lane;
  current_length = std::max(init_length, 0.0);
  int backward_index = target_index;
  double backward_remain = backward;
  while (true) {
    if (current_length - backward_remain < 0.0) {
      waypoints_segments.emplace_front(
          std::move(current_lane->way_points(0.0, current_length)));
      if (waypoints_segments.front().empty()) {
        LOG(WARNING) << "1find empty: "
                     << "0.0 - " << current_length
                     << " while length= " << current_lane->length();
      }
      backward_remain -= current_length;

      if (!current_lane->HasPreSuccessor()) break;

      hdmap::LaneId previous_lane_id = current_lane->previous_lanes().front();
      if (backward_index != 0) {
        for (const auto& id : current_lane->previous_lanes()) {
          if (id == route_sequence.at(backward_index - 1).id()) {
            previous_lane_id = id;
            break;
          }
        }
        --backward_index;
      }

      lane_id_list->emplace_back(previous_lane_id);
      current_lane = hdmap::LaneMap::GetLane(previous_lane_id);
      current_length = current_lane->length();
    } else {
      waypoints_segments.emplace_front(std::move(current_lane->way_points(
          current_length - backward_remain, current_length)));
      if (waypoints_segments.front().empty()) {
        LOG(WARNING) << "2find empty: " << current_length - backward_remain
                     << " - " << current_length
                     << " while length= " << current_lane->length();
      }
      break;
    }
  }

  // * combine waypoints segments into one
  waypoints->clear();
  constexpr double kEpsilon = 1e-6;
  const int num_of_segments = waypoints_segments.size();
  double s_offset = 0.0;

  for (int i = 0; i < num_of_segments - 1; ++i) {
    const double s_begin = waypoints_segments[i].front().s;
    for (auto& point : waypoints_segments[i]) point.s += s_offset - s_begin;
    const auto& current_end_point = waypoints_segments[i].back().point;
    const auto& next_start_point = waypoints_segments[i + 1].front().point;
    if ((current_end_point - next_start_point).squaredNorm() <= kEpsilon) {
      waypoints->insert(waypoints->end(), waypoints_segments[i].begin(),
                        waypoints_segments[i].end() - 1);
    } else {
      waypoints->insert(waypoints->end(), waypoints_segments[i].begin(),
                        waypoints_segments[i].end());
    }
    if (waypoints->empty()) continue;
    s_offset = waypoints->back().s +
               (next_start_point - waypoints->back().point).norm();
  }
  const double s_begin = waypoints_segments[num_of_segments - 1].front().s;
  for (auto& point : waypoints_segments[num_of_segments - 1]) {
    point.s += s_offset - s_begin;
  }
  waypoints->insert(waypoints->end(),
                    waypoints_segments[num_of_segments - 1].begin(),
                    waypoints_segments[num_of_segments - 1].end());

  std::sort(lane_id_list->begin(), lane_id_list->end());
}

void NavigationMap::GenerateWaypoints(
    const common::State& state, const double forward, const double backward,
    int target_lane, int forward_lane_hint, int backward_lane_hint,
    std::vector<hdmap::WayPoint>* waypoints, std::vector<int>* lane_id_list) {
  if (!lane_id_list->empty()) lane_id_list->clear();

  std::deque<std::vector<hdmap::WayPoint>> waypoints_segments;

  lane_id_list->emplace_back(target_lane);

  auto init_lane = hdmap::LaneMap::GetLane(target_lane);
  const double init_length = init_lane->GetArcLength(state.position);
  double current_length = 0.0;
  std::shared_ptr<hdmap::Lane> current_lane;

  // * forward search
  current_lane = init_lane;
  current_length = std::min(init_length, current_lane->length());
  double forward_remain = forward;
  while (true) {
    if (current_length + forward_remain > current_lane->length()) {
      waypoints_segments.emplace_back(std::move(
          current_lane->way_points(current_length, current_lane->length())));
      forward_remain -= current_lane->length() - current_length;

      if (!current_lane->HasSuccessor()) break;

      hdmap::LaneId next_lane_id = current_lane->next_lanes().front();
      for (const auto& id : current_lane->next_lanes()) {
        if (static_cast<int>(id) == forward_lane_hint) {
          next_lane_id = id;
          break;
        }
      }

      lane_id_list->emplace_back(next_lane_id);
      current_length = 0.0;
      current_lane = hdmap::LaneMap::GetLane(next_lane_id);
    } else {
      waypoints_segments.emplace_back(std::move(current_lane->way_points(
          current_length, current_length + forward_remain)));
      if (waypoints_segments.back().empty()) {
        LOG(WARNING) << "1find empty: " << current_length << " - "
                     << current_length + forward_remain
                     << " while length= " << current_lane->length();
      }
      break;
    }
  }

  // * backward search
  current_lane = init_lane;
  current_length = std::max(init_length, 0.0);
  LOG(INFO) << "init_length: " << current_length;
  double backward_remain = backward;
  while (true) {
    if (current_length - backward_remain < 0.0) {
      waypoints_segments.emplace_front(
          std::move(current_lane->way_points(0.0, current_length)));
      if (waypoints_segments.front().empty()) {
        LOG(WARNING) << "1find empty: "
                     << "0.0 - " << current_length
                     << " while length= " << current_lane->length();
      }
      backward_remain -= current_length;

      if (!current_lane->HasPreSuccessor()) break;

      hdmap::LaneId previous_lane_id = current_lane->previous_lanes().front();
      for (const auto& id : current_lane->previous_lanes()) {
        if (static_cast<int>(id) == backward_lane_hint) {
          previous_lane_id = id;
          break;
        }
      }

      lane_id_list->emplace_back(previous_lane_id);
      current_lane = hdmap::LaneMap::GetLane(previous_lane_id);
      current_length = current_lane->length();
    } else {
      waypoints_segments.emplace_front(std::move(current_lane->way_points(
          current_length - backward_remain, current_length)));
      if (waypoints_segments.front().empty()) {
        LOG(WARNING) << "2find empty: " << current_length - backward_remain
                     << " - " << current_length
                     << " while length= " << current_lane->length();
      }
      break;
    }
  }

  // * combine waypoints segments into one
  waypoints->clear();
  LOG(INFO) << "size: " << waypoints_segments.size();
  constexpr double kEpsilon = 1e-6;
  const int num_of_segments = waypoints_segments.size();
  double s_offset = 0.0;

  for (int i = 0; i < num_of_segments - 1; ++i) {
    const double s_begin = waypoints_segments[i].front().s;
    for (auto& point : waypoints_segments[i]) point.s += s_offset - s_begin;
    const auto& current_end_point = waypoints_segments[i].back().point;
    const auto& next_start_point = waypoints_segments[i + 1].front().point;
    if ((current_end_point - next_start_point).squaredNorm() <= kEpsilon) {
      waypoints->insert(waypoints->end(), waypoints_segments[i].begin(),
                        waypoints_segments[i].end() - 1);
    } else {
      waypoints->insert(waypoints->end(), waypoints_segments[i].begin(),
                        waypoints_segments[i].end());
    }
    if (waypoints->empty()) continue;
    s_offset = waypoints->back().s +
               (next_start_point - waypoints->back().point).norm();
  }
  const double s_begin = waypoints_segments[num_of_segments - 1].front().s;
  for (auto& point : waypoints_segments[num_of_segments - 1]) {
    point.s += s_offset - s_begin;
  }
  waypoints->insert(waypoints->end(),
                    waypoints_segments[num_of_segments - 1].begin(),
                    waypoints_segments[num_of_segments - 1].end());

  std::sort(lane_id_list->begin(), lane_id_list->end());
}

void NavigationMap::PublishRouteSequence() {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.2;
  marker.scale.y = 0.5;
  marker.scale.z = 2.0;
  marker.points.resize(2);
  marker.lifetime = ros::Duration(1);

  int id = 0;
  int current_index = route_sequence_->current_index();
  for (int i = 0; i < route_sequence_->size(); ++i) {
    marker.id = id++;

    Eigen::Vector2d start, end;
    auto lane = hdmap::LaneMap::GetLane(route_sequence_->at(i).id());
    start = lane->way_points().front().point;
    if (route_sequence_->at(i).main_action() == LaneSegmentBehavior::kKeep) {
      end = lane->way_points().back().point;
    } else {
      auto next_lane = hdmap::LaneMap::GetLane(route_sequence_->at(i + 1).id());
      end = next_lane->way_points().front().point;
    }
    if (current_index == i) {
      marker.color = common::ColorMap::at(common::Color::kRed).toRosMsg();
    } else {
      marker.color =
          common::ColorMap::at(common::Color::kFroestGreen).toRosMsg();
    }
    const auto& waypoints = lane->way_points();
    marker.points[0].x = waypoints.front().point.x();
    marker.points[0].y = waypoints.front().point.y();
    marker.points[0].z = 0.2;
    marker.points[1].x = waypoints.back().point.x();
    marker.points[1].y = waypoints.back().point.y();
    marker.points[0].z = 0.2;
    markers.markers.emplace_back(marker);
  }
  route_sequence_pub_.publish(markers);
}

void NavigationMap::PublishReferenceLine() {
  visualization_msgs::MarkerArray markers;

  constexpr double sample_dis = 2.0;
  int id_count = 0;
  for (const auto reference_line : reference_lines_) {
    int num_of_points = std::ceil(reference_line.length() / sample_dis);
    double sample_interval = reference_line.length() / num_of_points;

    visualization_msgs::Marker maker;
    maker.type = visualization_msgs::Marker::LINE_STRIP;
    maker.action = visualization_msgs::Marker::MODIFY;
    maker.id = id_count++;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.scale.x = 1.0;
    maker.scale.y = 0.0;
    maker.scale.z = 0.0;
    maker.color =
        common::ColorMap::at(common::Color::kRoyalBlue, 0.3).toRosMsg();
    maker.pose.orientation.w = 1;
    maker.lifetime = ros::Duration(0.15);

    for (int i = 0; i < num_of_points; ++i) {
      maker.points.emplace_back(
          reference_line.GetRosPose(i * sample_interval).position);
    }
    markers.markers.emplace_back(maker);
  }

  reference_line_pub_.publish(markers);
}

void NavigationMap::PublishVirtualObstacles() {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.lifetime = ros::Duration(0.15);
  for (size_t i = 0; i < virtual_obstacles_.size(); ++i) {
    PlanningVisual::GetTrafficConeMarker(virtual_obstacles_[i], i, &marker);
    markers.markers.emplace_back(marker);
  }
  virtual_obstacle_pub_.publish(markers);
}
}  // namespace planning
