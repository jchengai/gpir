/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/road_network/lane.h"

#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/ENUOperation.hpp"
#include "common/utils/math.h"
#include "hdmap/road_network/lane_map.h"
#include "hdmap/utils/admap_convertion.h"

namespace hdmap {

using ad::map::point::distance;

Lane::Lane(const ::ad::map::lane::Lane::ConstPtr admap_lane)
    : id_(admap_lane->id),
      length_(admap_lane->length),
      width_(admap_lane->width),
      admap_lane_(admap_lane) {
  for (const auto& contact_lane : admap_lane_->contactLanes) {
    auto contact_lane_direction =
        ad::map::lane::getLane(contact_lane.toLane).direction;
    switch (contact_lane.location) {
      case ad::map::lane::ContactLocation::LEFT:
        if (admap_lane_->direction == contact_lane_direction) {
          if (admap_lane_->direction ==
              ad::map::lane::LaneDirection::POSITIVE) {
            AddNeighborLane(contact_lane.toLane, LaneNeighborType::kLeft);
          } else {
            AddNeighborLane(contact_lane.toLane, LaneNeighborType::kRight);
          }
        }
        break;
      case ad::map::lane::ContactLocation::RIGHT:
        if (admap_lane_->direction == contact_lane_direction) {
          if (admap_lane_->direction ==
              ad::map::lane::LaneDirection::POSITIVE) {
            AddNeighborLane(contact_lane.toLane, LaneNeighborType::kRight);
          } else {
            AddNeighborLane(contact_lane.toLane, LaneNeighborType::kLeft);
          }
        }
        break;
      case ad::map::lane::ContactLocation::SUCCESSOR:
        if (admap_lane_->direction == ad::map::lane::LaneDirection::POSITIVE) {
          AddConnectedLane(contact_lane, LaneConnectionType::kNext);
        } else {
          AddConnectedLane(contact_lane, LaneConnectionType::kPrevious);
        }
        break;
      case ad::map::lane::ContactLocation::PREDECESSOR:
        if (admap_lane_->direction == ad::map::lane::LaneDirection::POSITIVE) {
          AddConnectedLane(contact_lane, LaneConnectionType::kPrevious);
        } else {
          AddConnectedLane(contact_lane, LaneConnectionType::kNext);
        }
        break;

      default:
        break;
    }
  }
  CreateWayPoints();
}

void Lane::FindAllNeighborLanes() {
  std::function<void(const std::shared_ptr<const Lane>,
                     const LaneNeighborType type)>
      find_neighbor =
          [this, &find_neighbor](const std::shared_ptr<const Lane> current_lane,
                                 const LaneNeighborType type) -> void {
    if (current_lane->HasNeighbor(type)) {
      auto neighbor_lane = current_lane->neighbor_lane(type);
      AddNeighborLane(neighbor_lane->id(), type);
      find_neighbor(neighbor_lane, type);
    }
  };

  if (HasLeftNeighbor()) find_neighbor(left_lane(), LaneNeighborType::kLeft);
  if (HasRightNeighbor()) find_neighbor(right_lane(), LaneNeighborType::kRight);
}

void Lane::AddNeighborLane(const LaneId neighbor_id, LaneNeighborType type) {
  auto admap_neighbor_lane = ::ad::map::lane::getLanePtr(neighbor_id);
  NeighborLane neighbor_lane;
  neighbor_lane.id = neighbor_id;
  neighbor_lane.admap_lane = admap_neighbor_lane;
  /* assume no solid line for simplicity */
  neighbor_lane.lane_changable =
      admap_neighbor_lane->direction == admap_lane_->direction;
  neighbor_lanes_[static_cast<size_t>(type)].emplace_back(neighbor_lane);
}

void Lane::AddConnectedLane(const ad::map::lane::ContactLane& contact_lane,
                            LaneConnectionType type) {
  if (std::find(contact_lane.types.begin(), contact_lane.types.end(),
                ::ad::map::lane::ContactType::LANE_CONTINUATION) !=
      contact_lane.types.end()) {
    ConnectedLane connected_lane;
    connected_lane.id = contact_lane.toLane;
    connected_lanes_[static_cast<size_t>(type)].emplace_back(connected_lane);
  }
}

void Lane::CreateWayPoints() {
  auto center_line = admap_lane_->center;
  auto left_edge = admap_lane_->edgeLeft.private_enuEdgeCache.enuEdge;
  auto right_edge = admap_lane_->edgeRight.private_enuEdgeCache.enuEdge;

  // assert(left_edge.size() == center_line.size());
  // assert(right_edge.size() == center_line.size());

  points_num_ = center_line.size();

  if (admap_lane_->direction == ad::map::lane::LaneDirection::NEGATIVE) {
    std::reverse(center_line.begin(), center_line.end());
    std::reverse(left_edge.begin(), left_edge.end());
    std::reverse(right_edge.begin(), right_edge.end());
  }

  double arc_length = 0.0;
  double left = 0.0, right = 0.0;
  WayPoint way_point;
  for (int i = 0; i < points_num_ - 1; ++i) {
    way_point.s = arc_length;
    way_point.heading = std::atan2(center_line[i + 1].y - center_line[i].y,
                                   center_line[i + 1].x - center_line[i].x);
    way_point.point = AdMapConvertion::FromEnuPoint(center_line[i]);
    way_points_.emplace_back(way_point);
    arc_length += distance(center_line[i], center_line[i + 1]);
    // left = distance(left_edge[i], center_line[i]);
    // right = -distance(right_edge[i], center_line[i]);
    // lane_boundary_.emplace_back(std::make_pair(left, right));
  }
  way_point.s = arc_length;
  way_point.heading = way_points_.empty() ? 0.0 : way_points_.back().heading;
  way_point.point = AdMapConvertion::FromEnuPoint(center_line.back());
  way_points_.emplace_back(way_point);
  // lane_boundary_.emplace_back(std::make_pair(
  //     distance(left_edge[points_num_ - 1], center_line[points_num_ - 1]),
  //     -distance(right_edge[points_num_ - 1], center_line[points_num_ - 1])));

  // * update arc length
  length_ = arc_length;
}

bool Lane::HasNeighbor(const LaneNeighborType type) const {
  if (type == LaneNeighborType::kRight)
    return HasRightNeighbor();
  else if (type == LaneNeighborType::kLeft)
    return HasLeftNeighbor();
  else
    return false;
}

bool Lane::HasLeftNeighbor() const { return !neighbor_lanes_[0].empty(); }

bool Lane::HasRightNeighbor() const { return !neighbor_lanes_[1].empty(); }

bool Lane::LeftLaneChangable() const {
  if (HasLeftNeighbor())
    return neighbor_lanes_[0][0].lane_changable;
  else
    return false;
}

bool Lane::RightLaneChangable() const {
  if (HasRightNeighbor())
    return neighbor_lanes_[1][0].lane_changable;
  else
    return false;
}

std::shared_ptr<const Lane> Lane::neighbor_lane(
    const LaneNeighborType type) const {
  if (type == LaneNeighborType::kLeft)
    return left_lane();
  else if (type == LaneNeighborType::kRight)
    return right_lane();
  else
    return nullptr;
}

std::shared_ptr<const Lane> Lane::left_lane() const {
  if (HasLeftNeighbor()) {
    return LaneMap::GetLane(neighbor_lanes_[0][0].id);
  } else {
    return nullptr;
  }
}

std::shared_ptr<const Lane> Lane::right_lane() const {
  if (HasRightNeighbor()) {
    return LaneMap::GetLane(neighbor_lanes_[1][0].id);
  } else {
    return nullptr;
  }
}

template <typename T>
bool HasRelationshipWith(const std::vector<T>& lanes, const LaneId& id) {
  return std::find_if(lanes.cbegin(), lanes.cend(),
                      [&id](const T& lane) -> bool { return id == lane.id; }) !=
         lanes.end();
}

bool Lane::HasSuccessor() const {
  return !connected_lanes_[static_cast<size_t>(LaneConnectionType::kNext)]
              .empty();
}

bool Lane::HasSuccessor(const LaneId& id) const {
  return HasRelationshipWith(
      connected_lanes_[static_cast<size_t>(LaneConnectionType::kNext)], id);
}

bool Lane::HasPreSuccessor() const {
  return !connected_lanes_[static_cast<size_t>(LaneConnectionType::kPrevious)]
              .empty();
}

bool Lane::HasPreSuccessor(const LaneId& id) const {
  return HasRelationshipWith(
      connected_lanes_[static_cast<size_t>(LaneConnectionType::kPrevious)], id);
}

std::vector<LaneId> Lane::next_lanes() const {
  std::vector<LaneId> next_lanes;
  for (const auto& lane :
       connected_lanes_[static_cast<size_t>(LaneConnectionType::kNext)]) {
    next_lanes.emplace_back(lane.id);
  }
  return next_lanes;
}

std::vector<LaneId> Lane::previous_lanes() const {
  std::vector<LaneId> previous_lanes;
  for (const auto& lane :
       connected_lanes_[static_cast<size_t>(LaneConnectionType::kPrevious)]) {
    previous_lanes.emplace_back(lane.id);
  }
  return previous_lanes;
}

double Lane::PercentageToArcLength(const double percentage_s) {
  if (0.0 <= percentage_s && percentage_s <= 1.0) {
    double true_s =
        admap_lane_->direction == ad::map::lane::LaneDirection::NEGATIVE
            ? 1 - percentage_s
            : percentage_s;

    return true_s * length_;
  } else {
    spdlog::error("percentage s ({}) should be in [0, 1]", percentage_s);
    return 0.0;
  }
}

WayPoint Lane::Interpolate(const WayPoint& p1, const WayPoint& p2,
                           const double s) const {
  const double weight = (s - p1.s) / (p2.s - p1.s);
  WayPoint p;
  p.point = (1 - weight) * p1.point + weight * p2.point;
  p.s = s;
  p.heading = common::InterpolateAngle(p1.heading, p1.s, p2.heading, p2.s, s);
  return p;
}

std::vector<WayPoint> Lane::way_points(const double start_s,
                                       const double end_s) const {
  std::vector<WayPoint> waypoints;
  for (const auto& p : way_points_) {
    if (start_s <= p.s && p.s <= end_s) {
      // TODO: use linear interpolate here
      waypoints.emplace_back(p);
    }
  }
  return waypoints;
}

WayPoint Lane::GetWayPoint(const double s) {
  int index = std::distance(
      way_points_.begin(),
      std::lower_bound(way_points_.begin(), way_points_.end(), s,
                       [](const WayPoint& p, const double val) -> bool {
                         return p.s < val;
                       }));
  return Interpolate(way_points_[std::max(0, index - 1)],
                     way_points_[std::min(index, points_num_ - 1)], s);
}

int Lane::GetNearestionPointIndex(const Eigen::Vector2d& point) {
  int min_index = 0;
  double dist = 0.0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < points_num_; ++i) {
    dist = (way_points_[i].point - point).squaredNorm();
    if (dist < min_dist) {
      min_index = i;
      min_dist = dist;
    }
  }
  return min_index;
}

double Lane::GetArcLength(const Eigen::Vector2d& point) {
  int min_index = GetNearestionPointIndex(point);
  Eigen::Vector2d tangent(std::cos(way_points_[min_index].heading),
                          std::sin(way_points_[min_index].heading));
  return std::min(
      std::max(way_points_[min_index].s +
                   tangent.dot(point - way_points_[min_index].point),
               0.0),
      length_);
}

SLPair Lane::GetProjection(const Eigen::Vector2d& point) {
  int min_index = GetNearestionPointIndex(point);
  Eigen::Vector2d vec = point - way_points_[min_index].point;
  const double cos = std::cos(way_points_[min_index].heading);
  const double sin = std::sin(way_points_[min_index].heading);
  return std::make_pair(
      way_points_[min_index].s + cos * vec.x() + sin * vec.y(),
      -sin * vec.x() + cos * vec.y());
}

SLPair Lane::GetProjectionAndBoundary(const Eigen::Vector2d& point,
                                      std::pair<double, double>* boundary) {
  int min_index = GetNearestionPointIndex(point);
  *boundary = std::make_pair<double, double>(-width_ / 2, width_ / 2);
  Eigen::Vector2d vec = point - way_points_[min_index].point;
  const double cos = std::cos(way_points_[min_index].heading);
  const double sin = std::sin(way_points_[min_index].heading);
  return std::make_pair(
      way_points_[min_index].s + cos * vec.x() + sin * vec.y(),
      -sin * vec.x() + cos * vec.y());
}

bool Lane::IsInLane(const Eigen::Vector2d& point) {
  auto sl = GetProjection(point);
  std::cout << sl.second << " vs. " << width_ / 2 << std::endl;
  return (sl.first > 0.0 && sl.first < length_ &&
          std::fabs(sl.second) < width_ / 2);
}
}  // namespace hdmap
