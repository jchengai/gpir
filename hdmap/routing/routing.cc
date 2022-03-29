/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/routing/routing.h"

#include <glog/logging.h>

#include "ad/map/match/AdMapMatching.hpp"
#include "ad/map/route/Planning.hpp"
#include "common/utils/contains.hpp"
#include "hdmap/road_network/lane_map.h"
#include "hdmap/utils/admap_convertion.h"

using ad::map::match::AdMapMatching;
using ad::map::point::ENUHeading;
using ad::map::route::planning::planRoute;

namespace hdmap {

bool Routing::CreateFullRoute(const Eigen::Vector2d& start,
                              const Eigen::Vector2d& end,
                              const double start_heading,
                              const double end_heading, FullRoute* full_route) {
  AdMapMatching admap_match;

  auto ad_start_points = admap_match.getMapMatchedPositions(
      AdMapConvertion::ToEnuPoint(start), 1.0, 0.05);
  if (ad_start_points.empty()) {
    LOG(ERROR) << "Routing: start configuration is invalid";
    return false;
  }

  auto ad_end_points = admap_match.getMapMatchedPositions(
      AdMapConvertion::ToEnuPoint(end), 1.0, 0.05);
  if (ad_end_points.empty()) {
    LOG(ERROR) << "Routing: end configuration is invalid";
    return false;
  }

  ad_full_route_ = planRoute(
      ad_start_points.front().lanePoint.paraPoint, ENUHeading(start_heading),
      ad_end_points.front().lanePoint.paraPoint, ENUHeading(end_heading),
      ad::map::route::RouteCreationMode::SameDrivingDirection);

  if (ad_full_route_.roadSegments.empty()) {
    LOG(ERROR) << "Routing: create full route failed";
    return false;
  }

  // convert from admap full route
  full_route->start = start;
  full_route->end = end;
  full_route->target_lane_id = ad_end_points.front().lanePoint.paraPoint.laneId;

  for (int i = 0; i < ad_full_route_.roadSegments.size(); ++i) {
    const auto& ad_road = ad_full_route_.roadSegments[i];

    full_route->road_segments.emplace_back();
    auto road_segment = &full_route->road_segments.back();
    road_segment->num_of_drivable_lanes = ad_road.drivableLaneSegments.size();

    for (int j = 0; j < ad_road.drivableLaneSegments.size(); ++j) {
      const auto& ad_lane_segment = ad_road.drivableLaneSegments[j];
      auto lane = LaneMap::GetLane(ad_lane_segment.laneInterval.laneId);
      double start_s =
          lane->PercentageToArcLength(ad_lane_segment.laneInterval.start);
      double end_s =
          lane->PercentageToArcLength(ad_lane_segment.laneInterval.end);

      road_segment->lane_segments.emplace_back();
      auto lane_segment = &road_segment->lane_segments.back();

      lane_segment->id = ad_lane_segment.laneInterval.laneId;
      lane_segment->start_s = start_s;
      lane_segment->end_s = end_s;
      lane_segment->length = end_s - start_s;
      lane_segment->left_neighbor_id = ad_lane_segment.leftNeighbor;
      lane_segment->right_neighbor_id = ad_lane_segment.rightNeighbor;
      lane_segment->successors_id = ad_lane_segment.successors;
      lane_segment->presuccessor_id = ad_lane_segment.predecessors;
      full_route->lane_map[lane_segment->id] = std::pair<int, int>{i, j};
    }
  }

  DecideLaneSegmentBehavior(full_route->target_lane_id, full_route);

  has_route_ = true;
  return true;
}

void Routing::DecideLaneSegmentBehavior(const LaneId end_lane_id,
                                        FullRoute* full_route) {
  for (int i = ad_full_route_.roadSegments.size() - 1; i >= 0; --i) {
    const auto& ad_lanes = ad_full_route_.roadSegments[i].drivableLaneSegments;
    for (const auto& lane : ad_lanes) {
      auto lane_segment = full_route->GetLaneSegment(lane.laneInterval.laneId);
      if (lane.successors.empty()) {
        lane_segment->maximum_lane_keeping_length = lane_segment->length;
      }
      for (const auto& presuccessor_id : lane.predecessors) {
        auto presuccessor = full_route->GetLaneSegment(presuccessor_id);
        // * it could be the presuccessor of several lanes, choose the longest
        presuccessor->maximum_lane_keeping_length = std::max(
            presuccessor->maximum_lane_keeping_length,
            presuccessor->length + lane_segment->maximum_lane_keeping_length);
      }
    }

    std::vector<size_t> extensible_lanes, inextensible_lanes;
    PartitionLaneSegments(ad_lanes, end_lane_id, &extensible_lanes,
                          &inextensible_lanes);

    for (const auto& index : extensible_lanes) {
      full_route->road_segments[i].lane_segments[index].behavior =
          DecideExtensibleLaneBehavior(index, extensible_lanes);
    }
    for (const auto& index : inextensible_lanes) {
      full_route->road_segments[i].lane_segments[index].behavior =
          DecideUnExtensibleLaneBehavior(index, extensible_lanes);
    }
  }
}

std::vector<LaneSegmentBehavior> Routing::DecideUnExtensibleLaneBehavior(
    const int index, std::vector<size_t> extensible_lanes) {
  std::vector<LaneSegmentBehavior> behaviors;
  auto it =
      std::upper_bound(extensible_lanes.begin(), extensible_lanes.end(), index);

  if (it == extensible_lanes.begin()) {
    // * only have extensible lane on the left
    behaviors.emplace_back(LaneSegmentBehavior::kLeftChange);
  } else if (it == extensible_lanes.end()) {
    // * only have extensible lane on the right
    behaviors.emplace_back(LaneSegmentBehavior::kRightChange);
  } else {
    // * have extensible lane on both side, choose the nearest lane
    int right_distance = index - (*(it - 1));
    int left_distance = (*it) - index;
    if (left_distance >= right_distance)
      behaviors.emplace_back(LaneSegmentBehavior::kRightChange);
    if (left_distance <= right_distance)
      behaviors.emplace_back(LaneSegmentBehavior::kLeftChange);
  }

  return behaviors;
}

std::vector<LaneSegmentBehavior> Routing::DecideExtensibleLaneBehavior(
    const int index, std::vector<size_t> extensible_lanes) {
  std::vector<LaneSegmentBehavior> behaviors;
  behaviors.emplace_back(LaneSegmentBehavior::kKeep);
  for (int i = 0; i < extensible_lanes.size(); ++i) {
    // * ingore double lane change
    if (index - i == 1)
      behaviors.emplace_back(LaneSegmentBehavior::kRightChange);
    else if (index - i == -1)
      behaviors.emplace_back(LaneSegmentBehavior::kLeftChange);
  }
  return behaviors;
}

void Routing::PartitionLaneSegments(
    const ad::map::route::LaneSegmentList& lane_segments,
    const LaneId& end_lane_id, std::vector<size_t>* extensible_lanes,
    std::vector<size_t>* inextensible_lanes) {
  for (size_t i = 0; i < lane_segments.size(); ++i) {
    if (lane_segments[i].successors.empty() &&
        lane_segments[i].laneInterval.laneId != end_lane_id)
      inextensible_lanes->emplace_back(i);
    else
      extensible_lanes->emplace_back(i);
  }
}

size_t Routing::FindLaneInLaneSegments(
    const ad::map::route::LaneSegmentList& lane_segments, const LaneId& id) {
  return std::distance(
      lane_segments.begin(),
      std::find_if(lane_segments.begin(), lane_segments.end(),
                   [&id](const ad::map::route::LaneSegment& lane_segment) {
                     return lane_segment.laneInterval.laneId == id;
                   }));
}
}  // namespace hdmap
