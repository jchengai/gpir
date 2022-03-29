/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <visualization_msgs/MarkerArray.h>

#include "ad/map/lane/Lane.hpp"
#include "ad/map/point/ENUEdge.hpp"
#include "ad/map/route/FullRoute.hpp"
#include "common/utils/color_map.h"
#include "hdmap/road_network/lane.h"
#include "hdmap/routing/full_route.h"

namespace hdmap {

class AdMapVisual {
 public:
  static void SetMarkerFrameId(
      std::initializer_list<visualization_msgs::Marker*> markers,
      const std::string& frame_id);

  static bool LaneToMarker(const ad::map::lane::Lane& lane,
                           visualization_msgs::Marker* left_edge,
                           visualization_msgs::Marker* right_edge,
                           visualization_msgs::Marker* center);

  static bool LaneEdgeToMarker(const ad::map::point::ENUEdge& edge,
                               double scale, int32_t id,
                               const common::Color color,
                               const double transparent,
                               visualization_msgs::Marker* marker);

  static bool LaneNodeToMarker(const ad::map::lane::Lane& lane,
                               visualization_msgs::Marker* markers);

  static bool LaneDirectionToMarker(const ad::map::lane::Lane& lane,
                                    visualization_msgs::Marker* marker);

  static bool LaneDirectionToArrowMarker(const ad::map::lane::Lane& lane,
                                         visualization_msgs::Marker* marker);

  static bool LaneDirectionToArrowMarker(const Lane& lane,
                                         visualization_msgs::Marker* marker);

  static bool FullRouteToMarkerArray(
      const ad::map::route::FullRoute& full_route,
      visualization_msgs::MarkerArray* markers);

  static bool FullRouteToMarkerArray(const FullRoute& full_route,
                                     visualization_msgs::MarkerArray* markers);

  static bool WaypointsToMarkerArray(const std::vector<WayPoint>& waypoints,
                                     visualization_msgs::MarkerArray* makers);
};

}  // namespace hdmap
