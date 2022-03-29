/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/utils/admap_visual.h"

#include <string>

#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/EdgeOperation.hpp"
#include "hdmap/road_network/lane_map.h"

namespace hdmap {

using common::Color;
using common::ColorMap;

static uint32_t id_cnt = 0;

constexpr std::array<Color, 6> route_colors{Color::kSkyBlue, Color::kPink,
                                            Color::kOrange,  Color::kGreen,
                                            Color::kMagenta, Color::kRed};

void DefaultOrientation(geometry_msgs::Pose* pose) {
  pose->orientation.x = 0.0;
  pose->orientation.y = 0.0;
  pose->orientation.z = 0.0;
  pose->orientation.w = 1.0;
}

void AdMapVisual::SetMarkerFrameId(
    std::initializer_list<visualization_msgs::Marker*> markers,
    const std::string& frame_id) {
  for (auto marker : markers) {
    marker->header.frame_id = frame_id;
  }
}

bool AdMapVisual::LaneToMarker(const ad::map::lane::Lane& lane,
                               visualization_msgs::Marker* left_edge,
                               visualization_msgs::Marker* right_edge,
                               visualization_msgs::Marker* center) {
  LaneEdgeToMarker(lane.edgeLeft.private_enuEdgeCache.enuEdge, 0.1, id_cnt++,
                   common::kBlack, 0.8, left_edge);
  LaneEdgeToMarker(lane.edgeRight.private_enuEdgeCache.enuEdge, 0.1, id_cnt++,
                   common::kBlack, 0.8, right_edge);
  LaneEdgeToMarker(lane.center, 0.1, id_cnt++, common::kRoyalBlue, 1.0,
                   center);
  return true;
}

bool AdMapVisual::LaneEdgeToMarker(const ad::map::point::ENUEdge& edge,
                                   double scale, int32_t id,
                                   const common::Color color,
                                   const double transparent,
                                   visualization_msgs::Marker* marker) {
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time::now();
  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->id = id;
  marker->scale.x = scale;
  marker->color = ColorMap::at(color, transparent).toRosMsg();

  for (const auto enu_point : edge) {
    geometry_msgs::Point point;
    point.x = static_cast<double>(enu_point.x);
    point.y = static_cast<double>(enu_point.y);
    point.z = static_cast<double>(enu_point.z);
    marker->points.push_back(point);
  }

  DefaultOrientation(&marker->pose);

  return true;
}

bool AdMapVisual::LaneNodeToMarker(const ad::map::lane::Lane& lane,
                                   visualization_msgs::Marker* marker) {
  const auto& left_edge = lane.edgeLeft.private_enuEdgeCache.enuEdge;
  const auto& right_edge = lane.edgeRight.private_enuEdgeCache.enuEdge;
  std::vector<ad::map::point::ENUPoint> nodes{
      left_edge.front(), left_edge.back(),    right_edge.front(),
      right_edge.back(), lane.center.front(), lane.center.back()};

  marker->header.frame_id = "map";
  marker->type = visualization_msgs::Marker::SPHERE_LIST;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->id = id_cnt++;
  marker->scale.x = marker->scale.y = marker->scale.z = 0.2;
  marker->color = ColorMap::at(Color::kRoyalBlue).toRosMsg();
  geometry_msgs::Point point;
  for (const auto& node : nodes) {
    point.x = node.x;
    point.y = node.y;
    point.z = 0.0;
    marker->points.emplace_back(point);
  }
  DefaultOrientation(&marker->pose);

  return true;
}

bool AdMapVisual::LaneDirectionToMarker(const ad::map::lane::Lane& lane,
                                        visualization_msgs::Marker* marker) {
  marker->header.frame_id = "map";
  marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->text = toString(lane.direction);
  marker->id = id_cnt++;
  marker->scale.z = 1.0;
  marker->color = ColorMap::at(Color::kMagenta, 1.0).toRosMsg();

  size_t size = lane.center.size();
  DefaultOrientation(&marker->pose);

  if (size == 2) {
    marker->pose.position.x =
        (lane.center.front().x + lane.center.back().x) / 2.0;
    marker->pose.position.y =
        (lane.center.front().y + lane.center.back().y) / 2.0;
    marker->pose.position.z =
        (lane.center.front().z + lane.center.back().z) / 2.0;
  } else {
    marker->pose.position.x = lane.center[size / 2].x;
    marker->pose.position.y = lane.center[size / 2].y;
    marker->pose.position.z = lane.center[size / 2].z;
  }

  return true;
}

bool AdMapVisual::LaneDirectionToArrowMarker(
    const ad::map::lane::Lane& lane, visualization_msgs::Marker* marker) {
  marker->header.frame_id = "map";
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->id = id_cnt++;
  marker->scale.x = 0.2;
  marker->scale.y = 0.5;
  marker->scale.z = 2.0;
  marker->color = ColorMap::at(Color::kBlue, 1.0).toRosMsg();
  marker->points.resize(2);

  marker->points[0].x = lane.center.front().x;
  marker->points[0].y = lane.center.front().y;
  marker->points[0].z = lane.center.front().z;

  marker->points[1].x = lane.center.back().x;
  marker->points[1].y = lane.center.back().y;
  marker->points[1].z = lane.center.back().z;

  DefaultOrientation(&marker->pose);

  return true;
}

bool AdMapVisual::LaneDirectionToArrowMarker(
    const Lane& lane, visualization_msgs::Marker* marker) {
  marker->header.frame_id = "map";
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->id = id_cnt++;
  marker->scale.x = 0.2;
  marker->scale.y = 0.5;
  marker->scale.z = 2.0;
  marker->color = ColorMap::at(Color::kRoyalBlue, 1.0).toRosMsg();
  marker->points.resize(2);

  const auto& way_points = lane.way_points();
  marker->points[0].x = way_points.front().point.x();
  marker->points[0].y = way_points.front().point.y();
  marker->points[0].z = 0.1;

  marker->points[1].x = way_points.back().point.x();
  marker->points[1].y = way_points.back().point.y();
  marker->points[1].z = 0.1;

  DefaultOrientation(&marker->pose);

  return true;
}

bool AdMapVisual::FullRouteToMarkerArray(
    const ad::map::route::FullRoute& full_route,
    visualization_msgs::MarkerArray* markers) {
  int count = 0;
  for (const auto& road : full_route.roadSegments) {
    for (const auto& lane_segment : road.drivableLaneSegments) {
      const auto& lane =
          ad::map::lane::getLane(lane_segment.laneInterval.laneId);
      visualization_msgs::Marker lane_marker;
      LaneEdgeToMarker(lane.center, lane.width, id_cnt++, route_colors[count],
                       0.2, &lane_marker);
      markers->markers.emplace_back(lane_marker);
      ++count;
    }
    count = 0;
  }
  return true;
}

bool AdMapVisual::FullRouteToMarkerArray(
    const FullRoute& full_route, visualization_msgs::MarkerArray* markers) {
  visualization_msgs::Marker route_marker, text_marker;
  route_marker.header.frame_id = "map";
  route_marker.header.stamp = ros::Time::now();
  route_marker.type = visualization_msgs::Marker::LINE_STRIP;
  route_marker.action = visualization_msgs::Marker::MODIFY;
  route_marker.scale.x = 3.0;

  text_marker.header.frame_id = "map";
  text_marker.header.stamp = ros::Time::now();
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::MODIFY;
  text_marker.scale.z = 1;
  text_marker.pose.position.z = 0.1;
  text_marker.color = ColorMap::at(Color::kBlack).toRosMsg();
  DefaultOrientation(&route_marker.pose);
  DefaultOrientation(&text_marker.pose);

  geometry_msgs::Point point;

  for (const auto& road : full_route.road_segments) {
    for (size_t i = 0; i < road.lane_segments.size(); ++i) {
      route_marker.id = id_cnt++;
      route_marker.points.clear();
      route_marker.color = ColorMap::at(route_colors[i], 0.5).toRosMsg();

      auto lane = LaneMap::GetLane(road.lane_segments[i].id);

      auto waypoints = lane->way_points(road.lane_segments[i].start_s,
                                        road.lane_segments[i].end_s);
      for (const auto& p : waypoints) {
        point.x = p.point.x();
        point.y = p.point.y();
        point.z = 0.1;
        route_marker.points.emplace_back(point);
      }

      std::string behavior_string = "";
      for (const auto& behavior : road.lane_segments[i].behavior) {
        behavior_string += ToString(behavior) + "\n";
      }
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2)
         << road.lane_segments[i].maximum_lane_keeping_length;
      behavior_string += ss.str() + "\n";
      behavior_string += std::to_string(lane->id());
      text_marker.id = id_cnt++;
      text_marker.pose.position.x = waypoints[waypoints.size() / 2].point.x();
      text_marker.pose.position.y = waypoints[waypoints.size() / 2].point.y();
      text_marker.text = behavior_string;

      markers->markers.emplace_back(text_marker);
      markers->markers.emplace_back(route_marker);
    }
  }

  return true;
}

bool AdMapVisual::WaypointsToMarkerArray(
    const std::vector<WayPoint>& waypoints,
    visualization_msgs::MarkerArray* makers) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.id = 98888;
  marker.color = ColorMap::at(Color::kRed).toRosMsg();
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;

  geometry_msgs::Point point;
  for (const auto& p : waypoints) {
    point.x = p.point.x();
    point.y = p.point.y();
    point.z = 0.1;
    marker.points.emplace_back(point);
  }
  DefaultOrientation(&marker.pose);
  makers->markers.emplace_back(marker);

  return true;
}
}  // namespace hdmap
