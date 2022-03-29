/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/hdmap_impl.h"

#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "ad/map/access/Operation.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/GeometryOperation.hpp"
#include "common/utils/math.h"
#include "hdmap/pointcloud/pointcloud_loader.h"
#include "hdmap/road_network/lane_map.h"
#include "hdmap/utils/admap_convertion.h"
#include "hdmap/utils/admap_visual.h"

namespace hdmap {

ad::map::match::AdMapMatching HdMapImpl::admap_matching_;

bool HdMapImpl::LoadMap(const std::string& file, const std::string& pcd_file) {
  if (!::ad::map::access::init(file)) {
    LOG(ERROR) << "Fail to load hdmap in path: " << file;
    return false;
  }

  LaneMapType& lane_map = *LaneMap::mutable_lane_map();

  auto lane_ids = ::ad::map::lane::getLanes();
  for (size_t i = 0; i < lane_ids.size(); ++i) {
    auto admap_lane = ::ad::map::lane::getLanePtr(lane_ids[i]);
    /* ad_map use ECEF coordinate, cache all ENU edges manually */
    ad::map::point::getCachedENUEdge(admap_lane->edgeLeft);
    ad::map::point::getCachedENUEdge(admap_lane->edgeRight);
    ad::map::point::calculateCachedENUCenterLine(
        admap_lane->edgeLeft, admap_lane->edgeRight, admap_lane->center);
    lane_map[admap_lane->id] = std::make_shared<Lane>(admap_lane);
  }

  for (const auto& id : lane_ids) {
    lane_map[id]->FindAllNeighborLanes();
  }

  InitRos();
  VisualizeHdMap(pcd_file);

  return true;
}

void HdMapImpl::InitRos() {
  ros::NodeHandle node;
  hdmap_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("hdmap1", 10, true);
  hdmap_topo_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("hdmap_topo", 1, true);
  pointcloud_pub_ =
      node.advertise<sensor_msgs::PointCloud2>("hdmap_pointcloud", 1, true);
  full_route_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("hdmap_full_route", 1);
}

void HdMapImpl::VisualizeHdMap(const std::string& pcd_file) {
  visualization_msgs::MarkerArray hdmap_topo_markers;
  sensor_msgs::PointCloud2 hdmap_pointcloud;

  auto lane_ids = ad::map::lane::getLanes();
  for (const auto& id : lane_ids) {
    const auto& admap_lane = ad::map::lane::getLane(id);

    if (admap_lane.type != ad::map::lane::LaneType::NORMAL &&
        admap_lane.type != ad::map::lane::LaneType::INTERSECTION)
      continue;

    auto lane = LaneMap::GetLane(id);
    visualization_msgs::Marker ledge, redge, center, direction, arrow, nodes;
    AdMapVisual::SetMarkerFrameId({&ledge, &redge, &center, &direction, &arrow},
                                  "map");
    AdMapVisual::LaneToMarker(admap_lane, &ledge, &redge, &center);
    AdMapVisual::LaneNodeToMarker(admap_lane, &nodes);
    AdMapVisual::LaneDirectionToArrowMarker(*lane, &arrow);

    hdmap_markers_.markers.emplace_back(ledge);
    hdmap_markers_.markers.emplace_back(redge);
    hdmap_markers_.markers.emplace_back(center);
    hdmap_markers_.markers.emplace_back(nodes);

    // hdmap_topo_markers.markers.emplace_back(direction);
    hdmap_topo_markers.markers.emplace_back(arrow);
  }

  for (auto& marker : hdmap_markers_.markers) {
    marker.lifetime = ros::Duration(1.2);
  }

  if (!pcd_file.empty()) {
    PointCloudLoader::LoadPcdFile(pcd_file, &hdmap_pointcloud);
    hdmap_pointcloud.header.frame_id = "map";
    pointcloud_pub_.publish(hdmap_pointcloud);
  }
  hdmap_pub_.publish(hdmap_markers_);
  hdmap_topo_pub_.publish(hdmap_topo_markers);

  rviz_thread_ = std::thread(&HdMapImpl::RvizThread, this);
}

void HdMapImpl::RvizThread() {
  while (ros::ok()) {
    hdmap_pub_.publish(hdmap_markers_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void HdMapImpl::DeletePreviousRoute() {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.emplace_back(marker);
  full_route_pub_.publish(markers);
}

bool HdMapImpl::CreateFullRoute(const Eigen::Vector2d& start,
                                const Eigen::Vector2d& end,
                                const double start_heading,
                                const double end_heading,
                                FullRoute* full_route) {
  if (routing_.CreateFullRoute(start, end, start_heading, end_heading,
                               full_route)) {
    admap_matching_.addRouteHint(routing_.ad_full_route());
    DeletePreviousRoute();
    visualization_msgs::MarkerArray full_route_markers;
    AdMapVisual::FullRouteToMarkerArray(*full_route, &full_route_markers);
    full_route_pub_.publish(full_route_markers);
    return true;
  } else {
    admap_matching_.clearRouteHints();
  }
  return false;
}

LaneId HdMapImpl::NearestLane(const Eigen::Vector2d& position,
                              const double heading) {
  admap_matching_.addHeadingHint(ad::map::point::ENUHeading(heading),
                                 ad::map::access::getENUReferencePoint());
  auto nearest_lane_list = admap_matching_.getMapMatchedPositions(
      AdMapConvertion::ToEnuPoint(position), 1.0, 0.05);
  if (nearest_lane_list.empty()) {
    LOG(WARNING) << "can't find nearest lane";
    return LaneId(0);
  }
  return nearest_lane_list.front().lanePoint.paraPoint.laneId;
}

LaneId HdMapImpl::NearestLaneNoHint(const Eigen::Vector2d& position,
                                    const double heading) {
  ad::map::match::AdMapMatching admap_match;
  admap_match.addHeadingHint(ad::map::point::ENUHeading(heading),
                             ad::map::access::getENUReferencePoint());
  auto nearest_lane_list = admap_match.getMapMatchedPositions(
      AdMapConvertion::ToEnuPoint(position), 1.0, 0.05);
  if (nearest_lane_list.empty()) {
    LOG(WARNING) << "can't find nearest lane";
    return LaneId(0);
  }

  LaneId optimal_lane = 0;
  double min_angle_diff = std::numeric_limits<double>::infinity();
  for (const auto& possible_lane : nearest_lane_list) {
    if (possible_lane.type != ad::map::match::MapMatchedPositionType::LANE_IN) {
      continue;
    }
    auto lane =
        hdmap::LaneMap::GetLane(possible_lane.lanePoint.paraPoint.laneId);
    if (lane == nullptr) continue;

    auto waypoint = lane->GetWayPoint(lane->GetArcLength(position));
    double angle_diff =
        std::fabs(common::NormalizeAngle(heading - waypoint.heading));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      optimal_lane = possible_lane.lanePoint.paraPoint.laneId;
    }
  }

  return optimal_lane;
  // return nearest_lane_list.front().lanePoint.paraPoint.laneId;
}
}  // namespace hdmap
