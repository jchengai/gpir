/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <string>
#include <thread>

#include "ad/map/match/AdMapMatching.hpp"
#include "hdmap/road_network/lane.h"
#include "hdmap/routing/routing.h"

namespace hdmap {

class HdMapImpl {
 public:
  HdMapImpl() = default;
  HdMapImpl(const HdMapImpl&) = delete;
  HdMapImpl& operator=(const HdMapImpl&) = delete;
  ~HdMapImpl() {
    if (rviz_thread_.joinable()) rviz_thread_.join();
  }

  bool LoadMap(const std::string& map_file, const std::string& pcd_file = "");

  bool CreateFullRoute(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                       const double start_heading, const double end_heading,
                       FullRoute* full_route);

  LaneId NearestLane(const Eigen::Vector2d& position,
                     const double heading = 0.0);

  LaneId NearestLaneNoHint(const Eigen::Vector2d& position,
                           const double heading = 0.0);

 private:
  void InitRos();
  void VisualizeHdMap(const std::string& pcd_file = "");
  void DeletePreviousRoute();
  void RvizThread();

 private:
  ros::Publisher hdmap_pub_;
  ros::Publisher hdmap_topo_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher full_route_pub_;

  std::thread rviz_thread_;
  visualization_msgs::MarkerArray hdmap_markers_;

  Routing routing_;
  static ad::map::match::AdMapMatching admap_matching_;
};
}  // namespace hdmap
