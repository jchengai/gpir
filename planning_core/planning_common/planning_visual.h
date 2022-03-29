/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/utils/color_map.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

namespace jsk_msgs = jsk_recognition_msgs;

class PlanningVisual {
 public:
  static void FillHeader(std_msgs::Header* header);

  static void SetScaleAndColor(const std::array<double, 3>& scale,
                               common::Color,
                               visualization_msgs::Marker* marker,
                               const double alpha = 1.0);

  static void BBoxToSolidCubeMarker(const common::Box2D& bbox,
                                    common::Color color,
                                    visualization_msgs::Marker* maker);

  static void BBoxToJskBBox(const common::Box2D& bbox,
                            jsk_recognition_msgs::BoundingBox* jsk_bbox,
                            const int label);

  static void ObstacleToJskBBoxArray(
      const std::vector<Obstacle>& obstacles,
      jsk_recognition_msgs::BoundingBoxArray* bbox_array);

  static void ObstacleInfoToMarkerArray(
      const std::vector<Obstacle>& obstacles,
      visualization_msgs::MarkerArray* makers);

  static void GetTrafficConeMarker(const Eigen::Vector2d& pos, const int id,
                                   visualization_msgs::Marker* marker);

  static void Get2DBoxMarker(const Eigen::Vector2d& pos, const double width,
                             const double length, const double heading,
                             common::Color color,
                             const std::array<double, 3>& scale,
                             visualization_msgs::Marker* marker);
};
}  // namespace planning
