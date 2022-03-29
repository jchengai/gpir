/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#pragma once

namespace common {

class CommonVisual {
 public:
  static void FillHeader(visualization_msgs::Marker* maker);

  static void StateToPose(const Eigen::Vector2d& position, const double heading,
                          geometry_msgs::Pose* pose, const double z = 0.0);

  static void PositionToPose(const Eigen::Vector2d& position,
                             geometry_msgs::Pose* pose, const double z = 0.0);

  static void PositionToPoint(const Eigen::Vector2d& position,
                              geometry_msgs::Point* point,
                              const double z = 0.0);
};

}  // namespace common
