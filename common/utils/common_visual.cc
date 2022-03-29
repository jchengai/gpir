/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "common/utils/common_visual.h"

#include <tf/tf.h>

namespace common {

void CommonVisual::FillHeader(visualization_msgs::Marker* maker) {
  maker->header.frame_id = "map";
  maker->header.stamp = ros::Time::now();
}

void CommonVisual::StateToPose(const Eigen::Vector2d& position,
                               const double heading, geometry_msgs::Pose* pose,
                               const double z) {
  pose->position.x = position.x();
  pose->position.y = position.y();
  pose->position.z = z;

  auto q = tf::createQuaternionFromYaw(heading);
  tf::quaternionTFToMsg(q, pose->orientation);
}

void CommonVisual::PositionToPose(const Eigen::Vector2d& position,
                                  geometry_msgs::Pose* pose, const double z) {
  pose->position.x = position.x();
  pose->position.y = position.y();
  pose->position.z = z;
  pose->orientation.x = 0;
  pose->orientation.y = 0;
  pose->orientation.z = 0;
  pose->orientation.w = 1;
}

void CommonVisual::PositionToPoint(const Eigen::Vector2d& position,
                                   geometry_msgs::Point* point,
                                   const double z) {
  point->x = position.x();
  point->y = position.y();
  point->z = z;
}

}  // namespace common
