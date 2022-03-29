/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/carla/mock_perception/carla_mock_perception.h"

#include <glog/logging.h>
#include <tf/tf.h>

#include "planning_core/planning_common/planning_visual.h"
#include "planning_core/planning_common/vehicle_info.h"

namespace planning {
namespace simulation {

void CarlaMockPerception::Init() {
  ros::NodeHandle node;
  objects_sub_ = node.subscribe(
      "/carla/objects", 10, &CarlaMockPerception::CarlaObjectsCallback, this);

  obstacles_pub_ =
      node.advertise<jsk_msgs::BoundingBoxArray>("/obstacle_bbox", 1);
  obstacles_info_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/obstacle_info", 1);
  obj_timer_.set_timeout(500);
}

void CarlaMockPerception::CarlaObjectsCallback(
    derived_object_msgs::ObjectArrayConstPtr objects) {
  std::lock_guard<std::mutex> lock(mutex_);
  obj_timer_.Reset();
  objects_ = objects;
}

bool CarlaMockPerception::UpdateMockPerceptionResult(
    std::vector<Obstacle>* obstacles) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (obj_timer_.timeout() || objects_ == nullptr) {
    LOG_EVERY_N(WARNING, 50) << "cannot receive objects info from carla";
    return false;
  }

  static int ego_id = VehicleInfo::Instance().id();
  if (!obstacles->empty()) obstacles->clear();
  obstacles->reserve(objects_->objects.size());

  Obstacle obstacle;
  std::vector<Obstacle> ego;
  const double stamp = objects_->header.stamp.toSec();
  for (const auto& object : objects_->objects) {
    obstacle.set_id(object.id);
    auto state = obstacle.mutable_state();
    state->position =
        Eigen::Vector2d(object.pose.position.x, object.pose.position.y);
    state->heading = tf::getYaw(object.pose.orientation);
    state->velocity = std::sqrt(object.twist.linear.x * object.twist.linear.x +
                                object.twist.linear.y * object.twist.linear.y);
    state->acceleration =
        std::sqrt(object.accel.linear.x * object.accel.linear.x +
                  object.accel.linear.y * object.accel.linear.y);
    obstacle.SetBoundingBox(object.shape.dimensions[0],
                            object.shape.dimensions[1],
                            object.shape.dimensions[2]);
    if (object.id == ego_id) {
      ego.emplace_back(obstacle);
    } else {
      obstacles->emplace_back(obstacle);
    }
  }

  jsk_msgs::BoundingBoxArray bbox_array;
  visualization_msgs::MarkerArray markers;
  PlanningVisual::ObstacleToJskBBoxArray(*obstacles, &bbox_array);
  PlanningVisual::ObstacleInfoToMarkerArray(*obstacles, &markers);
  PlanningVisual::ObstacleInfoToMarkerArray(ego, &markers);
  obstacles_pub_.publish(bbox_array);
  obstacles_info_pub_.publish(markers);

  return true;
}
}  // namespace simulation
}  // namespace planning
