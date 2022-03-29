/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <derived_object_msgs/ObjectArray.h>
#include <ros/ros.h>

#include <mutex>

#include "common/utils/timer.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {
namespace simulation {

class CarlaMockPerception {
 public:
  CarlaMockPerception() = default;

  void Init();

  bool UpdateMockPerceptionResult(std::vector<Obstacle>* obstacles);

 private:
  void CarlaObjectsCallback(derived_object_msgs::ObjectArrayConstPtr objects);

 private:
  ::common::Timer obj_timer_;
  ros::Subscriber objects_sub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher obstacles_info_pub_;

  std::mutex mutex_;
  derived_object_msgs::ObjectArrayConstPtr objects_ = nullptr;
};

}  // namespace simulation
}  // namespace planning
