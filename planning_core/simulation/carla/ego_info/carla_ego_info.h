/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mutex>

#include "carla_msgs/CarlaEgoVehicleInfo.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "common/base/state.h"
#include "common/utils/timer.h"
#include "planning_core/planning_common/vehicle_info.h"

namespace planning {
namespace simulation {

class CarlaEgoInfo {
 public:
  CarlaEgoInfo() = default;

  void Init();

  bool GetVehicleParam(VehicleParam* vehicle_param);

  bool UpdateEgoState(common::State* state);

 private:
  void CarlaEgoInfoCallback(carla_msgs::CarlaEgoVehicleInfoConstPtr ego_info);

  void CarlaEgoStatusCallback(
      carla_msgs::CarlaEgoVehicleStatusConstPtr ego_status);

  void CarlaOdometryCallback(nav_msgs::OdometryConstPtr odometry);

 private:
  ros::Subscriber odom_sub_;
  ros::Subscriber ego_status_sub_;
  ros::Subscriber ego_info_sub_;

  common::Timer odom_timer_;
  common::Timer ego_status_timer_;

  std::mutex mutex_odom_;
  std::mutex mutex_status_;
  std::mutex mutex_info_;

  bool is_ego_info_received_ = false;

  std::string vehicle_type_;
  nav_msgs::OdometryConstPtr odometry_ = nullptr;
  carla_msgs::CarlaEgoVehicleStatusConstPtr ego_status_ = nullptr;
  carla_msgs::CarlaEgoVehicleInfoConstPtr ego_info_ = nullptr;
};

}  // namespace simulation
}  // namespace planning
