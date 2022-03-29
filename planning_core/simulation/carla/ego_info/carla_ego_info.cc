/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/carla/ego_info/carla_ego_info.h"

#include <glog/logging.h>
#include <tf/tf.h>

#include <Eigen/Geometry>

#include "planning_core/planning_common/vehicle_info.h"

namespace planning {
namespace simulation {

void CarlaEgoInfo::Init() {
  ros::NodeHandle node;

  odom_sub_ = node.subscribe("/carla/ego_vehicle/odometry", 10,
                             &CarlaEgoInfo::CarlaOdometryCallback, this);
  ego_status_sub_ = node.subscribe("/carla/ego_vehicle/vehicle_status", 10,
                                   &CarlaEgoInfo::CarlaEgoStatusCallback, this);
  ego_info_sub_ = node.subscribe("/carla/ego_vehicle/vehicle_info", 10,
                                 &CarlaEgoInfo::CarlaEgoInfoCallback, this);

  odom_timer_.set_timeout(500);
  ego_status_timer_.set_timeout(500);
}

void CarlaEgoInfo::CarlaEgoInfoCallback(
    carla_msgs::CarlaEgoVehicleInfoConstPtr ego_info) {
  std::lock_guard<std::mutex> lock(mutex_info_);
  is_ego_info_received_ = true;
  ego_info_ = ego_info;
}

void CarlaEgoInfo::CarlaOdometryCallback(nav_msgs::OdometryConstPtr odometry) {
  std::lock_guard<std::mutex> lock(mutex_odom_);
  odom_timer_.Reset();
  odometry_ = odometry;
}

void CarlaEgoInfo::CarlaEgoStatusCallback(
    carla_msgs::CarlaEgoVehicleStatusConstPtr ego_status) {
  std::lock_guard<std::mutex> lock(mutex_status_);
  ego_status_timer_.Reset();
  ego_status_ = ego_status;
}

bool CarlaEgoInfo::UpdateEgoState(common::State* state) {
  {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    if (odom_timer_.timeout() || odometry_ == nullptr) {
      LOG_EVERY_N(WARNING, 20) << "cannot receive odom data from carla";
      return false;
    }
    state->stamp = odometry_->header.stamp.toSec();
    state->position = Eigen::Vector2d(odometry_->pose.pose.position.x,
                                      odometry_->pose.pose.position.y);
    state->heading = tf::getYaw(odometry_->pose.pose.orientation);
    state->velocity = odometry_->twist.twist.linear.x;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_status_);
    if (ego_status_timer_.timeout() || ego_status_ == nullptr) {
      LOG_EVERY_N(WARNING, 20) << "cannot receive ego status from carla";
      return false;
    }
    Eigen::Quaterniond q(ego_status_->orientation.w, ego_status_->orientation.x,
                         ego_status_->orientation.y,
                         ego_status_->orientation.z);
    auto acc_vrf = q * Eigen::Vector3d(ego_status_->acceleration.linear.x,
                                       ego_status_->acceleration.linear.y,
                                       ego_status_->acceleration.linear.z);
    state->acceleration = acc_vrf.x();
  }
  return true;
}

bool CarlaEgoInfo::GetVehicleParam(VehicleParam* vehicle_param) {
  std::lock_guard<std::mutex> lock(mutex_info_);
  if (!is_ego_info_received_ || ego_info_ == nullptr) {
    LOG_EVERY_N(WARNING, 20) << "cannot receive ego info from carla";
    return false;
  }

  VehicleInfo::Instance().set_id(ego_info_->id);
  const auto& wheels = ego_info_->wheels;
  vehicle_param->wheel_base = (wheels[0].position.x + wheels[1].position.x -
                               wheels[2].position.x - wheels[3].position.x) /
                              2.0;
  CHECK_GT(vehicle_param->wheel_base, 1.0)
      << "You may need to update your carla ros_bridge, check "
         "https://github.com/carla-simulator/ros-bridge/issues/521";
  vehicle_param->max_steer_angle =
      (wheels[0].max_steer_angle + wheels[1].max_steer_angle) / 2.0;

  std::map<std::string, std::array<double, 3>> carla_vehicle_param{
      {"vehicle.tesla.model3",
       {4.791779518127441, 2.163450002670288, 1.488319993019104}},
      {"vehicle.bmw.grandtourer",
       {4.611005783081055, 2.241713285446167, 1.6672759056091309}}};

  auto it = carla_vehicle_param.find(ego_info_->type);
  if (it == carla_vehicle_param.end()) {
    LOG(FATAL) << "information for " << ego_info_->type
               << " has not implemented yet";
    return false;
  }
  vehicle_param->length = it->second[0];
  vehicle_param->width = it->second[1];
  vehicle_param->height = it->second[2];
  vehicle_param->front_to_front_axle =
      vehicle_param->length / 2.0 - wheels[0].position.x;
  vehicle_param->back_to_rear_axle =
      vehicle_param->length / 2.0 + wheels[2].position.x;
  vehicle_param->rear_axle_to_center =
      vehicle_param->length / 2.0 - vehicle_param->back_to_rear_axle;

  ego_info_sub_.shutdown();
  return true;
}
}  // namespace simulation
}  // namespace planning
