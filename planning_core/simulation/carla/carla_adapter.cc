/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/carla/carla_adapter.h"

#include <ackermann_msgs/AckermannDrive.h>
#include <glog/logging.h>

#include <thread>

#include "common/utils/math.h"

namespace planning {
namespace simulation {

CarlaAdapter::CarlaAdapter() {}

void CarlaAdapter::Init() {
  carla_ego_info_.Init();
  carla_mock_perception_.Init();

  ros::NodeHandle node;
  control_cmd_pub_ = node.advertise<ackermann_msgs::AckermannDrive>(
      "carla/ego_vehicle/ackermann_cmd", 1);
  mpc_controller_.Init();
};

bool CarlaAdapter::InitVehicleParam(VehicleParam* vehicle_param) {
  for (int i = 0; i < 10 && ros::ok(); ++i) {
    ros::spinOnce();
    if (carla_ego_info_.GetVehicleParam(vehicle_param)) {
      wheel_base_ = vehicle_param->wheel_base;
      LOG(INFO) << "\n" << vehicle_param->DebugString();
      return true;
    }
    LOG(WARNING) << "fail to get vehicle param from Carla, Retry: " << i;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return false;
}

bool CarlaAdapter::UpdateEgoState(common::State* state) {
  return carla_ego_info_.UpdateEgoState(state);
}

bool CarlaAdapter::UpdatePerceptionResults(std::vector<Obstacle>* obstacles) {
  return carla_mock_perception_.UpdateMockPerceptionResult(obstacles);
}

void CarlaAdapter::SetTrajectory(const common::Trajectory& trajectory) {
  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  CHECK_GT(wheel_base_, 2);
  mpc_controller_.set_wheel_base(wheel_base_);
  carla_ego_info_.UpdateEgoState(&ego_state);
  mpc_controller_.CalculateAckermannDrive(ego_state, trajectory,
                                          &ackermann_drive);
  control_cmd_pub_.publish(ackermann_drive);
}

void CarlaAdapter::ControlLoop() {
  ros::Rate rate(20);
  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  mpc_controller_.set_wheel_base(wheel_base_);

  while (ros::ok()) {
    carla_ego_info_.UpdateEgoState(&ego_state);
    mpc_controller_.CalculateAckermannDrive(ego_state, trajectory_,
                                            &ackermann_drive);
    control_cmd_pub_.publish(ackermann_drive);
    rate.sleep();
  }
}
}  // namespace simulation
}  // namespace planning
