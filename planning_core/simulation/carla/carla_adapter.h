/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <mutex>
#include <thread>

#include "planning_core/simulation/carla/ego_info/carla_ego_info.h"
#include "planning_core/simulation/carla/mock_perception/carla_mock_perception.h"
#include "planning_core/simulation/controller/mpc_controller.h"
#include "planning_core/simulation/simulator_adapter.h"

namespace planning {
namespace simulation {

class CarlaAdapter final : public SimulatorAdapter {
 public:
  CarlaAdapter();

  void Init() override;
  std::string Name() const { return "Carla"; }
  bool InitVehicleParam(VehicleParam* vehicle_param) override;
  bool UpdateEgoState(common::State* state) override;
  bool UpdatePerceptionResults(std::vector<Obstacle>* obstacles) override;
  void SetTrajectory(const common::Trajectory& trajectory) override;

 protected:
  void ControlLoop();

 private:
  CarlaEgoInfo carla_ego_info_;
  CarlaMockPerception carla_mock_perception_;

  // control
  ros::Publisher control_cmd_pub_;
  std::mutex control_mutex_;

  double wheel_base_ = 0.0;

  common::Trajectory trajectory_;
  MpcController mpc_controller_;
};
}  // namespace simulation
}  // namespace planning
