/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "planning_core/planning_common/obstacle.h"
#include "planning_core/planning_common/vehicle_info.h"

namespace planning {
namespace simulation {

class SimulatorAdapter {
 public:
  SimulatorAdapter() = default;
  virtual ~SimulatorAdapter() = default;

  virtual void Init() = 0;
  virtual std::string Name() const = 0;
  virtual bool InitVehicleParam(VehicleParam* vehicle_param) = 0;
  virtual bool UpdateEgoState(common::State* state) = 0;
  virtual bool UpdatePerceptionResults(std::vector<Obstacle>* obstacles) = 0;
  virtual void SetTrajectory(const common::Trajectory& trajectory) = 0;
};

enum class SimulatorType {
  kCarla = 0,
};
}  // namespace simulation
}  // namespace planning
