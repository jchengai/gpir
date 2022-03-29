/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/simulator_adapter_factory.h"

#include <glog/logging.h>

#include <memory>

#include "planning_core/simulation/carla/carla_adapter.h"

namespace planning {
namespace simulation {

std::unique_ptr<SimulatorAdapter> SimulatorFactory::CreateSimulatorAdapter(
    const std::string& type) {
  if (type == "Carla") {
    return std::move(std::make_unique<CarlaAdapter>());
  } else {
    LOG(FATAL) << "simulator adapter " << type << " has not implemented yet";
  }
  return nullptr;
}

}  // namespace simulation
}  // namespace planning
