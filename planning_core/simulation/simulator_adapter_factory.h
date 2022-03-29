/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>

#include "planning_core/simulation/simulator_adapter.h"

namespace planning {
namespace simulation {

class SimulatorFactory {
 public:
  static std::unique_ptr<SimulatorAdapter> CreateSimulatorAdapter(
      const std::string& type);
};

}  // namespace simulation
}  // namespace planning
