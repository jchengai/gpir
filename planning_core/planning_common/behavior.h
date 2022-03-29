/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

namespace planning {

enum class LaneChangeStatus {
  kIdle = 0,
  kTryLaneChange = 1,
  kLaneChanging = 2,
  kLaneChangeFailed = 3,
  kLaneChangeCanceled = 4
};

using LCStatus = LaneChangeStatus;

}  // namespace planning
