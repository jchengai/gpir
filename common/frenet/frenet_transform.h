/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "common/frenet/frenet_state.h"

namespace common {
class FrenetTransfrom {
 public:
  static void StateToFrenetState(const State& state,
                                 const FrenetReferencePoint& ref,
                                 FrenetState* frenet_state);

  // static void SpeedProfileToFrenet(const Eigen::Vector3d& s, const
  // FrenetReferencePoint& ref, Eigen::Vector3d)

  static void FrenetStateToState(const FrenetState& frenet_state,
                                 const FrenetReferencePoint& ref, State* state);

  static void LateralFrenetStateToState(const Eigen::Vector3d& d,
                                        const FrenetReferencePoint& ref,
                                        common::State* state);

  static double GetCurvature(const std::array<double, 3>& d,
                             const double kappa_r, const double dkappa_r);
};

}  // namespace common
