/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "common/frenet/frenet_transform.h"

#include <glog/logging.h>

#include "common/utils/math.h"

namespace common {

constexpr double kEpsilon = 1e-6;

void FrenetTransfrom::StateToFrenetState(const State& state,
                                         const FrenetReferencePoint& ref,
                                         FrenetState* frenet_state) {
  Eigen::Vector2d normal(-std::sin(ref.theta), std::cos(ref.theta));
  frenet_state->d[0] = (state.position - ref.point).dot(normal);

  const double one_minus_kappa_rd = 1 - ref.kappa * frenet_state->d[0];

  const double delta_theta = NormalizeAngle(state.heading - ref.theta);
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);
  frenet_state->d[1] = one_minus_kappa_rd * tan_delta_theta;
  const double dkappa_d_kappa_dd =
      ref.dkappa * frenet_state->d[0] + ref.kappa * frenet_state->d[1];
  frenet_state->d[2] =
      -dkappa_d_kappa_dd * tan_delta_theta +
      one_minus_kappa_rd / (cos_delta_theta * cos_delta_theta) *
          (ref.kappa * one_minus_kappa_rd / cos_delta_theta - ref.kappa);

  frenet_state->s[0] = ref.s;
  frenet_state->s[1] = state.velocity * cos_delta_theta / one_minus_kappa_rd;

  const double d_delta_theta =
      1.0 / (1.0 + tan_delta_theta * tan_delta_theta) *
      (frenet_state->d[2] * one_minus_kappa_rd +
       frenet_state->d[1] * frenet_state->d[1] * ref.kappa) /
      (one_minus_kappa_rd * one_minus_kappa_rd);

  frenet_state->s[2] =
      (state.acceleration * cos_delta_theta -
       frenet_state->s[1] * frenet_state->s[1] *
           (frenet_state->d[1] * d_delta_theta - dkappa_d_kappa_dd)) /
      one_minus_kappa_rd;
}

void FrenetTransfrom::FrenetStateToState(const FrenetState& frenet_state,
                                         const FrenetReferencePoint& ref,
                                         State* state) {
  Eigen::Vector2d normal(-std::sin(ref.theta), std::cos(ref.theta));

  const double one_minus_kappa_rd = 1 - ref.kappa * frenet_state.d[0];

  const double tan_delta_theta = frenet_state.d[1] / one_minus_kappa_rd;
  const double delta_theta = std::atan2(frenet_state.d[1], one_minus_kappa_rd);
  const double cos_delta_theta = std::cos(delta_theta);

  state->s = ref.s;
  state->position = normal * frenet_state.d[0] + ref.point;
  state->velocity = frenet_state.s[1] * one_minus_kappa_rd / cos_delta_theta;
  state->heading = NormalizeAngle(delta_theta + ref.theta);

  const double dkappa_d_kappa_dd =
      ref.dkappa * frenet_state.d[0] + ref.kappa * frenet_state.d[1];
  state->kappa = (((frenet_state.d[2] + dkappa_d_kappa_dd * tan_delta_theta) *
                   (cos_delta_theta * cos_delta_theta) / one_minus_kappa_rd) +
                  ref.kappa) *
                 cos_delta_theta / one_minus_kappa_rd;

  const double d_delta_theta =
      1.0 / (1.0 + tan_delta_theta * tan_delta_theta) *
      (frenet_state.d[2] * one_minus_kappa_rd +
       frenet_state.d[1] * frenet_state.d[1] * ref.kappa) /
      (one_minus_kappa_rd * one_minus_kappa_rd);

  state->acceleration =
      frenet_state.s[2] * one_minus_kappa_rd / cos_delta_theta +
      frenet_state.s[1] * frenet_state.s[1] / cos_delta_theta *
          (frenet_state.d[1] - dkappa_d_kappa_dd);
}

void FrenetTransfrom::LateralFrenetStateToState(const Eigen::Vector3d& d,
                                                const FrenetReferencePoint& ref,
                                                common::State* state) {
  Eigen::Vector2d normal(-std::sin(ref.theta), std::cos(ref.theta));
  const double one_minus_kappa_rd = 1 - ref.kappa * d[0];
  const double tan_delta_theta = d[1] / one_minus_kappa_rd;
  const double delta_theta = std::atan2(d[1], one_minus_kappa_rd);
  const double cos_delta_theta = std::cos(delta_theta);
  const double dkappa_d_kappa_dd = ref.dkappa * d[0] + ref.kappa * d[1];

  state->s = ref.s;
  state->position = normal * d[0] + ref.point;
  state->heading = NormalizeAngle(delta_theta + ref.theta);
  state->kappa = (((d[2] + dkappa_d_kappa_dd * tan_delta_theta) *
                   (cos_delta_theta * cos_delta_theta) / one_minus_kappa_rd) +
                  ref.kappa) *
                 cos_delta_theta / one_minus_kappa_rd;
}

double FrenetTransfrom::GetCurvature(const std::array<double, 3>& d,
                                     const double kappa_r,
                                     const double dkappa_r) {
  const double one_minus_kappa_rd = 1 - kappa_r * d[0];
  const double delta_theta = std::atan2(d[1], one_minus_kappa_rd);
  const double tan_delta_theta = d[1] / one_minus_kappa_rd;
  const double cos_delta_theta = std::cos(delta_theta);
  const double dkappa_d_kappa_dd = dkappa_r * d[0] + kappa_r * d[1];
  return (((d[2] + dkappa_d_kappa_dd * tan_delta_theta) *
           (cos_delta_theta * cos_delta_theta) / one_minus_kappa_rd) +
          kappa_r) *
         cos_delta_theta / one_minus_kappa_rd;
}
}  // namespace common
