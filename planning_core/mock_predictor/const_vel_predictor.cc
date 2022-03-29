/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/mock_predictor/const_vel_predictor.h"

#include <glog/logging.h>

#include "common/utils/timer.h"
#include "hdmap/hdmap.h"
#include "hdmap/road_network/lane_map.h"

namespace planning {

bool ConstVelPredictor::GeneratePrediction(std::vector<Obstacle>* obstacles) {
  UpdateVehicleLaneMap(*obstacles);
  for (size_t i = 0; i < obstacles->size(); ++i) {
    Obstacle& obstacle = (*obstacles)[i];
    if (obstacle.is_static()) continue;

    const auto& state = obstacle.state();
    auto prediction = obstacle.mutable_prediction();
    auto lane_id_list = obstacle.mutable_lane_id_list();
    prediction->emplace_back(state);

    auto lane_id = vehicle_lane_map_[obstacle.id()];
    if (lane_id == hdmap::LaneId(0)) {
      LOG(WARNING) << "fail to get lane info for obstacle " << obstacle.id();
      continue;
    }

    lane_id_list->emplace_back(lane_id);
    auto lane = hdmap::LaneMap::GetLane(lane_id);
    auto lane_length = lane->length();
    auto sl_pair = lane->GetProjection(state.position);

    double current_l = sl_pair.second;
    double current_s = sl_pair.first;

    const double l_decrease = current_l / (predict_num_ * 3);

    const double predict_length = prediction_horizon_ * state.velocity;
    const double segment_length = predict_length / predict_num_;

    common::State predict_state(state);
    for (int i = 1; i < predict_num_; ++i) {
      bool has_next_lane = true;
      while (current_s + segment_length > lane_length) {
        if (!lane->HasSuccessor()) {
          LOG(WARNING) << "do not has next lane";
          has_next_lane = false;
          break;
        }
        // TODO: maybe use better strategy
        lane_id = lane->next_lanes().front();
        lane = hdmap::LaneMap::GetLane(lane->next_lanes().front());
        current_s -= lane_length;
        lane_length = lane->length();
      }
      if (!has_next_lane) break;

      if (lane_id != lane_id_list->back()) {
        lane_id_list->emplace_back(lane_id);
      }

      current_s += segment_length;
      current_l -= l_decrease;
      auto waypoint = lane->GetWayPoint(current_s);

      Eigen::Vector2d tangent(-std::sin(waypoint.heading),
                              std::cos(waypoint.heading));

      common::State predict_state(state);
      predict_state.position = waypoint.point + tangent * current_l;
      predict_state.heading = waypoint.heading;
      predict_state.stamp = state.stamp + i * dt_;

      prediction->emplace_back(predict_state);
    }

    std::sort(lane_id_list->begin(), lane_id_list->end());
  }

  VisualizePrediction(*obstacles);
  return true;
}

}  // namespace planning
