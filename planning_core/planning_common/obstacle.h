/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Dense>

#include "common/base/state.h"
#include "common/geometry/box2d.h"

namespace planning {

class Obstacle {
 public:
  Obstacle() = default;

  inline int id() const { return id_; }
  inline void set_id(const int id) { id_ = id; }

  inline double speed() const { return state_.velocity; }
  inline double length() const { return bbox_.length(); }
  inline double width() const {return bbox_.width();}
  inline double height() const { return height_; }

  inline bool is_static() const { return is_static_; }
  inline void set_static(const bool is_static) { is_static_ = is_static; }

  inline const common::State& state() const { return state_; }
  inline common::State* mutable_state() { return &state_; }

  inline const std::vector<int>& lane_id_list() const { return lane_id_list_; }
  inline std::vector<int>* mutable_lane_id_list() { return &lane_id_list_; }

  inline const std::vector<common::State>& prediction() const {
    return predition_;
  }
  inline std::vector<common::State>* mutable_prediction() {
    return &predition_;
  }

  const common::Box2D& BoundingBox() const { return bbox_; }
  void SetBoundingBox(const double length, const double width,
                      const double height);

 private:
  int id_ = 0;
  bool is_static_ = false;

  double height_ = 0.0;  // only for visualization
  common::State state_;  // pos is the center of the obstacle
  common::Box2D bbox_;

  std::vector<int> lane_id_list_;
  std::vector<common::State> predition_;
};

}  // namespace planning
