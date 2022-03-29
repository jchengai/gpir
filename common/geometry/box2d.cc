/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "common/geometry/box2d.h"

namespace common {

Box2D::Box2D(const Eigen::Vector2d& center, const double length,
             const double width, const double angle, const double height)
    : center_(center),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      angle_(angle),
      height_(height) {
  CalculateCorners();
}

void Box2D::CalculateCorners() {
  cos_angle_ = std::cos(angle_);
  sin_angle_ = std::sin(angle_);
  helper_[0] = cos_angle_ * half_length_;
  helper_[1] = sin_angle_ * half_length_;
  helper_[2] = sin_angle_ * half_width_;
  helper_[3] = -cos_angle_ * half_width_;

  corners_.clear();
  corners_.emplace_back(center_.x() + helper_[0] + helper_[2],
                        center_.y() + helper_[1] + helper_[3]);
  corners_.emplace_back(center_.x() + helper_[0] - helper_[2],
                        center_.y() + helper_[1] - helper_[3]);
  corners_.emplace_back(center_.x() - helper_[0] - helper_[2],
                        center_.y() - helper_[1] - helper_[3]);
  corners_.emplace_back(center_.x() - helper_[0] + helper_[2],
                        center_.y() - helper_[1] + helper_[3]);

  auto x_minmax = std::minmax_element(
      corners_.cbegin(), corners_.cend(),
      [](const Eigen::Vector2d& p1, const Eigen::Vector2d p2) -> bool {
        return p1.x() < p2.x();
      });

  auto y_minmax = std::minmax_element(
      corners_.cbegin(), corners_.cend(),
      [](const Eigen::Vector2d& p1, const Eigen::Vector2d p2) -> bool {
        return p1.y() < p2.y();
      });

  xlim_[0] = x_minmax.first->x();
  xlim_[1] = x_minmax.second->x();
  ylim_[0] = y_minmax.first->y();
  ylim_[1] = y_minmax.second->y();
}

bool Box2D::HasOverlapWith(const Box2D& box) {
  if (box.xlim_[1] < xlim_[0] || box.xlim_[0] > xlim_[1] ||
      box.ylim_[1] < ylim_[0] || box.ylim_[0] > ylim_[1]) {
    return false;
  }

  const double dx = box.center_.x() - center_.x();
  const double dy = box.center_.y() - center_.y();

  return std::abs(dx * cos_angle_ + dy * sin_angle_) <=
             std::abs(box.helper_[0] * cos_angle_ +
                      box.helper_[1] * sin_angle_) +
                 std::abs(box.helper_[2] * cos_angle_ +
                          box.helper_[3] * sin_angle_) +
                 half_length_ &&
         std::abs(dx * sin_angle_ - dy * cos_angle_) <=
             std::abs(box.helper_[0] * sin_angle_ -
                      box.helper_[1] * cos_angle_) +
                 std::abs(box.helper_[2] * sin_angle_ -
                          box.helper_[3] * cos_angle_) +
                 half_width_ &&
         std::abs(dx * box.cos_angle_ + dy * box.sin_angle_) <=
             std::abs(helper_[0] * box.cos_angle_ +
                      helper_[1] * box.sin_angle_) +
                 std::abs(helper_[2] * box.cos_angle_ +
                          helper_[3] * box.sin_angle_) +
                 box.half_length_ &&
         std::abs(dx * box.sin_angle_ - dy * box.cos_angle_) <=
             std::abs(helper_[0] * box.sin_angle_ -
                      helper_[1] * box.cos_angle_) +
                 std::abs(helper_[2] * box.sin_angle_ -
                          helper_[3] * box.cos_angle_) +
                 box.half_width_;
}
}  // namespace common
