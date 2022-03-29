/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/sdf/grid_map_2d.h"

namespace planning {

template <typename T>
void GridMap2D<T>::FillCircle(const Eigen::Vector2d& center, double radius) {
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), map_data_.data());

  Eigen::Vector2i index;
  Coordinate2Index(center, &index);

  int grid_radius = radius * cell_resolution_inv_[0];
  cv::Point2i cv_center(index(0), index(1));
  cv::circle(grid_mat, cv_center, grid_radius, cv::Scalar(1), -1);
}

template <typename T>
void GridMap2D<T>::FillPoly(const vector_Eigen<Eigen::Vector2d>& points) {
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), map_data_.data());

  Eigen::Vector2i index;
  std::vector<cv::Point2i> poly;
  for (const auto& p : points) {
    Coordinate2Index(p, &index);
    poly.emplace_back(cv::Point2i(index(0), index(1)));
  }
  std::vector<std::vector<cv::Point2i>> polys;
  polys.emplace_back(poly);
  cv::fillPoly(grid_mat, polys, cv::Scalar(1));
}

template <typename T>
void GridMap2D<T>::FillConvexPoly(const vector_Eigen<Eigen::Vector2d>& points) {
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), map_data_.data());

  Eigen::Vector2i index;
  std::vector<cv::Point2i> convex_poly;
  for (const auto& p : points) {
    Coordinate2Index(p, &index);
    convex_poly.emplace_back(cv::Point2i(index(0), index(1)));
  }
  cv::fillConvexPoly(grid_mat, convex_poly, cv::Scalar(static_cast<T>(1)));
}

template <typename T>
void GridMap2D<T>::PolyLine(const vector_Eigen<Eigen::Vector2d>& points) {
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), map_data_.data());
  Eigen::Vector2i index;
  std::vector<cv::Point2i> polyline;
  for (const auto& p : points) {
    Coordinate2Index(p, &index);
    polyline.emplace_back(cv::Point2i(index(0), index(1)));
  }
  std::vector<std::vector<cv::Point2i>> polylines;
  polylines.emplace_back(std::move(polyline));
  cv::polylines(grid_mat, polylines, false, cv::Scalar(static_cast<T>(1)));
}

template <typename T>
void GridMap2D<T>::FillEntireRow(const double y_coord) {
  const int row_index =
      std::floor((y_coord - origin_[1]) * cell_resolution_inv_[1]);
  if (row_index < 0 || row_index > cell_num_[1] - 1) return;

  int address = row_index * cell_num_[0];
  std::fill(&map_data_[address], &map_data_[address] + cell_num_[0],
            static_cast<T>(1));
}

template class GridMap2D<int>;
template class GridMap2D<uint8_t>;
}  // namespace planning