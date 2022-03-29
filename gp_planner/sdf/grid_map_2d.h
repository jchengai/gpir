/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <glog/logging.h>

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "common/base/type.h"

namespace planning {

/**
 * @class GridMap2D
 * @brief origin is the left-bottom cornor
 *   y
 *   ^
 *   |________________
 *   |_*_|_*_|_*_|_*_|
 *   |_*_|_*_|_*_|_*_|
 *   |_*_|_*_|_*_|_*_|
 *   |_*_|_*_|_*_|_*_|------->x
 *  o
 */

template <typename T>
class GridMap2D {
 public:
  GridMap2D() = default;
  GridMap2D(const GridMap2D<T>& grid_map);
  GridMap2D(GridMap2D<T>&& grid_map);
  GridMap2D(std::array<double, 2> origin, std::array<int, 2> cell_num,
            std::array<double, 2> resolution);
  ~GridMap2D() = default;

  void ResetMap();

  template <typename M>
  inline void ResizeFrom(const GridMap2D<M>& grid_map);

  inline std::array<double, 2> origin() const { return origin_; }
  inline std::array<double, 2> resolution() const { return cell_resolution_; }
  inline std::array<int, 2> cell_num() const { return cell_num_; }
  inline const std::vector<T>& data() const { return map_data_; }
  inline std::vector<T>* mutable_data() { return &map_data_; }

  inline void set_origin(std::array<double, 2> origin);
  inline void set_data(const std::vector<T>& data,
                       const std::array<int, 2>& cell_num);
  inline void set_resolution(const std::array<double, 2>& resolution);
  inline void set_cell_number(std::array<int, 2> cell_num);

  inline void BoundIndex(Eigen::Vector2i* index) const;
  inline int Index2Address(const int x, const int y) const;
  inline int Index2Address(const Eigen::Vector2i& index) const;
  inline void Coordinate2Index(const Eigen::Vector2d& coord,
                               Eigen::Vector2i* index) const;
  inline void Index2Coordinate(const Eigen::Vector2i& index,
                               Eigen::Vector2d* coord) const;

  inline bool IsInMap(const Eigen::Vector2i& index) const;
  inline bool IsInMap(const Eigen::Vector2d& coord) const;

  inline T GetValue(const Eigen::Vector2i& index) const;
  inline T GetValue(const int index_x, const int index_y) const;
  inline T GetValueSafe(const Eigen::Vector2i& index) const;
  inline double GetValueBilinear(const Eigen::Vector2d& coord,
                                 Eigen::Vector2d* grad = nullptr) const;
  inline void SetValue(const Eigen::Vector2i& index, const T value);
  inline void SetValue(const int index_x, const int index_y, const T val);

  inline bool IsOccupied(const Eigen::Vector2i& index) const;
  inline bool IsOccupied(const int index_x, const int index_y) const;
  inline bool IsOccupied(const Eigen::Vector2d& coord) const;

  // only used for debug and visualization, data is copied on every call
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix()
      const;
  cv::Mat ImageSec() const;
  cv::Mat BinaryImage() const;

 public:
  void FillCircle(const Eigen::Vector2d& center, double radius);
  void FillPoly(const vector_Eigen<Eigen::Vector2d>& points);
  // much faster than FillPoly
  void FillConvexPoly(const vector_Eigen<Eigen::Vector2d>& points);
  void PolyLine(const vector_Eigen<Eigen::Vector2d>& points);
  void FillEntireRow(const double y_coord);

 public:
  void SearchForVerticalBoundaries(
      const std::vector<double> x_coords,
      std::vector<std::vector<std::pair<double, double>>>* vertical_boundaries)
      const;

  void FindVerticalBoundary(const double x, const double y, double* lb,
                            double* ub) const;

 protected:
  inline void InitStepAndSize();

 private:
  std::array<int, 2> cell_num_;
  std::array<double, 2> cell_resolution_{1.0, 1.0};
  std::array<double, 2> cell_resolution_inv_{1.0, 1.0};
  std::array<int, 2> cell_step_{1};
  std::array<double, 2> origin_{0.0, 0.0};

  int map_size_ = 1;
  std::vector<T> map_data_;
};

// inline function
template <typename T>
GridMap2D<T>::GridMap2D(std::array<double, 2> origin,
                        std::array<int, 2> cell_num,
                        std::array<double, 2> resolution) {
  set_origin(origin);
  set_cell_number(cell_num);
  set_resolution(resolution);
}

template <typename T>
GridMap2D<T>::GridMap2D(const GridMap2D<T>& grid_map) {
  set_origin(grid_map.origin());
  set_data(grid_map.data(), grid_map.cell_num());
  set_resolution(grid_map.resolution());
}

template <typename T>
GridMap2D<T>::GridMap2D(GridMap2D<T>&& grid_map)
    : map_data_(std::move(grid_map.map_data_)) {
  cell_num_ = grid_map.cell_num_;
  cell_resolution_ = grid_map.cell_resolution_;
  cell_resolution_inv_ = grid_map.cell_resolution_inv_;
  origin_ = grid_map.origin_;
  cell_step_ = grid_map.cell_step_;
  map_size_ = grid_map.map_size_;

  grid_map.map_size_ = 0;
}

template <typename T>
void GridMap2D<T>::ResetMap() {
  // std::fill(map_data_.begin(), map_data_.end(), static_cast<T>(0));
  if (!map_data_.empty()) {
    std::memset(&map_data_[0], static_cast<T>(0),
                sizeof(map_data_[0]) * map_data_.size());
  }
}

template <typename T>
template <typename M>
inline void GridMap2D<T>::ResizeFrom(const GridMap2D<M>& grid_map) {
  set_origin(grid_map.origin());
  set_cell_number(grid_map.cell_num());
  set_resolution(grid_map.resolution());
}

template <typename T>
inline void GridMap2D<T>::InitStepAndSize() {
  map_size_ = cell_num_[0] * cell_num_[1];
  cell_step_[0] = 1;
  cell_step_[1] = cell_num_[0];
}

template <typename T>
inline void GridMap2D<T>::set_origin(std::array<double, 2> origin) {
  origin_ = origin;
}

template <typename T>
inline void GridMap2D<T>::set_cell_number(std::array<int, 2> cell_num) {
  cell_num_ = cell_num;
  InitStepAndSize();
  map_data_ = std::vector<T>(map_size_, static_cast<T>(0));
}

template <typename T>
inline void GridMap2D<T>::set_data(const std::vector<T>& data,
                                   const std::array<int, 2>& cell_num) {
  cell_num_ = cell_num;
  map_data_ = data;
  InitStepAndSize();
}

template <typename T>
inline void GridMap2D<T>::set_resolution(
    const std::array<double, 2>& resolution) {
  cell_resolution_ = resolution;
  cell_resolution_inv_[0] = 1.0 / cell_resolution_[0];
  cell_resolution_inv_[1] = 1.0 / cell_resolution_[1];
}

template <typename T>
inline int GridMap2D<T>::Index2Address(const Eigen::Vector2i& index) const {
  return index(0) * cell_step_[0] + index(1) * cell_step_[1];
}

template <typename T>
inline int GridMap2D<T>::Index2Address(const int x, const int y) const {
  return x * cell_step_[0] + y * cell_step_[1];
}

template <typename T>
inline void GridMap2D<T>::BoundIndex(Eigen::Vector2i* index) const {
  (*index)(0) = std::max(std::min((*index)(0), cell_num_[0] - 1), 0);
  (*index)(1) = std::max(std::min((*index)(1), cell_num_[1] - 1), 0);
}

template <typename T>
inline void GridMap2D<T>::Coordinate2Index(const Eigen::Vector2d& coord,
                                           Eigen::Vector2i* index) const {
  (*index)(0) = std::floor((coord(0) - origin_[0]) * cell_resolution_inv_[0]);
  (*index)(1) = std::floor((coord(1) - origin_[1]) * cell_resolution_inv_[1]);
}

template <typename T>
inline void GridMap2D<T>::Index2Coordinate(const Eigen::Vector2i& index,
                                           Eigen::Vector2d* coord) const {
  (*coord)(0) = (index(0) + 0.5) * cell_resolution_[0] + origin_[0];
  (*coord)(1) = (index(1) + 0.5) * cell_resolution_[1] + origin_[1];
}

template <typename T>
inline bool GridMap2D<T>::IsInMap(const Eigen::Vector2i& index) const {
  if (index(0) < 0 || index(1) < 0 || index(0) > cell_num_[0] - 1 ||
      index(1) > cell_num_[1] - 1)
    return false;
  else
    return true;
}

template <typename T>
inline bool GridMap2D<T>::IsInMap(const Eigen::Vector2d& coord) const {
  Eigen::Vector2i index;
  Coordinate2Index(coord, &index);
  return IsInMap(index);
}

template <typename T>
inline bool GridMap2D<T>::IsOccupied(const Eigen::Vector2i& index) const {
  return map_data_[Index2Address(index)] > 0;
}

template <typename T>
inline bool GridMap2D<T>::IsOccupied(const int x, const int y) const {
  return map_data_[Index2Address(x, y)] > 0;
}

template <typename T>
inline bool GridMap2D<T>::IsOccupied(const Eigen::Vector2d& coord) const {
  Eigen::Vector2i index;
  Coordinate2Index(coord, &index);
  BoundIndex(&index);
  return map_data_[Index2Address(index)] > 0;
}

template <typename T>
inline T GridMap2D<T>::GetValue(const Eigen::Vector2i& index) const {
  return map_data_[Index2Address(index)];
}

template <typename T>
inline T GridMap2D<T>::GetValueSafe(const Eigen::Vector2i& index) const {
  auto index_tmp = index;
  BoundIndex(&index_tmp);
  return map_data_[Index2Address(index_tmp)];
}

template <typename T>
inline T GridMap2D<T>::GetValue(const int x, const int y) const {
  return map_data_[Index2Address(x, y)];
}

template <typename T>
inline double GridMap2D<T>::GetValueBilinear(const Eigen::Vector2d& coord,
                                             Eigen::Vector2d* grad) const {
  if (!IsInMap(coord)) {
    // LOG(WARNING) << "query out of map, "
    //              << "(" << coord.x() << ", " << coord.y() << ")";
    return static_cast<T>(0);
  }

  Eigen::Vector2d coord_tmp(coord(0) - 0.5 * cell_resolution_[0],
                            coord(1) - 0.5 * cell_resolution_[1]);

  Eigen::Vector2i index_lb;
  Coordinate2Index(coord_tmp, &index_lb);
  BoundIndex(&index_lb);

  Eigen::Vector2d coord_lb, diff;
  Index2Coordinate(index_lb, &coord_lb);

  diff(0) = (coord(0) - coord_lb(0)) * cell_resolution_inv_[0];
  diff(1) = (coord(1) - coord_lb(1)) * cell_resolution_inv_[1];

  T value[2][2];
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      value[x][y] = GetValueSafe(index_lb + Eigen::Vector2i(x, y));
    }
  }

  double y0 = (1 - diff(0)) * value[0][0] + diff(0) * value[1][0];
  double y1 = (1 - diff(0)) * value[0][1] + diff(0) * value[1][1];
  double x0 = (1 - diff(1)) * value[0][0] + diff(1) * value[0][1];
  double x1 = (1 - diff(1)) * value[1][0] + diff(1) * value[1][1];

  if (grad) {
    (*grad)(0) = (x1 - x0) * cell_resolution_inv_[0];
    (*grad)(1) = (y1 - y0) * cell_resolution_inv_[1];
  }

  return (1 - diff(1)) * y0 + diff(1) * y1;
}

template <typename T>
inline void GridMap2D<T>::SetValue(const Eigen::Vector2i& index,
                                   const T value) {
  map_data_[Index2Address(index)] = value;
}

template <typename T>
inline void GridMap2D<T>::SetValue(const int x, const int y, const T value) {
  map_data_[Index2Address(x, y)] = value;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
GridMap2D<T>::Matrix() const {
  auto copy_data = map_data_;
  auto M = Eigen::Map<
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      copy_data.data(), cell_num_[1], cell_num_[0]);
  // since the origin is the left-bottom cornerd
  return M.colwise().reverse();
}

template <typename T>
cv::Mat GridMap2D<T>::ImageSec() const {
  auto copy_data = map_data_;
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), copy_data.data());
  cv::Mat g_result_map, c_result_map, c_filped_map;

  double scale, min, max;
  cv::minMaxIdx(grid_mat, &min, &max);
  scale = 255.0 / std::abs(max - min);

  grid_mat.convertTo(g_result_map, CV_8UC1, scale, -scale * min);
  cv::applyColorMap(g_result_map, c_result_map, cv::COLORMAP_JET);
  // since the origin is the left-bottom cornerd, flip around x-axis
  cv::flip(c_result_map, c_filped_map, 0);
  return c_filped_map;
}

template <typename T>
cv::Mat GridMap2D<T>::BinaryImage() const {
  auto copy_data = map_data_;
  cv::Mat grid_mat(cell_num_[1], cell_num_[0],
                   CV_MAKETYPE(cv::DataType<T>::type, 1), copy_data.data());
  cv::Mat binary_map, fliped_map;
  cv::threshold(grid_mat, binary_map, 0.5, 200, cv::THRESH_BINARY_INV);
  cv::flip(binary_map, fliped_map, 0);
  return fliped_map;
}

template <typename T>
void GridMap2D<T>::SearchForVerticalBoundaries(
    const std::vector<double> x_coords,
    std::vector<std::vector<std::pair<double, double>>>* vertical_boundaries)
    const {
  vertical_boundaries->clear();

  for (const auto x_coord : x_coords) {
    const int x = std::floor((x_coord - origin_[0]) * cell_resolution_inv_[0]);

    std::vector<std::pair<double, double>> boundary;

    int lb = 0;
    bool is_valid_boundary = false;

    for (int y = 0; y < cell_num_[1]; ++y) {
      const int address = Index2Address(x, y);
      if (!is_valid_boundary) {
        if (map_data_[address] == static_cast<T>(0)) {
          is_valid_boundary = true;
          lb = y;
        }
      } else {
        if (map_data_[address] > static_cast<T>(0) || y == cell_num_[1] - 1) {
          // convert to coordinate
          boundary.emplace_back(std::make_pair<double, double>(
              (lb + 0.5) * cell_resolution_[1] + origin_[1],
              (y - 1) * cell_resolution_[1] + origin_[1]));
          is_valid_boundary = false;
        }
      }
    }
    vertical_boundaries->emplace_back(std::move(boundary));
  }
}

template <typename T>
void GridMap2D<T>::FindVerticalBoundary(const double x, const double y,
                                        double* lb, double* ub) const {
  const int x_idx = std::floor((x - origin_[0]) * cell_resolution_inv_[0]);
  const int y_idx = std::floor((y - origin_[1]) * cell_resolution_inv_[1]);
  *lb = y, *ub = y;
  int address = 0;

  int current_y_idx = y_idx + 1;
  while (current_y_idx < cell_num_[1]) {
    address = Index2Address(x_idx, current_y_idx);
    if (map_data_[address] == static_cast<T>(0)) {
      ++current_y_idx;
      *ub += cell_resolution_[1];
    } else {
      break;
    }
  }

  current_y_idx = y_idx - 1;
  while (current_y_idx >= 0) {
    address = Index2Address(x_idx, current_y_idx);
    if (map_data_[address] == static_cast<T>(0)) {
      --current_y_idx;
      *lb -= cell_resolution_inv_[1];
    } else {
      break;
    }
  }
}
}  // namespace planning