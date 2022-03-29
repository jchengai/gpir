/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/sdf/signed_distance_field_2d.h"

#include <omp.h>

#include <future>
#include <iostream>

namespace planning {

SignedDistanceField2D::SignedDistanceField2D(std::array<double, 2> origin,
                                             std::array<int, 2> dim,
                                             const double map_resolution)
    : map_resolution_(map_resolution) {
  occupancy_map_.set_origin(origin);
  occupancy_map_.set_cell_number(dim);
  occupancy_map_.set_resolution(
      std::array<double, 2>{map_resolution, map_resolution});
  esdf_.ResizeFrom(occupancy_map_);
}

SignedDistanceField2D::SignedDistanceField2D(OccupancyMap&& occupancy_map)
    : occupancy_map_(std::move(occupancy_map)) {
  esdf_.ResizeFrom(occupancy_map_);
  map_resolution_ = occupancy_map_.resolution()[0];
}

void SignedDistanceField2D::EuclideanDistanceTransform(
    std::array<int, 2> dim,
    std::function<bool(const int x, const int y)> is_occupied,
    DistanceMap* output_map) {
  int inf = dim[0] + dim[1] + 10;

  std::vector<std::vector<int>> g(dim[0], std::vector<int>(dim[1], 0));
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    // column scan
    for (int x = 0; x < dim[0]; ++x) {
      g[x][0] = is_occupied(x, 0) ? 0 : inf;

      for (int y = 1; y < dim[1]; ++y) {
        g[x][y] = is_occupied(x, y) ? 0 : 1 + g[x][y - 1];
      }

      for (int y = dim[1] - 2; y >= 0; --y) {
        if (g[x][y + 1] < g[x][y]) g[x][y] = 1 + g[x][y + 1];
      }
    }
  }

  // row scan
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int y = 0; y < dim[1]; ++y) {
      int q = 0, w;
      std::vector<int> s(dim[0], 0);
      std::vector<int> t(dim[0], 0);

      auto f = [&g, &y](int x, int i) -> double {
        return (x - i) * (x - i) + g[i][y] * g[i][y];
      };

      for (int u = 1; u < dim[0]; ++u) {
        while (q >= 0 && f(t[q], s[q]) > f(t[q], u)) {
          --q;
        }

        if (q < 0) {
          q = 0;
          s[0] = u;
        } else {
          w = 1 + std::floor((u * u - s[q] * s[q] + g[u][y] * g[u][y] -
                              g[s[q]][y] * g[s[q]][y]) /
                             (2 * (u - s[q])));
          if (w < dim[0]) {
            ++q;
            s[q] = u;
            t[q] = w;
          }
        }
      }

      for (int u = dim[0] - 1; u >= 0; --u) {
        output_map->SetValue(u, y, map_resolution_ * std::sqrt(f(u, s[q])));
        if (u == t[q]) --q;
      }
    }
  }
}

void SignedDistanceField2D::VerticalEuclideanDistanceTransform(
    std::array<int, 2> dim,
    std::function<bool(const int x, const int y)> is_occupied,
    DistanceMap* output_map) {
  int inf = 1e9;

  std::vector<std::vector<int>> g(dim[0], std::vector<int>(dim[1], 0));
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    // column scan
    for (int x = 0; x < dim[0]; ++x) {
      g[x][0] = is_occupied(x, 0) ? 0 : inf;

      for (int y = 1; y < dim[1]; ++y) {
        g[x][y] = is_occupied(x, y) ? 0 : 1 + g[x][y - 1];
      }

      for (int y = dim[1] - 2; y >= 0; --y) {
        if (g[x][y + 1] < g[x][y]) g[x][y] = 1 + g[x][y + 1];
      }

      for (int y = 0; y < dim[1]; ++y) {
        output_map->SetValue(x, y, map_resolution_ * g[x][y]);
      }
    }
  }
}

void SignedDistanceField2D::UpdateSDF() {
  DistanceMap distance_map, inv_distance_map;
  distance_map.ResizeFrom(occupancy_map_);
  inv_distance_map.ResizeFrom(occupancy_map_);
  OccupancyMap& map = occupancy_map_;

  auto dim = occupancy_map_.cell_num();
  EuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool { return map.IsOccupied(x, y); },
      &distance_map);
  EuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool {
        return !map.IsOccupied(x, y);
      },
      &inv_distance_map);

  const auto& dis_map_data = distance_map.data();
  const auto& inv_dis_map_data = inv_distance_map.data();
  auto& esdf_data = *esdf_.mutable_data();

  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int x = 0; x < dim[0]; ++x) {
      for (int y = 0; y < dim[1]; ++y) {
        int address = occupancy_map_.Index2Address(x, y);
        esdf_data[address] = dis_map_data[address];
        if (inv_dis_map_data[address] > 0) {
          esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
        }
      }
    }
  }
}

void SignedDistanceField2D::UpdateVerticalSDF() {
  DistanceMap distance_map, inv_distance_map;
  distance_map.ResizeFrom(occupancy_map_);
  inv_distance_map.ResizeFrom(occupancy_map_);
  OccupancyMap& map = occupancy_map_;

  auto dim = occupancy_map_.cell_num();
  VerticalEuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool { return map.IsOccupied(x, y); },
      &distance_map);
  VerticalEuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool {
        return !map.IsOccupied(x, y);
      },
      &inv_distance_map);

  const auto& dis_map_data = distance_map.data();
  const auto& inv_dis_map_data = inv_distance_map.data();
  auto& esdf_data = *esdf_.mutable_data();

  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int x = 0; x < dim[0]; ++x) {
      for (int y = 0; y < dim[1]; ++y) {
        int address = occupancy_map_.Index2Address(x, y);
        esdf_data[address] = dis_map_data[address];
        if (inv_dis_map_data[address] > 0) {
          esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
        }
      }
    }
  }
}

}  // namespace planning
