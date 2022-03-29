/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#pragma once

#include <vector>

#include "opendrive/geometry/Geometry.h"
#include "opendrive/types.hpp"

namespace opendrive {
namespace geometry {

struct CenterLine {
  using GeometryVector = std::vector<std::unique_ptr<geometry::Geometry>>;
  using LaneOffsetVector = std::vector<LaneOffset>;

  GeometryVector geometry;

  LaneOffsetVector offsetVector;

  double length{0.};

  /**
   * @brief Evalues the center line at the given position along the curve and
   * returns a DirectedPoint which contains the position and the tangent vector.
   * If applyLateralOffset is true, it will apply a lateral offset as defined by
   * the lateral offset vector.
   */
  geometry::DirectedPoint eval(double s, bool applyLateralOffset = true) const;

  /**
   * @brief Calculates the center line offset at the given position along the
   * curve.
   */
  double calculateOffset(double s) const;

  /**
   * @brief Generates a list of sampling points along the center line between
   * [0, arclength]. The sampling points are optimized depending of the type of
   * geometry of the inner segments.
   */
  std::vector<double> samplingPoints() const;
};

/**
 * @brief Generates a center line object using the road geometry definition.
 * Returns false if invalid geometry definitions are present.
 */
bool generateCenterLine(RoadInformation &roadInfo, CenterLine &centerLine);
}  // namespace geometry
}  // namespace opendrive
