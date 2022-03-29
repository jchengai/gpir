// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/Types.hpp"

namespace ad {
namespace map {
namespace lane {

/*!
 * @brief calculate and update the lane length and width values
 */
struct IndexPairs
{
  std::vector<std::size_t> leftEdgeIndices;
  std::vector<std::size_t> rightEdgeIndices;
};

IndexPairs getIndexPairs(point::ENUEdge const &leftEdge, point::ENUEdge const &rightEdge);
void updateLaneLengths(Lane &lane);
void interpolateHeadingParametricPoints(physics::Distance const &length,
                                        physics::ParametricValue const &headingT,
                                        physics::ParametricValue &longTStart,
                                        physics::ParametricValue &longTEnd);

} // namespace lane
} // namespace map
} // namespace ad
