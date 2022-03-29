/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2019-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#pragma once

#include <algorithm>
#include "opendrive/geometry/Geometry.h"
#include "opendrive/types.hpp"

namespace opendrive {
namespace geometry {

// we have AD driving maps, so less than 1 mm segments is actually ridicoulous
const double MinimumSegmentLength = 1e-3;

/**
 * @brief Searches a road by id. Returns a const iterator to the road.
 *
 */
inline std::vector<::opendrive::RoadInformation>::const_iterator findRoad(opendrive::OpenDriveData const &mapData,
                                                                          int roadId)
{
  auto compare = [&roadId](::opendrive::RoadInformation const &road) { return road.attributes.id == roadId; };
  return std::find_if(mapData.roads.begin(), mapData.roads.end(), compare);
}

/**
 * @brief Searches a road by id. Returns an iterator to the road.
 *
 */
inline std::vector<::opendrive::RoadInformation>::iterator findRoad(opendrive::OpenDriveData &mapData, int roadId)
{
  auto compare = [&roadId](::opendrive::RoadInformation const &road) { return road.attributes.id == roadId; };
  return std::find_if(mapData.roads.begin(), mapData.roads.end(), compare);
}

/**
 * @brief Calculates the parametric position of the start position relative to the lane section boundaries.
 *
 */
inline double parametricPosition(double startPosition, LaneSection const &laneSection)
{
  return (startPosition - laneSection.start_position) / (laneSection.end_position - laneSection.start_position);
}

/**
 * @brief Generates a full geometry description of the OpenDRIVE data.
 * The OpenDRIVE data geometry is sampled, and left and right borders are created for every lane.
 * All the lane's successors, predecessors, neighbors are calculated.
 * Junction lanes are tested for overlaps.
 * The landmark's positions are generated in the global reference frame and signal references are created
 * for all the lanes.
 *
 * @param[in] overlapMargin margin the lanes are narrowed when calculating overlaps.
 *
 */
bool GenerateGeometry(opendrive::OpenDriveData &open_drive_data, double const overlapMargin);
}
}
