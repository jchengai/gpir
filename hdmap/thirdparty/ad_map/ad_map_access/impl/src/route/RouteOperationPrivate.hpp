// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/Types.hpp"
#include "ad/map/route/Types.hpp"

namespace ad {
namespace map {
namespace route {

void updateRouteLaneOffset(bool const rightNeighbor, RouteLaneOffset &routeLaneOffset, FullRoute &route);
void alignRouteStartingPoints(point::ParaPoint const &alignmentParaPoint, route::FullRoute &route);
void alignRouteEndingPoints(point::ParaPoint const &alignmentParaPoint, route::FullRoute &route);

} // namespace route
} // namespace map
} // namespace ad
