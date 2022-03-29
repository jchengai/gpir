// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/point/ENUOperation.hpp"

#include "ad/map/point/EdgeOperation.hpp"

namespace ad {
namespace map {
namespace point {

physics::Distance calcLength(ENUEdge const &edge) {
  return calculateEdgeLength(edge);
}

}  // namespace point
}  // namespace map
}  // namespace ad
