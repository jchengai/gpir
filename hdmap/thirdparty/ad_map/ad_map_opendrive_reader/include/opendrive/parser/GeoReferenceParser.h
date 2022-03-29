/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 * Copyright (C) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#pragma once

#include <string>
#include "opendrive/types.hpp"

namespace opendrive {
namespace parser {

class GeoReferenceParser
{
public:
  static ::opendrive::geom::GeoLocation Parse(const std::string &geo_reference_string);
};

} // parser
} // opendrive
